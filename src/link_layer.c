// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include "alarm_sigaction.h"

#include <signal.h>
#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

// Definição dos bytes da trama segundo o Guião
#define FLAG 0x7E
#define A_TX 0x03 // Comandos do Emissor ou Respostas do Recetor
#define A_RX 0x01 // Comandos do Recetor ou Respostas do Emissor
#define C_SET 0x03
#define C_UA 0x07

int global_timeout = 0;
int global_nRetransmissions = 0;

// Enumeração para a Máquina de Estados
typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE
} State;

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters) {
    global_timeout = llParameters.timeout;
    global_nRetransmissions = llParameters.nRetransmissions;

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);
    State state = START;
    unsigned char byte;
        struct sigaction act = {0};
        act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        exit(1);
    }
    alarmEnabled = FALSE;
    alarmCount = 0;
    // 1. Construir e enviar a trama SET
    unsigned char setFrame[5];
    setFrame[0] = FLAG;
    setFrame[1] = A_TX; // 0x03
    setFrame[2] = C_SET; // 0x03
    setFrame[3] = setFrame[1] ^ setFrame[2]; // BCC1
    setFrame[4] = FLAG;

    while (alarmCount <= llParameters.nRetransmissions && state != STOP_STATE){
        printf("Transmissor: A enviar trama SET...\n");
        int bytesWritten = writeBytesSerialPort(setFrame, 5);

        for(int i = 0; i < 5; i++) {
            printf("Enviado: 0x%02X\n", setFrame[i]);
        }

        printf("Transmissor: Enviou trama SET (%d bytes)\n", bytesWritten);

        alarm(llParameters.timeout);
        alarmEnabled = 1;

        printf("Transmissor: A aguardar trama UA...\n");
        while (state != STOP_STATE && alarmEnabled == 1)
        {
            if (readByteSerialPort(&byte) > 0)
            {
                printf("Transmissor leu byte: 0x%02X\n", byte); // Print do byte recebido
                
                switch (state)
                {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == A_TX) state = A_RCV; // Respostas do recetor usam A=0x03
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case A_RCV:
                        if (byte == C_UA) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_TX ^ C_UA)) state = BCC_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC_OK:
                        if (byte == FLAG) state = STOP_STATE;
                        else state = START;
                        break;
                    default:
                        break;
                }
            }
        }
    }
    if (state != STOP_STATE){
        fprintf(stderr, "Transmissor: ERRO - nenhuma trama UA recebida apos %d tentativas\n", llParameters.nRetransmissions + 1);
        closeSerialPort();
        return -1;
    }

    printf("Transmissor: Trama UA recebida com sucesso! Ligacao estabelecida.\n");

    // NOTA: A porta série NÃO é fechada aqui. Fica aberta para o llSend().
    return 0;
}

int llOpenRx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // 1. Ler a trama SET usando uma Máquina de Estados
    State state = START;
    unsigned char byte;
    
    printf("Recetor: A aguardar trama SET...\n");
    while (state != STOP_STATE)
    {
        if (readByteSerialPort(&byte) > 0)
        {
            printf("Recetor leu byte: 0x%02X\n", byte); // Print do byte recebido

            switch (state)
            {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == A_TX) state = A_RCV; // Emissor envia com A=0x03
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case A_RCV:
                    if (byte == C_SET) state = C_RCV;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case C_RCV:
                    if (byte == (A_TX ^ C_SET)) state = BCC_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC_OK:
                    if (byte == FLAG) state = STOP_STATE;
                    else state = START;
                    break;
                default:
                    break;
            }
        }
    }

    printf("Recetor: Trama SET recebida corretamente!\n");

    // 2. Construir e enviar a resposta UA
    unsigned char uaFrame[5];
    uaFrame[0] = FLAG;
    uaFrame[1] = A_TX; // Respostas do recetor também usam A=0x03
    uaFrame[2] = C_UA; // 0x07
    uaFrame[3] = uaFrame[1] ^ uaFrame[2]; // BCC1
    uaFrame[4] = FLAG;

    printf("Recetor: A enviar trama UA...\n");
    for(int i = 0; i < 5; i++) {
        printf("Enviado: 0x%02X\n", uaFrame[i]);
    }

    int bytesWritten = writeBytesSerialPort(uaFrame, 5);
    printf("Recetor: Enviou trama UA (%d bytes). Ligacao estabelecida.\n", bytesWritten);

    // NOTA: A porta série NÃO é fechada aqui. Fica aberta para o llReceive().
    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    static int tx_ns = 0; // Variável estática para alternar o N(s) entre 0 e 1 a cada envio com sucesso

    // 1. Calcular C e BCC1
    unsigned char c_byte = (tx_ns == 0) ? 0x00 : 0x80;
    unsigned char bcc1 = A_TX ^ c_byte;

    // 2. Calcular BCC2 (D1 ^ D2 ^ ... ^ Dn) antes do byte stuffing
    unsigned char bcc2 = buf[0];
    for (int i = 1; i < bufSize; i++) {
        bcc2 ^= buf[i];
    }

    // 3. Alocar espaço para a nova trama com stuffing 
    // Tamanho máximo teórico: F(1) + A(1) + C(1) + BCC1(1) + Dados_Stuffed(bufSize * 2) + BCC2_Stuffed(2) + F(1)
    int max_frame_size = 5 + (bufSize + 1) * 2;
    unsigned char *frame = (unsigned char *)malloc(max_frame_size);
    if (frame == NULL) return -1;

    int frame_idx = 0;
    frame[frame_idx++] = FLAG;
    frame[frame_idx++] = A_TX;
    frame[frame_idx++] = c_byte;
    frame[frame_idx++] = bcc1;

    // Função de Byte Stuffing para o buffer de dados
    for (int i = 0; i < bufSize; i++) {
        if (buf[i] == FLAG) {
            frame[frame_idx++] = 0x7D;
            frame[frame_idx++] = 0x5E;
        } else if (buf[i] == 0x7D) {
            frame[frame_idx++] = 0x7D;
            frame[frame_idx++] = 0x5D;
        } else {
            frame[frame_idx++] = buf[i];
        }
    }

    // Byte Stuffing para o BCC2
    if (bcc2 == FLAG) {
        frame[frame_idx++] = 0x7D;
        frame[frame_idx++] = 0x5E;
    } else if (bcc2 == 0x7D) {
        frame[frame_idx++] = 0x7D;
        frame[frame_idx++] = 0x5D;
    } else {
        frame[frame_idx++] = bcc2;
    }

    frame[frame_idx++] = FLAG; // FLAG de fecho
    int frame_size = frame_idx;

    // 4. Lógica de Envio e Retransmissões (Stop-and-Wait)
    int frame_accepted = 0;
    alarmCount = 0;

    unsigned char expected_rr = (tx_ns == 0) ? 0xAB : 0xAA; // RR1 se N(s)=0, RR0 se N(s)=1
    unsigned char expected_rej = (tx_ns == 0) ? 0x54 : 0x55; // REJ0 se N(s)=0, REJ1 se N(s)=1

    while (alarmCount <= global_nRetransmissions && !frame_accepted) {
        writeBytesSerialPort(frame, frame_size);
        
        alarmEnabled = 1;
        alarm(global_timeout);

        State state = START;
        unsigned char byte;
        unsigned char control_received = 0;

        // 5. Máquina de Estados para receção da confirmação
        while (state != STOP_STATE && alarmEnabled == 1) {
            if (readByteSerialPort(&byte) > 0) {
                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == A_TX) state = A_RCV; 
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case A_RCV:
                        if (byte == expected_rr || byte == expected_rej) {
                            control_received = byte;
                            state = C_RCV;
                        } 
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_TX ^ control_received)) state = BCC_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC_OK:
                        if (byte == FLAG) {
                            state = STOP_STATE;
                            if (control_received == expected_rr) {
                                frame_accepted = 1;
                            } else if (control_received == expected_rej) {
                                // Foi recebido um REJ (Negative ACK) -> Forçar retransmissão imediata
                                alarm(0); 
                                alarmEnabled = 0; 
                            }
                        }
                        else state = START;
                        break;
                    default:
                        break;
                }
            }
        }
    }

    free(frame);
    alarm(0); // Desativar alarme de segurança

    if (!frame_accepted) {
        return -1; // Falhou após exceder o limite de retransmissões
    }

    tx_ns = (tx_ns == 0) ? 1 : 0; // Alternar o número de sequência para a próxima trama
    return bufSize;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    static int rx_nr = 0; // Próximo número de sequência N(s) esperado (0 ou 1)
    
    unsigned char byte, c_byte;
    State state = START;
    
    // Alocar um buffer temporário largo o suficiente para receber os dados
    // (inclui espaço para o BCC2 no final).
    int temp_buf_size = 2048; 
    unsigned char *temp_buffer = (unsigned char *)malloc(temp_buf_size);
    if (!temp_buffer) return -1;

    int packetSize = 0;
    int is_escape = 0;

    // Fica no ciclo até conseguir receber uma trama válida corretamente
    while (1) {
        state = START;
        packetSize = 0;
        is_escape = 0;
        int frame_complete = 0;

        // Máquina de estados para ler uma trama de Informação (I)
        while (!frame_complete) {
            if (readByteSerialPort(&byte) > 0) {
                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == A_TX) state = A_RCV; // A_TX = 0x03, usado em Comandos do Emissor
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case A_RCV:
                        if (byte == 0x00 || byte == 0x80) { // N(s)=0 ou N(s)=1
                            c_byte = byte;
                            state = C_RCV;
                        } 
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_TX ^ c_byte)) { // Validação do BCC1
                            state = BCC_OK;
                        } 
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START; // Cabeçalho errado, ignora a trama
                        break;
                    case BCC_OK:
                        // A partir daqui estamos a ler os Dados + BCC2
                        if (byte == FLAG) {
                            frame_complete = 1; // Chegamos ao fim da trama
                        } else if (byte == 0x7D) {
                            is_escape = 1; // Encontramos o byte de escape
                        } else {
                            if (is_escape) {
                                // Destuffing: faz XOR com 0x20
                                temp_buffer[packetSize++] = byte ^ 0x20; 
                                is_escape = 0;
                            } else {
                                temp_buffer[packetSize++] = byte;
                            }
                        }
                        break;
                    default:
                        break;
                }
            }
        }

        // Se a trama estiver completa e contiver dados (pelo menos 1 byte do BCC2)
        if (packetSize > 0) {
            int data_length = packetSize - 1; // O último byte lido é o BCC2
            unsigned char received_bcc2 = temp_buffer[data_length];
            
            // Recalcular o BCC2
            unsigned char calc_bcc2 = temp_buffer[0];
            for (int i = 1; i < data_length; i++) {
                calc_bcc2 ^= temp_buffer[i];
            }

            int ns_received = (c_byte == 0x80) ? 1 : 0;
            
            // Decidir como responder (RR ou REJ) de acordo com o guião
            if (calc_bcc2 == received_bcc2) { // Dados recebidos sem erros
                if (ns_received == rx_nr) {
                    // É a trama que estávamos à espera (Nova Trama)
                    rx_nr = (rx_nr == 0) ? 1 : 0; // Avança o N(r) esperado
                    
                    // Copia os dados validados para o array da aplicação
                    for (int i = 0; i < data_length; i++) {
                        packet[i] = temp_buffer[i];
                    }
                    
                    // Responde com RR (Positive ACK) pedindo a *próxima* trama
                    unsigned char rr_c = (rx_nr == 0) ? 0xAA : 0xAB; // RR0 (0xAA) ou RR1 (0xAB)
                    unsigned char rr_frame[5] = {FLAG, A_TX, rr_c, A_TX ^ rr_c, FLAG};
                    writeBytesSerialPort(rr_frame, 5);
                    
                    free(temp_buffer);
                    return data_length; // Retorna à aplicação
                } else {
                    // É uma Trama Duplicada (o RR anterior deve ter-se perdido)
                    // Descarta os dados, mas envia RR indicando qual é a trama que o recetor quer receber
                    unsigned char rr_c = (rx_nr == 0) ? 0xAA : 0xAB; 
                    unsigned char rr_frame[5] = {FLAG, A_TX, rr_c, A_TX ^ rr_c, FLAG};
                    writeBytesSerialPort(rr_frame, 5);
                    // Continua no while(1) à espera da trama correta
                }
            } else { // Erro nos Dados (BCC2 Falhou)
                if (ns_received == rx_nr) {
                    // É a trama correta mas com erros. Envia REJ (Negative ACK)
                    unsigned char rej_c = (rx_nr == 0) ? 0x54 : 0x55; // REJ0 (0x54) ou REJ1 (0x55)
                    unsigned char rej_frame[5] = {FLAG, A_TX, rej_c, A_TX ^ rej_c, FLAG};
                    writeBytesSerialPort(rej_frame, 5);
                } else {
                    // É um duplicado que ainda por cima vem com erros. Pede o N(r) correto usando RR.
                    unsigned char rr_c = (rx_nr == 0) ? 0xAA : 0xAB; 
                    unsigned char rr_frame[5] = {FLAG, A_TX, rr_c, A_TX ^ rr_c, FLAG};
                    writeBytesSerialPort(rr_frame, 5);
                }
            }
        }
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    // TODO: Implement this function (Fase de Terminação)
    // Aqui irás enviar o DISC, ler DISC, enviar UA e finalmente fechar a porta.
    return 0;
}

int llCloseRx()
{
    // TODO: Implement this function (Fase de Terminação)
    // Aqui irás ler o DISC, enviar DISC, ler UA e finalmente fechar a porta.
    return 0;
}

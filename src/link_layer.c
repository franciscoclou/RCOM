// RCOM 2026/2027
//
// Link layer protocol implementation

#define _POSIX_SOURCE 1 // POSIX compliant source (must come before any #include)

#include "link_layer.h"
#include "serial_port.h"
#include "alarm_sigaction.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// MISC
#define BUF_SIZE 256

// Definição dos bytes da trama segundo o Guião
#define FLAG 0x7E
#define A_TX 0x03 // Comandos do Emissor ou Respostas do Recetor
#define A_RX 0x01 // Comandos do Recetor ou Respostas do Emissor
#define C_SET 0x03
#define C_UA 0x07

#define ESC 0x7D
#define MAX_RX_BUF 2048

// alarmEnabled e alarmCount são definidos em alarm_sigaction.c
// (alarm_sigaction.h deve declará-los como "extern int ...;")
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

// Envia uma trama de supervisão/não numerada (5 bytes) com A = A_TX
static void sendSupervisionFrame(unsigned char control)
{
    unsigned char frame[5] = {FLAG, A_TX, control, (unsigned char)(A_TX ^ control), FLAG};
    writeBytesSerialPort(frame, 5);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
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
    alarmEnabled = 0;
    alarmCount = 0;

    // 1. Construir a trama SET
    unsigned char setFrame[5];
    setFrame[0] = FLAG;
    setFrame[1] = A_TX;
    setFrame[2] = C_SET;
    setFrame[3] = setFrame[1] ^ setFrame[2]; // BCC1
    setFrame[4] = FLAG;

    while (alarmCount <= llParameters.nRetransmissions && state != STOP_STATE)
    {
        state = START; // recomeça a máquina de estados em cada tentativa

        printf("Transmissor: A enviar trama SET...\n");
        int bytesWritten = writeBytesSerialPort(setFrame, 5);
        printf("Transmissor: Enviou trama SET (%d bytes)\n", bytesWritten);

        alarmEnabled = 1;
        alarm(llParameters.timeout);

        printf("Transmissor: A aguardar trama UA...\n");
        while (state != STOP_STATE && alarmEnabled == 1)
        {
            if (readByteSerialPort(&byte) > 0)
            {
                printf("Transmissor leu byte: 0x%02X\n", byte);

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

    alarm(0); // desativa o alarme pendente

    if (state != STOP_STATE)
    {
        fprintf(stderr, "Transmissor: ERRO - nenhuma trama UA recebida apos %d tentativas\n",
                llParameters.nRetransmissions + 1);
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
            printf("Recetor leu byte: 0x%02X\n", byte);

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

    // 2. Enviar a resposta UA
    printf("Recetor: A enviar trama UA...\n");
    sendSupervisionFrame(C_UA);
    printf("Recetor: Enviou trama UA. Ligacao estabelecida.\n");

    // NOTA: A porta série NÃO é fechada aqui. Fica aberta para o llReceive().
    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    if (buf == NULL || bufSize <= 0) return -1;

    static int tx_ns = 0; // alterna entre 0 e 1 a cada envio com sucesso

    // 1. Calcular C e BCC1
    unsigned char c_byte = (tx_ns == 0) ? 0x00 : 0x80;
    unsigned char bcc1 = A_TX ^ c_byte;

    // 2. Calcular BCC2 (D1 ^ D2 ^ ... ^ Dn) antes do byte stuffing
    unsigned char bcc2 = 0;
    for (int i = 0; i < bufSize; i++) {
        bcc2 ^= buf[i];
    }

    // 3. Alocar espaço para a trama com stuffing
    // F + A + C + BCC1 + Dados_Stuffed(bufSize*2) + BCC2_Stuffed(2) + F
    int max_frame_size = 5 + (bufSize + 1) * 2;
    unsigned char *frame = (unsigned char *)malloc(max_frame_size);
    if (frame == NULL) return -1;

    int frame_idx = 0;
    frame[frame_idx++] = FLAG;
    frame[frame_idx++] = A_TX;
    frame[frame_idx++] = c_byte;
    frame[frame_idx++] = bcc1;

    // Byte stuffing dos dados
    for (int i = 0; i < bufSize; i++) {
        if (buf[i] == FLAG) {
            frame[frame_idx++] = ESC;
            frame[frame_idx++] = 0x5E;
        } else if (buf[i] == ESC) {
            frame[frame_idx++] = ESC;
            frame[frame_idx++] = 0x5D;
        } else {
            frame[frame_idx++] = buf[i];
        }
    }

    // Byte stuffing do BCC2
    if (bcc2 == FLAG) {
        frame[frame_idx++] = ESC;
        frame[frame_idx++] = 0x5E;
    } else if (bcc2 == ESC) {
        frame[frame_idx++] = ESC;
        frame[frame_idx++] = 0x5D;
    } else {
        frame[frame_idx++] = bcc2;
    }

    frame[frame_idx++] = FLAG; // FLAG de fecho
    int frame_size = frame_idx;

    // 4. Envio e retransmissões (Stop-and-Wait)
    int frame_accepted = 0;
    alarmCount = 0;

    unsigned char expected_rr  = (tx_ns == 0) ? 0xAB : 0xAA; // RR1 se N(s)=0, RR0 se N(s)=1
    unsigned char expected_rej = (tx_ns == 0) ? 0x54 : 0x55; // REJ0 se N(s)=0, REJ1 se N(s)=1

    while (alarmCount <= global_nRetransmissions && !frame_accepted) {
        writeBytesSerialPort(frame, frame_size);

        alarmEnabled = 1;
        alarm(global_timeout);

        State state = START;
        unsigned char byte;
        unsigned char control_received = 0;

        // 5. Máquina de estados para a confirmação (RR/REJ)
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
                                // REJ -> retransmissão imediata (conta como tentativa)
                                alarm(0);
                                alarmEnabled = 0;
                                alarmCount++;
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
    alarm(0);

    if (!frame_accepted) {
        return -1; // Falhou após exceder o limite de retransmissões
    }

    tx_ns = (tx_ns == 0) ? 1 : 0; // Alterna N(s) para a próxima trama
    return bufSize;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    static int rx_nr = 0; // Próximo N(s) esperado (0 ou 1)

    unsigned char byte;
    unsigned char c_byte = 0;
    State state;

    // Buffer temporário (dados + BCC2)
    unsigned char *temp_buffer = (unsigned char *)malloc(MAX_RX_BUF);
    if (!temp_buffer) return -1;

    int packetSize = 0;
    int is_escape = 0;
    int overflow = 0;

    // Fica no ciclo até receber uma trama válida
    while (1) {
        state = START;
        packetSize = 0;
        is_escape = 0;
        overflow = 0;
        c_byte = 0;
        int frame_complete = 0;

        // Máquina de estados para ler uma trama
        while (!frame_complete) {
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
                        // I(0), I(1) ou SET (caso o UA do llOpen se tenha perdido)
                        if (byte == 0x00 || byte == 0x80 || byte == C_SET) {
                            c_byte = byte;
                            state = C_RCV;
                        }
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_TX ^ c_byte)) state = BCC_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC_OK:
                        // Dados + BCC2
                        if (byte == FLAG) {
                            frame_complete = 1;
                        } else if (byte == ESC) {
                            is_escape = 1;
                        } else {
                            unsigned char value = byte;
                            if (is_escape) {
                                value = byte ^ 0x20; // destuffing
                                is_escape = 0;
                            }
                            if (packetSize < MAX_RX_BUF) {
                                temp_buffer[packetSize++] = value;
                            } else {
                                overflow = 1; // trama demasiado grande
                            }
                        }
                        break;
                    default:
                        break;
                }
            }
        }

        // SET repetido: o UA anterior perdeu-se, voltar a responder
        if (c_byte == C_SET) {
            sendSupervisionFrame(C_UA);
            continue;
        }

        // Trama demasiado grande ou sem BCC2: ignora (o emissor fará timeout)
        if (overflow || packetSize < 1) continue;

        int data_length = packetSize - 1; // o último byte é o BCC2
        unsigned char received_bcc2 = temp_buffer[data_length];

        unsigned char calc_bcc2 = 0;
        for (int i = 0; i < data_length; i++) {
            calc_bcc2 ^= temp_buffer[i];
        }

        int ns_received = (c_byte == 0x80) ? 1 : 0;

        if (calc_bcc2 == received_bcc2) { // Dados sem erros
            if (ns_received == rx_nr) {
                // Trama nova e esperada
                rx_nr = (rx_nr == 0) ? 1 : 0;

                for (int i = 0; i < data_length; i++) {
                    packet[i] = temp_buffer[i];
                }

                // RR com o próximo N(r) esperado
                sendSupervisionFrame((rx_nr == 0) ? 0xAA : 0xAB);

                free(temp_buffer);
                return data_length;
            } else {
                // Duplicado: descarta e volta a pedir a trama certa
                sendSupervisionFrame((rx_nr == 0) ? 0xAA : 0xAB);
            }
        } else { // BCC2 falhou
            if (ns_received == rx_nr) {
                // Trama esperada com erros -> REJ
                sendSupervisionFrame((rx_nr == 0) ? 0x54 : 0x55);
            } else {
                // Duplicado com erros -> RR a pedir a trama certa
                sendSupervisionFrame((rx_nr == 0) ? 0xAA : 0xAB);
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
// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
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
int llOpenTx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // 1. Construir e enviar a trama SET
    unsigned char setFrame[5];
    setFrame[0] = FLAG;
    setFrame[1] = A_TX; // 0x03
    setFrame[2] = C_SET; // 0x03
    setFrame[3] = setFrame[1] ^ setFrame[2]; // BCC1
    setFrame[4] = FLAG;

    int bytesWritten = writeBytesSerialPort(setFrame, 5);
    printf("Transmissor: Enviou trama SET (%d bytes)\n", bytesWritten);

    // 2. Ler a trama UA usando uma Máquina de Estados
    State state = START;
    unsigned char byte;
    
    printf("Transmissor: A aguardar trama UA...\n");
    while (state != STOP_STATE)
    {
        if (readByteSerialPort(&byte) > 0)
        {
            switch (state)
            {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == A_TX) state = A_RCV; // Respostas do recetor usam A=0x03[cite: 2]
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
            switch (state)
            {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == A_TX) state = A_RCV; // Emissor envia com A=0x03[cite: 2]
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
    uaFrame[1] = A_TX; // Respostas do recetor também usam A=0x03[cite: 2]
    uaFrame[2] = C_UA; // 0x07
    uaFrame[3] = uaFrame[1] ^ uaFrame[2]; // BCC1
    uaFrame[4] = FLAG;

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
    // TODO: Implement this function (Fase de transferência de dados)
    return 0;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    // TODO: Implement this function (Fase de transferência de dados)
    return 0;
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
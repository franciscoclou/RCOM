// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Packet control field (C)
#define C_DATA 2
#define C_START 1
#define C_END 3

// TLV types in control packets
#define T_FILE_SIZE 0
#define T_FILE_NAME 1

// Data packet header: C + L2 + L1
#define DATA_HEADER_SIZE 3
// File bytes per data packet; can be lowered at compile time (-DMAX_DATA_SIZE=...)
// to vary the I frame size in the efficiency tests
#ifndef MAX_DATA_SIZE
#define MAX_DATA_SIZE (MAX_PAYLOAD_SIZE - DATA_HEADER_SIZE)
#endif

#if MAX_DATA_SIZE < 1 || MAX_DATA_SIZE > MAX_PAYLOAD_SIZE - DATA_HEADER_SIZE
#error "MAX_DATA_SIZE must be between 1 and MAX_PAYLOAD_SIZE - DATA_HEADER_SIZE"
#endif

// llReceive may write up to this many bytes into the packet buffer
#define RX_PACKET_BUF 2048

typedef struct
{
    long fileSize;
    char fileName[256];
} FileInfo;

// Builds a START/END control packet with the file size and file name as TLVs.
// Returns the packet size.
static int buildControlPacket(unsigned char control, long fileSize, const char *fileName,
                              unsigned char *packet)
{
    int idx = 0;
    packet[idx++] = control;

    // T=0: file size, big-endian, using the minimum number of octets (at least 1)
    int sizeLen = 1;
    for (long tmp = fileSize >> 8; tmp > 0; tmp >>= 8) sizeLen++;

    packet[idx++] = T_FILE_SIZE;
    packet[idx++] = (unsigned char)sizeLen;
    for (int i = sizeLen - 1; i >= 0; i--)
        packet[idx++] = (unsigned char)((fileSize >> (8 * i)) & 0xFF);

    // T=1: file name (L is one octet, so at most 255 chars, and must fit the payload)
    int nameLen = strlen(fileName);
    int maxNameLen = MAX_PAYLOAD_SIZE - idx - 2;
    if (nameLen > 255) nameLen = 255;
    if (nameLen > maxNameLen) nameLen = maxNameLen;

    packet[idx++] = T_FILE_NAME;
    packet[idx++] = (unsigned char)nameLen;
    memcpy(&packet[idx], fileName, nameLen);
    idx += nameLen;

    return idx;
}

// Parses the TLVs of a control packet into info.
// Returns 0 on success or -1 if the packet is malformed or has no file size.
static int parseControlPacket(const unsigned char *packet, int size, FileInfo *info)
{
    int hasSize = FALSE;
    info->fileSize = 0;
    info->fileName[0] = '\0';

    int idx = 1; // skip C
    while (idx < size)
    {
        if (idx + 2 > size) return -1;

        unsigned char type = packet[idx++];
        unsigned char len = packet[idx++];
        if (idx + len > size) return -1;

        if (type == T_FILE_SIZE)
        {
            if (len == 0 || len > sizeof(long)) return -1;
            long value = 0;
            for (int i = 0; i < len; i++)
                value = (value << 8) | packet[idx + i];
            info->fileSize = value;
            hasSize = TRUE;
        }
        else if (type == T_FILE_NAME)
        {
            memcpy(info->fileName, &packet[idx], len);
            info->fileName[len] = '\0';
        }
        // Unknown types are skipped

        idx += len;
    }

    return hasSize ? 0 : -1;
}

// Sends the already opened file (START, DATA..., END). Does not close the file.
static int transmitFile(FILE *file, const char *filename, long fileSize)
{
    unsigned char packet[MAX_PAYLOAD_SIZE];

    // Only the file name goes in the packets, not the local path
    const char *slash = strrchr(filename, '/');
    if (slash != NULL) filename = slash + 1;

    // 1. START packet
    int packetSize = buildControlPacket(C_START, fileSize, filename, packet);
    if (llSend(packet, packetSize) < 0)
    {
        fprintf(stderr, "Application: failed to send START packet\n");
        return -1;
    }
    printf("Application: START sent (file \"%s\", %ld bytes)\n", filename, fileSize);

    // 2. Data packets
    long bytesSent = 0;
    int nBytes;
    while ((nBytes = fread(&packet[DATA_HEADER_SIZE], 1, MAX_DATA_SIZE, file)) > 0)
    {
        packet[0] = C_DATA;
        packet[1] = (unsigned char)(nBytes >> 8);   // L2
        packet[2] = (unsigned char)(nBytes & 0xFF); // L1

        if (llSend(packet, nBytes + DATA_HEADER_SIZE) < 0)
        {
            fprintf(stderr, "Application: failed to send data packet\n");
            return -1;
        }

        bytesSent += nBytes;
        printf("Application: sent %ld/%ld bytes\n", bytesSent, fileSize);
    }

    if (ferror(file))
    {
        perror("fread");
        return -1;
    }

    // 3. END packet (repeats the START information)
    packetSize = buildControlPacket(C_END, fileSize, filename, packet);
    if (llSend(packet, packetSize) < 0)
    {
        fprintf(stderr, "Application: failed to send END packet\n");
        return -1;
    }
    printf("Application: END sent\n");

    return 0;
}

static int receiveFile(const char *filename)
{
    unsigned char packet[RX_PACKET_BUF];
    FileInfo startInfo, endInfo;
    int n;

    // 1. Wait for the START packet
    while (TRUE)
    {
        n = llReceive(packet);
        if (n < 0)
        {
            fprintf(stderr, "Application: llReceive failed\n");
            return -1;
        }
        if (n > 0 && packet[0] == C_START) break;

        fprintf(stderr, "Application: ignoring packet received before START\n");
    }

    if (parseControlPacket(packet, n, &startInfo) < 0)
    {
        fprintf(stderr, "Application: malformed START packet\n");
        return -1;
    }
    printf("Application: START received (file \"%s\", %ld bytes)\n",
           startInfo.fileName, startInfo.fileSize);

    FILE *file = fopen(filename, "wb");
    if (file == NULL)
    {
        perror("fopen");
        return -1;
    }

    // 2. Data packets until END
    long bytesReceived = 0;
    while (TRUE)
    {
        n = llReceive(packet);
        if (n < 0)
        {
            fprintf(stderr, "Application: llReceive failed\n");
            fclose(file);
            return -1;
        }
        if (n == 0) continue;

        if (packet[0] == C_END) break;

        if (packet[0] != C_DATA || n < DATA_HEADER_SIZE)
        {
            fprintf(stderr, "Application: ignoring unexpected packet (C=%d)\n", packet[0]);
            continue;
        }

        int k = 256 * packet[1] + packet[2];
        if (k != n - DATA_HEADER_SIZE)
        {
            fprintf(stderr, "Application: data packet size mismatch (header %d, got %d)\n",
                    k, n - DATA_HEADER_SIZE);
            fclose(file);
            return -1;
        }

        if (fwrite(&packet[DATA_HEADER_SIZE], 1, k, file) != (size_t)k)
        {
            perror("fwrite");
            fclose(file);
            return -1;
        }

        bytesReceived += k;
        printf("Application: received %ld/%ld bytes\n", bytesReceived, startInfo.fileSize);
    }

    fclose(file);

    // 3. Check END against START and the number of bytes received
    if (parseControlPacket(packet, n, &endInfo) < 0)
    {
        fprintf(stderr, "Application: malformed END packet\n");
        return -1;
    }
    printf("Application: END received\n");

    if (endInfo.fileSize != startInfo.fileSize ||
        strcmp(endInfo.fileName, startInfo.fileName) != 0)
    {
        fprintf(stderr, "Application: END packet does not match START packet\n");
        return -1;
    }

    if (bytesReceived != startInfo.fileSize)
    {
        fprintf(stderr, "Application: received %ld bytes but expected %ld\n",
                bytesReceived, startInfo.fileSize);
        return -1;
    }

    printf("Application: file \"%s\" received successfully (%ld bytes)\n",
           filename, bytesReceived);
    return 0;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    strncpy(llParameters.serialPort, serialPort, sizeof(llParameters.serialPort) - 1);
    llParameters.serialPort[sizeof(llParameters.serialPort) - 1] = '\0';

    if (strcmp(role, "tx") == 0)
    {
        // Open the file before the link so a bad filename fails without connecting
        FILE *file = fopen(filename, "rb");
        if (file == NULL)
        {
            perror("fopen");
            exit(1);
        }

        long fileSize = -1;
        if (fseek(file, 0, SEEK_END) == 0) fileSize = ftell(file);
        if (fileSize < 0 || fseek(file, 0, SEEK_SET) != 0)
        {
            perror("Application: could not get file size");
            fclose(file);
            exit(1);
        }

        if (llOpenTx(llParameters) < 0)
        {
            fprintf(stderr, "Application: llOpenTx failed\n");
            fclose(file);
            exit(1);
        }

        // Always close the link once it is open, even if the transfer failed
        int failed = FALSE;
        if (transmitFile(file, filename, fileSize) < 0)
        {
            fprintf(stderr, "Application: file transfer failed\n");
            failed = TRUE;
        }
        fclose(file);

        if (llCloseTx() < 0)
        {
            fprintf(stderr, "Application: llCloseTx failed\n");
            failed = TRUE;
        }

        if (failed) exit(1);
    }
    else if (strcmp(role, "rx") == 0)
    {
        if (llOpenRx(llParameters) < 0)
        {
            fprintf(stderr, "Application: llOpenRx failed\n");
            exit(1);
        }

        // Always close the link once it is open, even if the reception failed
        int failed = FALSE;
        if (receiveFile(filename) < 0)
        {
            fprintf(stderr, "Application: file reception failed\n");
            failed = TRUE;
        }

        if (llCloseRx() < 0)
        {
            fprintf(stderr, "Application: llCloseRx failed\n");
            failed = TRUE;
        }

        if (failed) exit(1);
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
        return;
    }
}

// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    strcpy(llParameters.serialPort, serialPort);

    if (strcmp(role, "tx") == 0)
    {
        if (llOpenTx(llParameters) < 0)
        {
            fprintf(stderr, "llopen falhou\n");
            exit(1);
        }

        // ---------------- TEST: Transmitter ----------------
        // Frame 1: contains 0x7E (FLAG) and 0x7D (ESC) to test byte stuffing
        unsigned char buf1[] = {0x01, 0x7E, 0x02, 0x7D, 0x03, 0x04, 0x7E, 0x7D, 0xFF};
        int n = llSend(buf1, sizeof(buf1));
        printf("TEST: llSend #1 returned %d (expected %zu)\n", n, sizeof(buf1));
        if (n < 0) exit(1);

        // Frame 2: plain text, tests the N(s) alternation (0 -> 1)
        unsigned char buf2[] = "Ola RCOM!";
        n = llSend(buf2, sizeof(buf2));
        printf("TEST: llSend #2 returned %d (expected %zu)\n", n, sizeof(buf2));
        if (n < 0) exit(1);

        // Frame 3: tests N(s) going back to 0
        unsigned char buf3[] = {0xAA, 0xBB, 0xCC};
        n = llSend(buf3, sizeof(buf3));
        printf("TEST: llSend #3 returned %d (expected %zu)\n", n, sizeof(buf3));
        if (n < 0) exit(1);

        printf("TEST: all frames sent\n");
        // ---------------------------------------------------
    }
    else if (strcmp(role, "rx") == 0)
    {
        if (llOpenRx(llParameters) < 0)
        {
            fprintf(stderr, "llopen falhou\n");
            exit(1);
        }

        // ---------------- TEST: Receiver ----------------
        unsigned char packet[2048];

        for (int k = 1; k <= 3; k++)
        {
            int n = llReceive(packet);
            if (n < 0)
            {
                fprintf(stderr, "TEST: llReceive #%d failed\n", k);
                exit(1);
            }

            printf("TEST: llReceive #%d got %d bytes:\n  HEX: ", k, n);
            for (int i = 0; i < n; i++) printf("%02X ", packet[i]);
            printf("\n  TXT: ");
            for (int i = 0; i < n; i++)
                printf("%c", (packet[i] >= 32 && packet[i] < 127) ? packet[i] : '.');
            printf("\n");
        }

        printf("TEST: all frames received\n");
        // ------------------------------------------------
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
        return;
    }
}
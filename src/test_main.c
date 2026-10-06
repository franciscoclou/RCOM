#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "link_layer.h"

int main(int argc, char *argv[])
{
    if (argc < 3) {
        printf("Uso: %s <porta> <tx|rx>\n", argv[0]);
        return 1;
    }

    LinkLayer ll;
    strcpy(ll.serialPort, argv[1]);
    ll.baudRate = 9600;
    ll.nRetransmissions = 3;
    ll.timeout = 3;

    if (strcmp(argv[2], "tx") == 0) {
        if (llOpenTx(ll) < 0) return 1;

        // Buffer de teste, com 0x7E e 0x7D para testar o stuffing
        unsigned char buf[] = {0x01, 0x7E, 0x02, 0x7D, 0x03, 0x04, 0x7E, 0x7D, 0xFF};
        int n = llSend(buf, sizeof(buf));
        printf("llSend devolveu %d\n", n);

        // Segunda trama, para testar a alternância de N(s)
        unsigned char buf2[] = "Olá RCOM!";
        n = llSend(buf2, sizeof(buf2));
        printf("llSend devolveu %d\n", n);
    } else {
        if (llOpenRx(ll) < 0) return 1;

        unsigned char packet[2048];

        for (int k = 0; k < 2; k++) {
            int n = llReceive(packet);
            printf("llReceive devolveu %d bytes:\n", n);
            for (int i = 0; i < n; i++) printf("0x%02X ", packet[i]);
            printf("\n");
        }
    }
    return 0;
}
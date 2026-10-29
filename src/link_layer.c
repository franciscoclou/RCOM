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

// Frame bytes, as defined in the protocol guide
#define FLAG 0x7E
#define A_TX 0x03 // Commands sent by the Transmitter or replies sent by the Receiver
#define A_RX 0x01 // Commands sent by the Receiver or replies sent by the Transmitter
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B

#define ESC 0x7D
#define MAX_RX_BUF 2048

// alarmEnabled and alarmCount are defined in alarm_sigaction.c
// (alarm_sigaction.h declares them as "extern int ...;")
int global_timeout = 0;
int global_nRetransmissions = 0;
int global_baudRate = 0;

// Next N(s) expected by the receiver (0 or 1).
// Kept outside llReceive because llCloseRx also needs it.
static int rx_nr = 0;

// States of the frame reception state machine
typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE
} State;

// Sends a supervision/unnumbered frame (5 bytes)
static void sendSupervisionFrame(unsigned char address, unsigned char control)
{
    unsigned char frame[5] = {FLAG, address, control, (unsigned char)(address ^ control), FLAG};
    writeBytesSerialPort(frame, 5);
}

// Reads a complete frame with a valid header (correct BCC1) and returns A and C.
// The data of I frames is skipped (BCC2 is not checked).
// If useAlarm is TRUE, gives up when the alarm fires.
// Returns 0 if a frame was read or -1 if the alarm fired.
static int readFrame(unsigned char *a, unsigned char *c, int useAlarm)
{
    State state = START;
    unsigned char byte;

    while (!useAlarm || alarmEnabled == 1)
    {
        if (readByteSerialPort(&byte) <= 0) continue;

        switch (state)
        {
            case START:
                if (byte == FLAG) state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if (byte == A_TX || byte == A_RX) {
                    *a = byte;
                    state = A_RCV;
                }
                else if (byte != FLAG) state = START;
                break;
            case A_RCV:
                if (byte == FLAG) state = FLAG_RCV;
                else {
                    *c = byte;
                    state = C_RCV;
                }
                break;
            case C_RCV:
                if (byte == (*a ^ *c)) state = BCC_OK;
                else if (byte == FLAG) state = FLAG_RCV;
                else state = START;
                break;
            case BCC_OK:
                // In an S/U frame the next byte is the FLAG; in an I frame
                // skip the data until the closing FLAG
                if (byte == FLAG) return 0;
                break;
            default:
                break;
        }
    }

    return -1;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    global_timeout = llParameters.timeout;
    global_nRetransmissions = llParameters.nRetransmissions;
    global_baudRate = llParameters.baudRate;

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

    // 1. Build the SET frame
    unsigned char setFrame[5];
    setFrame[0] = FLAG;
    setFrame[1] = A_TX;
    setFrame[2] = C_SET;
    setFrame[3] = setFrame[1] ^ setFrame[2]; // BCC1
    setFrame[4] = FLAG;

    while (alarmCount <= llParameters.nRetransmissions && state != STOP_STATE)
    {
        state = START; // restart the state machine on every attempt

        printf("Transmitter: sending SET frame...\n");
        int bytesWritten = writeBytesSerialPort(setFrame, 5);
        printf("Transmitter: sent SET frame (%d bytes)\n", bytesWritten);

        alarmEnabled = 1;
        alarm(llParameters.timeout);

        printf("Transmitter: waiting for UA frame...\n");
        while (state != STOP_STATE && alarmEnabled == 1)
        {
            if (readByteSerialPort(&byte) > 0)
            {
                printf("Transmitter read byte: 0x%02X\n", byte);

                switch (state)
                {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == A_TX) state = A_RCV; // Replies from the receiver use A=0x03
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

    alarm(0); // disable the pending alarm

    if (state != STOP_STATE)
    {
        fprintf(stderr, "Transmitter: ERROR - no UA frame received after %d attempts\n",
                llParameters.nRetransmissions + 1);
        closeSerialPort();
        return -1;
    }

    printf("Transmitter: UA frame received. Connection established.\n");

    // NOTE: the serial port is NOT closed here. It stays open for llSend().
    return 0;
}

int llOpenRx(LinkLayer llParameters)
{
    global_timeout = llParameters.timeout;
    global_nRetransmissions = llParameters.nRetransmissions;

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // 1. Read the SET frame using a state machine
    State state = START;
    unsigned char byte;

    printf("Receiver: waiting for SET frame...\n");
    while (state != STOP_STATE)
    {
        if (readByteSerialPort(&byte) > 0)
        {
            printf("Receiver read byte: 0x%02X\n", byte);

            switch (state)
            {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == A_TX) state = A_RCV; // The transmitter sends with A=0x03
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

    printf("Receiver: SET frame received correctly.\n");

    // 2. Send the UA reply
    printf("Receiver: sending UA frame...\n");
    sendSupervisionFrame(A_TX, C_UA);
    printf("Receiver: sent UA frame. Connection established.\n");

    // NOTE: the serial port is NOT closed here. It stays open for llReceive().
    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    if (buf == NULL || bufSize <= 0) return -1;

    static int tx_ns = 0; // toggles between 0 and 1 after every successful send

    // 1. Compute C and BCC1
    unsigned char c_byte = (tx_ns == 0) ? 0x00 : 0x80;
    unsigned char bcc1 = A_TX ^ c_byte;

    // 2. Compute BCC2 (D1 ^ D2 ^ ... ^ Dn) before byte stuffing
    unsigned char bcc2 = 0;
    for (int i = 0; i < bufSize; i++) {
        bcc2 ^= buf[i];
    }

    // 3. Allocate space for the stuffed frame
    // F + A + C + BCC1 + Stuffed_data(bufSize*2) + Stuffed_BCC2(2) + F
    int max_frame_size = 5 + (bufSize + 1) * 2;
    unsigned char *frame = (unsigned char *)malloc(max_frame_size);
    if (frame == NULL) return -1;

    int frame_idx = 0;
    frame[frame_idx++] = FLAG;
    frame[frame_idx++] = A_TX;
    frame[frame_idx++] = c_byte;
    frame[frame_idx++] = bcc1;

    // Byte stuffing of the data
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

    // Byte stuffing of BCC2
    if (bcc2 == FLAG) {
        frame[frame_idx++] = ESC;
        frame[frame_idx++] = 0x5E;
    } else if (bcc2 == ESC) {
        frame[frame_idx++] = ESC;
        frame[frame_idx++] = 0x5D;
    } else {
        frame[frame_idx++] = bcc2;
    }

    frame[frame_idx++] = FLAG; // closing FLAG
    int frame_size = frame_idx;

    // 4. Send and retransmit (Stop-and-Wait)
    int frame_accepted = 0;
    int rejCount = 0;
    alarmCount = 0;

    unsigned char expected_rr  = (tx_ns == 0) ? 0xAB : 0xAA; // RR1 if N(s)=0, RR0 if N(s)=1
    unsigned char expected_rej = (tx_ns == 0) ? 0x54 : 0x55; // REJ0 if N(s)=0, REJ1 if N(s)=1

    while (alarmCount <= global_nRetransmissions && rejCount <= 30 && !frame_accepted) {
        writeBytesSerialPort(frame, frame_size);

        alarmEnabled = 1;
        // write() only queues the frame: add the time it takes on the line
        // (10 bits per byte), otherwise at low baud rates the alarm fires before it ends
        alarm(global_timeout + frame_size * 10 / global_baudRate + 1);

        State state = START;
        unsigned char byte;
        unsigned char control_received = 0;

        // 5. State machine for the acknowledgement (RR/REJ)
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
                                // REJ -> retransmit immediately (does not count as an attempt;
                                // the retransmission limit only applies to timeouts)
                                alarm(0);
                                alarmEnabled = 0;
                                rejCount++;
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
        return -1; // Failed after exceeding the retransmission limit
    }

    tx_ns = (tx_ns == 0) ? 1 : 0; // Toggle N(s) for the next frame
    return bufSize;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{

    unsigned char byte;
    unsigned char c_byte = 0;
    State state;

    // Temporary buffer (data + BCC2)
    unsigned char *temp_buffer = (unsigned char *)malloc(MAX_RX_BUF);
    if (!temp_buffer) return -1;

    int packetSize = 0;
    int is_escape = 0;
    int overflow = 0;

    // Loop until a valid frame is received
    while (1) {
        state = START;
        packetSize = 0;
        is_escape = 0;
        overflow = 0;
        c_byte = 0;
        int frame_complete = 0;

        // State machine to read one frame
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
                        // I(0), I(1) or SET (in case the UA sent by llOpen was lost)
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
                        // Data + BCC2
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
                                overflow = 1; // frame too large
                            }
                        }
                        break;
                    default:
                        break;
                }
            }
        }

        // Repeated SET: the previous UA was lost, reply again
        if (c_byte == C_SET) {
            sendSupervisionFrame(A_TX, C_UA);
            continue;
        }

        // Frame too large or without BCC2: ignore it (the transmitter will time out)
        if (overflow || packetSize < 1) continue;

        int data_length = packetSize - 1; // the last byte is BCC2
        unsigned char received_bcc2 = temp_buffer[data_length];

        unsigned char calc_bcc2 = 0;
        for (int i = 0; i < data_length; i++) {
            calc_bcc2 ^= temp_buffer[i];
        }

        int ns_received = (c_byte == 0x80) ? 1 : 0;

        if (calc_bcc2 == received_bcc2) { // Data without errors
            if (ns_received == rx_nr) {
                // New, expected frame
                rx_nr = (rx_nr == 0) ? 1 : 0;

                for (int i = 0; i < data_length; i++) {
                    packet[i] = temp_buffer[i];
                }

                // RR with the next expected N(r)
                sendSupervisionFrame(A_TX, (rx_nr == 0) ? 0xAA : 0xAB);

                free(temp_buffer);
                return data_length;
            } else {
                // Duplicate: discard it and ask for the right frame again
                sendSupervisionFrame(A_TX, (rx_nr == 0) ? 0xAA : 0xAB);
            }
        } else { // BCC2 failed
            if (ns_received == rx_nr) {
                // Expected frame with errors -> REJ
                sendSupervisionFrame(A_TX, (rx_nr == 0) ? 0x54 : 0x55);
            } else {
                // Duplicate with errors -> RR asking for the right frame
                sendSupervisionFrame(A_TX, (rx_nr == 0) ? 0xAA : 0xAB);
            }
        }
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    unsigned char a, c;
    int discReceived = 0;
    alarmCount = 0;

    // 1. Send DISC and wait for the receiver's DISC (with retransmissions)
    while (alarmCount <= global_nRetransmissions && !discReceived)
    {
        printf("Transmitter: sending DISC frame...\n");
        sendSupervisionFrame(A_TX, C_DISC); // Command from the transmitter -> A = 0x03

        alarmEnabled = 1;
        alarm(global_timeout);

        while (alarmEnabled == 1 && !discReceived)
        {
            // The receiver's DISC is a command from the receiver -> A = 0x01
            if (readFrame(&a, &c, TRUE) == 0 && a == A_RX && c == C_DISC)
                discReceived = 1;
        }
    }

    alarm(0);

    if (!discReceived)
    {
        fprintf(stderr, "Transmitter: ERROR - no DISC frame received after %d attempts\n",
                global_nRetransmissions + 1);
        closeSerialPort();
        return -1;
    }

    printf("Transmitter: DISC frame received. Sending UA...\n");

    // 2. Send UA (reply from the transmitter -> A = 0x01)
    sendSupervisionFrame(A_RX, C_UA);

    if (closeSerialPort() < 0) return -1;

    printf("Transmitter: connection closed.\n");
    return 0;
}

int llCloseRx()
{
    unsigned char a, c;

    // The receiver also needs the alarm to retransmit its DISC
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        closeSerialPort();
        return -1;
    }

    // 1. Wait for the transmitter's DISC (A = 0x03)
    printf("Receiver: waiting for DISC frame...\n");
    while (1)
    {
        readFrame(&a, &c, FALSE);

        if (a == A_TX && c == C_DISC) break;

        // Repeated I frame: the last RR was lost, acknowledge it again
        if (a == A_TX && (c == 0x00 || c == 0x80))
            sendSupervisionFrame(A_TX, (rx_nr == 0) ? 0xAA : 0xAB);
    }

    printf("Receiver: DISC frame received.\n");

    // 2. Send DISC (command from the receiver -> A = 0x01) and wait for UA (A = 0x01)
    int uaReceived = 0;
    alarmCount = 0;

    while (alarmCount <= global_nRetransmissions && !uaReceived)
    {
        printf("Receiver: sending DISC frame...\n");
        sendSupervisionFrame(A_RX, C_DISC);

        alarmEnabled = 1;
        alarm(global_timeout);

        while (alarmEnabled == 1 && !uaReceived)
        {
            if (readFrame(&a, &c, TRUE) != 0) break;

            if (a == A_RX && c == C_UA)
                uaReceived = 1;
            else if (a == A_TX && c == C_DISC)
            {
                // Our DISC was lost and the transmitter repeated its own: resend now
                alarm(0);
                alarmEnabled = 0;
            }
        }
    }

    alarm(0);
    closeSerialPort();

    if (!uaReceived)
    {
        fprintf(stderr, "Receiver: ERROR - no UA frame received after %d attempts\n",
                global_nRetransmissions + 1);
        return -1;
    }

    printf("Receiver: UA frame received. Connection closed.\n");
    return 0;
}

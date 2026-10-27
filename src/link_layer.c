// RCOM 2026/2027
//
// Link layer protocol implementation

#define _POSIX_SOURCE 1 // POSIX compliant source (must come before any #include)
#define _POSIX_C_SOURCE 199309L // clock_gettime

#include "link_layer.h"
#include "serial_port.h"
#include "alarm_sigaction.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

// MISC
#define BUF_SIZE 256

// Print every byte read (compile with -DDEBUG to enable)
#ifdef DEBUG
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

// Simulation for the protocol efficiency characterisation (slide 31).
// Applied at the receiver to every correctly received I frame; the values can
// be changed at compile time, e.g. -DBCC2_ERROR_PROB=0.1 -DPROC_DELAY_US=50000
#ifndef BCC1_ERROR_PROB
#define BCC1_ERROR_PROB 0.0 // probability of simulating a header error
#endif
#ifndef BCC2_ERROR_PROB
#define BCC2_ERROR_PROB 0.0 // probability of simulating a data field error
#endif
#ifndef PROC_DELAY_US
#define PROC_DELAY_US 0 // processing delay per received I frame (us)
#endif

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

// Next N(s) expected by the receiver (0 or 1).
// Kept outside llReceive because llCloseRx also needs it.
static int rx_nr = 0;

// Protocol statistics (printed by llCloseTx/llCloseRx)
typedef struct {
    struct timespec start;  // time when the connection was opened
    int iFramesSent;        // I frames written (including retransmissions)
    int retransmissions;    // I frames resent (timeout or REJ)
    int timeouts;           // alarms triggered
    int rrSent, rrReceived;
    int rejSent, rejReceived;
    int iFramesReceived;    // new I frames accepted
    int duplicates;         // duplicate I frames discarded
    int simulatedErrors;    // simulated errors (BCC1_ERROR_PROB/BCC2_ERROR_PROB)
    long bytesSent;         // data bytes delivered by llSend
    long bytesReceived;     // data bytes delivered by llReceive
} Statistics;

static Statistics stats = {0};

static void startStatistics()
{
    stats = (Statistics){0};
    clock_gettime(CLOCK_MONOTONIC, &stats.start);
}

static void printStatistics(const char *role)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double elapsed = (now.tv_sec - stats.start.tv_sec) +
                     (now.tv_nsec - stats.start.tv_nsec) / 1e9;

    printf("\n===== Statistics (%s) =====\n", role);
    printf("Elapsed time:           %.3f s\n", elapsed);
    printf("I frames sent:          %d\n", stats.iFramesSent);
    printf("Retransmissions:        %d\n", stats.retransmissions);
    printf("Timeouts:               %d\n", stats.timeouts);
    printf("RR sent/received:       %d / %d\n", stats.rrSent, stats.rrReceived);
    printf("REJ sent/received:      %d / %d\n", stats.rejSent, stats.rejReceived);
    printf("I frames received:      %d\n", stats.iFramesReceived);
    printf("Duplicates discarded:   %d\n", stats.duplicates);
    printf("Simulated errors:       %d\n", stats.simulatedErrors);
    printf("Bytes sent:             %ld\n", stats.bytesSent);
    printf("Bytes received:         %ld\n", stats.bytesReceived);
    if (elapsed > 0)
        printf("Useful bitrate:         %.1f bit/s\n",
               8.0 * (stats.bytesSent + stats.bytesReceived) / elapsed);
    printf("============================\n\n");
}

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

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);
    startStatistics();
    State state = START;
    unsigned char byte;

    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        closeSerialPort();
        return -1;
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
                DEBUG_PRINT("Transmitter read byte: 0x%02X\n", byte);

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
    stats.timeouts += alarmCount;

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
    startStatistics();
    srand(time(NULL) ^ getpid()); // for the error simulation

    // 1. Read the SET frame using a state machine
    State state = START;
    unsigned char byte;

    printf("Receiver: waiting for SET frame...\n");
    while (state != STOP_STATE)
    {
        if (readByteSerialPort(&byte) > 0)
        {
            DEBUG_PRINT("Receiver read byte: 0x%02X\n", byte);

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
    int attempts = 0;
    alarmCount = 0;

    unsigned char expected_rr  = (tx_ns == 0) ? 0xAB : 0xAA; // RR1 if N(s)=0, RR0 if N(s)=1
    unsigned char expected_rej = (tx_ns == 0) ? 0x54 : 0x55; // REJ0 if N(s)=0, REJ1 if N(s)=1

    while (alarmCount <= global_nRetransmissions && !frame_accepted) {
        writeBytesSerialPort(frame, frame_size);
        stats.iFramesSent++;
        if (attempts++ > 0) stats.retransmissions++;

        alarmEnabled = 1;
        alarm(global_timeout);

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
                                stats.rrReceived++;
                                frame_accepted = 1;
                            } else if (control_received == expected_rej) {
                                stats.rejReceived++;
                                // REJ -> retransmit immediately (does not count as an attempt;
                                // the retransmission limit only applies to timeouts)
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
    alarm(0);
    stats.timeouts += alarmCount;

    if (!frame_accepted) {
        return -1; // Failed after exceeding the retransmission limit
    }

    tx_ns = (tx_ns == 0) ? 1 : 0; // Toggle N(s) for the next frame
    stats.bytesSent += bufSize;
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
        int bcc2_ok = (calc_bcc2 == received_bcc2);

        // Simulated processing delay (varies T_prop)
        if (PROC_DELAY_US > 0) {
            struct timespec delay = {PROC_DELAY_US / 1000000, (PROC_DELAY_US % 1000000) * 1000L};
            nanosleep(&delay, NULL);
        }

        // Simulated errors on a correct frame (varies FER), handled as real errors
        if (bcc2_ok) {
            if ((double)rand() / RAND_MAX < BCC1_ERROR_PROB) {
                stats.simulatedErrors++;
                continue; // header error: ignore the frame without replying
            }
            if ((double)rand() / RAND_MAX < BCC2_ERROR_PROB) {
                stats.simulatedErrors++;
                bcc2_ok = 0; // data error: follow the normal wrong-BCC2 path
            }
        }

        if (bcc2_ok) { // Data without errors
            if (ns_received == rx_nr) {
                // New, expected frame
                rx_nr = (rx_nr == 0) ? 1 : 0;

                for (int i = 0; i < data_length; i++) {
                    packet[i] = temp_buffer[i];
                }

                // RR with the next expected N(r)
                sendSupervisionFrame(A_TX, (rx_nr == 0) ? 0xAA : 0xAB);
                stats.rrSent++;
                stats.iFramesReceived++;
                stats.bytesReceived += data_length;

                free(temp_buffer);
                return data_length;
            } else {
                // Duplicate: discard it and ask for the right frame again
                sendSupervisionFrame(A_TX, (rx_nr == 0) ? 0xAA : 0xAB);
                stats.rrSent++;
                stats.duplicates++;
            }
        } else { // BCC2 failed
            if (ns_received == rx_nr) {
                // Expected frame with errors -> REJ
                sendSupervisionFrame(A_TX, (rx_nr == 0) ? 0x54 : 0x55);
                stats.rejSent++;
            } else {
                // Duplicate with errors -> RR asking for the right frame
                sendSupervisionFrame(A_TX, (rx_nr == 0) ? 0xAA : 0xAB);
                stats.rrSent++;
                stats.duplicates++;
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
    stats.timeouts += alarmCount;

    if (!discReceived)
    {
        fprintf(stderr, "Transmitter: ERROR - no DISC frame received after %d attempts\n",
                global_nRetransmissions + 1);
        closeSerialPort();
        printStatistics("Transmitter");
        return -1;
    }

    printf("Transmitter: DISC frame received. Sending UA...\n");

    // 2. Send UA (reply from the transmitter -> A = 0x01)
    sendSupervisionFrame(A_RX, C_UA);

    int ret = closeSerialPort();

    printf("Transmitter: connection closed.\n");
    printStatistics("Transmitter");
    return ret < 0 ? -1 : 0;
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
        {
            sendSupervisionFrame(A_TX, (rx_nr == 0) ? 0xAA : 0xAB);
            stats.rrSent++;
            stats.duplicates++;
        }
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
    stats.timeouts += alarmCount;
    closeSerialPort();

    if (!uaReceived)
    {
        fprintf(stderr, "Receiver: ERROR - no UA frame received after %d attempts\n",
                global_nRetransmissions + 1);
        printStatistics("Receiver");
        return -1;
    }

    printf("Receiver: UA frame received. Connection closed.\n");
    printStatistics("Receiver");
    return 0;
}

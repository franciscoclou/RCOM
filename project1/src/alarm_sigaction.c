#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "alarm_sigaction.h"

#define FALSE 0
#define TRUE 1

int alarmEnabled = FALSE;
int alarmCount = 0;

// Alarm function handler.
// This function will run whenever the signal SIGALRM is received.
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d received\n", alarmCount);
}

#ifndef ALARME_H
#define ALARME_H

#define FALSE 0
#define TRUE 1

// Declaração das variáveis globais (serão instanciadas no .c)
extern int alarmEnabled;
extern int alarmCount;

// Protótipo da função
void alarmHandler(int signal);

#endif /* ALARME_H */
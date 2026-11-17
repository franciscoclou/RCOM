/*
 * download.c - cliente FTP simples (RCOM Lab 2, Parte 1)
 *
 * Uso:   ./download ftp://[<user>:<password>@]<host>/<url-path>
 * Ex.:   ./download ftp://ftp.up.pt/pub/gnu/emacs/elisp-manual-21-2.8.tar.gz
 *        ./download ftp://user:pass@host/pasta/ficheiro
 *
 * Fases: 1 parse do URL -> 2 DNS -> 3 ligação de controlo -> 4 login
 *        -> 5 modo binário + passivo -> 6 ligação de dados -> 7 RETR
 *        -> 8 receber ficheiro -> 9 fim (226) -> 10 QUIT
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define FTP_PORT 21
#define MAX_LEN 1024

struct url {
    char user[256];
    char password[256];
    char host[256];
    char path[1024];
    char filename[1024];
    char ip[64];
};

/* ---------- 1. Parse do URL ---------- */
/* Formato: ftp://[<user>:<password>@]<host>/<url-path> */
int parse_url(const char *input, struct url *u) {
    const char *prefix = "ftp://";
    if (strncmp(input, prefix, strlen(prefix)) != 0) return -1;
    const char *p = input + strlen(prefix);

    const char *slash = strchr(p, '/');
    if (slash == NULL || *(slash + 1) == '\0') return -1;   /* falta o caminho */

    /* user:password@ só conta se o '@' aparecer antes da primeira '/' */
    const char *at = memchr(p, '@', slash - p);
    if (at != NULL) {
        const char *colon = memchr(p, ':', at - p);
        if (colon == NULL) return -1;
        snprintf(u->user, sizeof(u->user), "%.*s", (int)(colon - p), p);
        snprintf(u->password, sizeof(u->password), "%.*s", (int)(at - colon - 1), colon + 1);
        p = at + 1;
    } else {
        strcpy(u->user, "anonymous");
        strcpy(u->password, "anonymous");
    }

    snprintf(u->host, sizeof(u->host), "%.*s", (int)(slash - p), p);
    if (strlen(u->host) == 0) return -1;
    snprintf(u->path, sizeof(u->path), "%s", slash + 1);

    const char *last = strrchr(u->path, '/');
    snprintf(u->filename, sizeof(u->filename), "%s", last ? last + 1 : u->path);
    if (strlen(u->filename) == 0) return -1;
    return 0;
}

/* ---------- 2. DNS (do getip.c) ---------- */
int get_ip(const char *host, char *ip) {
    struct hostent *h = gethostbyname(host);
    if (h == NULL) { herror("gethostbyname()"); return -1; }
    strcpy(ip, inet_ntoa(*((struct in_addr *) h->h_addr)));
    return 0;
}

/* ---------- 3/6. Abrir socket TCP (do clientTCP.c) ---------- */
int connect_socket(const char *ip, int port) {
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(ip);
    server_addr.sin_port = htons(port);

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) { perror("socket()"); return -1; }
    if (connect(fd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
        perror("connect()"); close(fd); return -1;
    }
    return fd;
}

/* Lê uma linha (até '\n') da ligação de controlo, byte a byte */
int read_line(int fd, char *line, int max) {
    int n = 0;
    char c;
    while (n < max - 1) {
        int r = read(fd, &c, 1);
        if (r <= 0) return -1;
        line[n++] = c;
        if (c == '\n') break;
    }
    line[n] = '\0';
    return n;
}

/*
 * Lê uma resposta FTP completa e devolve o código (ex.: 220).
 * Resposta de várias linhas: começa com "220-" e só acaba
 * na linha que começa com "220 " (código + espaço).
 * Guarda a última linha em 'last' (precisamos dela para o 227).
 */
int read_reply(int fd, char *last) {
    char line[MAX_LEN];
    if (read_line(fd, line, sizeof(line)) < 0) return -1;
    printf("< %s", line);
    int code = atoi(line);

    if (strlen(line) > 3 && line[3] == '-') {
        char end[5];
        snprintf(end, sizeof(end), "%.3s ", line);
        do {
            if (read_line(fd, line, sizeof(line)) < 0) return -1;
            printf("< %s", line);
        } while (strncmp(line, end, 4) != 0);
    }
    if (last) strcpy(last, line);
    return code;
}

/* Envia um comando terminado em \r\n (o telnet fazia isto sozinho) */
int send_cmd(int fd, const char *cmd, const char *arg) {
    char buf[MAX_LEN];
    if (arg) snprintf(buf, sizeof(buf), "%s %s\r\n", cmd, arg);
    else     snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    if (strcmp(cmd, "PASS") == 0) printf("> PASS ****\n");
    else printf("> %s", buf);
    return write(fd, buf, strlen(buf)) == (ssize_t) strlen(buf) ? 0 : -1;
}

void fail(const char *phase) {
    fprintf(stderr, "\nERRO na fase: %s\n", phase);
    exit(1);
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Uso: %s ftp://[<user>:<password>@]<host>/<url-path>\n", argv[0]);
        return 1;
    }

    struct url u;
    char reply[MAX_LEN];

    /* 1. URL */
    if (parse_url(argv[1], &u) != 0) fail("parse do URL");
    printf("Host: %s | User: %s | Caminho: %s | Ficheiro: %s\n",
           u.host, u.user, u.path, u.filename);

    /* 2. DNS */
    if (get_ip(u.host, u.ip) != 0) fail("DNS (gethostbyname)");
    printf("IP: %s\n", u.ip);

    /* 3. Ligação de controlo + boas-vindas 220 */
    int ctrl = connect_socket(u.ip, FTP_PORT);
    if (ctrl < 0) fail("ligação de controlo");
    if (read_reply(ctrl, reply) != 220) fail("boas-vindas do servidor (esperado 220)");

    /* 4. Login: USER -> 331 (ou 230 direto), PASS -> 230 */
    send_cmd(ctrl, "USER", u.user);
    int code = read_reply(ctrl, reply);
    if (code == 331) {
        send_cmd(ctrl, "PASS", u.password);
        code = read_reply(ctrl, reply);
    }
    if (code != 230) fail("autenticação (USER/PASS)");

    /* 5. Modo binário e modo passivo */
    send_cmd(ctrl, "TYPE", "I");
    if (read_reply(ctrl, reply) != 200) fail("TYPE I (modo binário)");

    send_cmd(ctrl, "PASV", NULL);
    if (read_reply(ctrl, reply) != 227) fail("PASV (modo passivo)");

    /* 227 Entering Passive Mode (h1,h2,h3,h4,p1,p2). */
    int h1, h2, h3, h4, p1, p2;
    char *open = strchr(reply, '(');
    if (open == NULL ||
        sscanf(open, "(%d,%d,%d,%d,%d,%d)", &h1, &h2, &h3, &h4, &p1, &p2) != 6)
        fail("interpretar resposta 227");
    char data_ip[64];
    snprintf(data_ip, sizeof(data_ip), "%d.%d.%d.%d", h1, h2, h3, h4);
    int data_port = p1 * 256 + p2;
    printf("Ligação de dados: %s:%d\n", data_ip, data_port);

    /* 6. Ligação de dados */
    int data = connect_socket(data_ip, data_port);
    if (data < 0) fail("ligação de dados");

    /* 7. Pedir o ficheiro: espera 150 (ou 125) */
    send_cmd(ctrl, "RETR", u.path);
    code = read_reply(ctrl, reply);
    if (code != 150 && code != 125) fail("RETR (ficheiro não existe ou sem permissão)");

    /* 8. Ler da ligação de dados até o servidor fechar (read devolve 0) */
    FILE *f = fopen(u.filename, "wb");
    if (f == NULL) { perror("fopen()"); fail("criar ficheiro local"); }
    char buf[MAX_LEN];
    ssize_t n;
    long total = 0;
    while ((n = read(data, buf, sizeof(buf))) > 0) {
        fwrite(buf, 1, n, f);
        total += n;
    }
    fclose(f);
    close(data);
    if (n < 0) fail("receber dados");
    printf("Recebidos %ld bytes -> %s\n", total, u.filename);

    /* 9. Confirmação final 226 na ligação de controlo */
    code = read_reply(ctrl, reply);
    if (code != 226 && code != 250) fail("fim da transferência (esperado 226)");

    /* 10. QUIT */
    send_cmd(ctrl, "QUIT", NULL);
    read_reply(ctrl, reply);
    close(ctrl);

    printf("Download concluído com sucesso.\n");
    return 0;
}

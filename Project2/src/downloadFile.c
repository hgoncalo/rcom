#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "getip.h"
#include "clientTCP.h"

#define FTP_PORT 21
#define MAX_SIZE 256
#define BUFFER_SIZE 1024

struct sockaddr_in server_addr;
socklen_t addrlen = sizeof(struct sockaddr_in);
int sockfd = -1;
int filefd = -1;
char buf[BUFFER_SIZE];

struct session
{
    char username[MAX_SIZE];
    char password[MAX_SIZE];
    char host[MAX_SIZE];
    char path[MAX_SIZE];
    char filename[MAX_SIZE];
    char ip[MAX_SIZE];
};

int parse_url(const char* url, struct session* credentials)
{
    char* real_url = strstr(url, "ftp://");
    if (real_url == NULL) 
    {
        return -1;
    }
    real_url += strlen("ftp://");
    
    // User Access
    char* access = strchr(real_url, '@');
    if (access == NULL) 
    {
        // Caso anońimo
        memcpy(credentials->username, "anonymous\0", strlen("anonymous")+1);
        memcpy(credentials->password, "anonymous\0", strlen("anonymous")+1);
    }
    else
    {
        // Caso com Username e Password
        // Dividido em user:pass@host/path
        int index = 0;
        char* init = real_url;
        while (*real_url != '@')
        {
            if (*real_url == ':')
            {
                memcpy(credentials->username, init, index);
                credentials->username[index] = '\0';
                real_url++;
                init = real_url;
                index = 0;
            }
            else
            {
                index++;
                real_url++;
            }
        }
        memcpy(credentials->password, init, index);
        credentials->password[index] = '\0';

        if (strlen(credentials->username) == 0 || strlen(credentials->password) == 0)
        {
            return -1;
        }
        real_url++;
    }

    // Host
    char* path_start = strchr(real_url,'/');
    if (path_start == NULL)
    {
        return -1;
    }
    memcpy(credentials->host, real_url, path_start - real_url);
    credentials->host[path_start - real_url] = '\0';
    real_url = path_start;

    // Path
    memcpy(credentials->path, real_url, strlen(real_url));
    credentials->path[strlen(real_url)] = '\0';
    char* fname = strrchr(credentials->path, '/');
    if (fname != NULL)
    {
        fname++; // o resto já está armazenado
        memcpy(credentials->filename, fname, strlen(fname));
        credentials->filename[strlen(fname)] = '\0';
    }
    else
    {
        memcpy(credentials->filename, credentials->path, strlen(credentials->path));
        credentials->filename[strlen(credentials->path)] = '\0';
    }
    return 0;
}

int download_file(void)
{
    ssize_t bytes_received;
    ssize_t total_bytes = 0;
    ssize_t bytes_written;

    printf("[DEBUG]: Starting Download..\n");

    while ((bytes_received = recv(sockfd, buf, BUFFER_SIZE, MSG_OOB | MSG_PEEK | MSG_DONTROUTE)) > 0)
    { // nao sei se é suposto usar estas flags??
        bytes_written = write(filefd, buf, bytes_received);

        if (bytes_written != bytes_received)
        {
            perror("Error while writing the file");
            return 1;
        }

        total_bytes += bytes_received;
        printf("[DEBUG]: \rBytes Transfered: %ld", total_bytes);
        fflush(stdout);
    }

    printf("\nDownload Complete: %ld bytes\n", total_bytes);

    if (bytes_received < 0)
        return 1;

    return 0;
}

int ftp_send_command(int sockfd, const char *command, char *response, int resp_size)
{
    char cmd[256];     // hardcoded por enquanto
    char buffer[1024]; // hardcoded por enquanto
    ssize_t nbytes;

    // Adicionar \r\n ao comando
    snprintf(cmd, sizeof(cmd), "%s\r\n", command);

    // Enviar o comando
    if (send(sockfd, cmd, strlen(cmd), 0) < 0)
    {
        return -1;
    }

    // Receber resposta (uma linha)
    nbytes = recv(sockfd, buffer, sizeof(buffer) - 1, 0);
    if (nbytes <= 0)
    {
        return -1;
    }

    buffer[nbytes] = '\0';

    // Copiar a resposta para um buffer
    if (response && resp_size > 0)
    {
        strncpy(response, buffer, resp_size - 1);
        response[resp_size - 1] = '\0';
    }

    printf("> %s", cmd);
    printf("< %s", buffer);
}

int main(int argc, char *argv[]) {

    if (argc < 2) 
    {
        printf("Uso: %s ftp://[user:pass@]host/path/file\n", argv[0]);
        return 1;
    }

    // Parsing
    struct session ftp;
    printf("--> A processar URL...\n");
    if (parse_url(argv[1], &ftp) < 0) 
    {
        printf("Erro: URL inválido.\n");
        return 1;
    }
    printf("Dados: Host=%s | User=%s | Pass=%s | Path=%s | File=%s\n", ftp.host, ftp.username, ftp.password, ftp.path, ftp.filename);

    // DNS
    printf("--> A resolver DNS...\n");
    if (get_ip(ftp.host,ftp.ip) < 0) 
    {
        printf("Erro: Não foi possível resolver o hostname.\n");
        return 1;
    }
    printf("IP do Servidor: %s\n", ftp.ip);
  
  // LUCAS
  filefd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (filefd == -1)
    {
        perror("Erro ao criar arquivo");
        return EXIT_FAILURE;
    }

    // TODO: Conectar ao servidor FTP
    // sockfd = connect_to_ftp(domain, FTP_PORT);

    if (sockfd > 0)
    {

        // TODO: Enviar comandos FTP (USER, PASS, TYPE I, etc.)

        char cmd_response[256];

        // 1. Enviar USER anonymous e verificar a resposta do servidor (esperado: 331)
        ftp_send_command(sockfd, "USER anonymous", cmd_response, sizeof(cmd_response));
        if (strncmp(cmd_response, "331", 3) != 0)
        {
            fprintf(stderr, "Resposta inesperada do USER: %s\n", cmd_response);
            return -1;
        }

        // 2. Enviar PASS e verificar a resposta do servidor (esperado: 230)
        if (ftp_send_command(sockfd, "PASS anonymous@", cmd_response, sizeof(cmd_response)) != 0)
        {
            fprintf(stderr, "Erro ao enviar PASS\n");
            return -1;
        }
        if (strncmp(cmd_response, "230", 3) != 0)
        {
            fprintf(stderr, "Login falhou: %s\n", cmd_response);
            return -1;
        }

        // 3. Enviar TYPE I e verificar a resposta do servidor (esperado: 200)
        if (ftp_send_command(sockfd, "TYPE I", cmd_response, sizeof(cmd_response)) != 0)
        {
            fprintf(stderr, "Erro TYPE I\n");
            return -1;
        }
        if (strncmp(cmd_response, "200", 3) != 0)
        {
            fprintf(stderr, "Erro no modo binário: %s\n", cmd_response);
            return -1;
        }

        // 4. Enviar PASSV e verificar a resposta do servidor (esperado: 227)
        if (ftp_send_command(sockfd, "TYPE I", cmd_response, sizeof(cmd_response)) != 0)
        {
            fprintf(stderr, "Erro PASSV\n");
            return -1;
        }
        if (strncmp(cmd_response, "227", 3) != 0)
        {
            fprintf(stderr, "Erro PASSV: %s\n", cmd_response);
            return -1;
        }

        // 5. Enviar RETR filename e verificar a resposta do servidor (esperado: 227)



        if (download_file() != 0)
        {
            fprintf(stderr, "Erro na transferência\n");
        }
    }

    // Cleanup
    if (filefd != -1)
    {
        close(filefd);
    }

    if (sockfd > 0)
    {
        close(sockfd);
    }
  
    return 0;
}
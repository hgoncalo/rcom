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
#define BUFFER_SIZE 1024

struct sockaddr_in server_addr;
socklen_t addrlen = sizeof(struct sockaddr_in);

int sockfd = -1;
int filefd = -1;
char *domain = "ftp.up.pt";                                       // hardcoded por enquanto
char *path = "/pub/archlinux/archive/iso/arch-0.8-base-i686.iso"; // hardcoded por enquanto
char *filename = "arch-0.8-base-i686.iso";                        // hardcoded por enquanto

char buf[BUFFER_SIZE];

int main(int argc, char *argv[])
{

    /*
    if (argc < 3) {
        fprintf(stderr, "Uso: %s <servidor> <arquivo_local>\n", argv[0]);
        fprintf(stderr, "Exemplo: %s ftp.example.com download.zip\n", argv[0]);
        return EXIT_FAILURE;
    }


    domain = argv[1];
    path = argv[2];

    */

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

    return EXIT_SUCCESS;
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

    return 0;
}
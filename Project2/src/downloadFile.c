#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdbool.h>

#include "getip.h"
#include "clientTCP.h"

#define FTP_PORT 21
#define MAX_SIZE 256
#define BUFFER_SIZE 1024

struct sockaddr_in server_addr;
socklen_t addrlen = sizeof(struct sockaddr_in);
int sockfd = -1;
int datasockfd = -1;

struct session
{
    char username[MAX_SIZE];
    char password[MAX_SIZE];
    char host[MAX_SIZE];
    char path[MAX_SIZE];
    char filename[MAX_SIZE];
    char ip[MAX_SIZE];
    char pasv_ip[MAX_SIZE];
    int pasv_port;
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

int download_file(int datasockfd, struct session* credentials)
{
    ssize_t bytes_received;
    ssize_t total_bytes = 0;
    char buf[BUFFER_SIZE];

    printf("-> A iniciar o download do ficheiro: %s\n", credentials->filename);

    FILE *f = fopen(credentials->filename, "wb");
    if (f == NULL){return -1;}

    while ((bytes_received = read(datasockfd,buf,sizeof(buf))) > 0)
    { 
        ssize_t bytes_written = fwrite(buf,1,bytes_received,f);

        if (bytes_written != bytes_received){return 1;}

        total_bytes += bytes_received;
        printf("\r-> Baixados: %ld bytes", total_bytes);
        fflush(stdout);
    }

    printf("\n-> Download Completo: %ld bytes\n", total_bytes);
    fclose(f);
    if (bytes_received < 0){return 1;}
    return 0;
}

// Lê uma linha da resposta até encontrar um '\n'
int read_line(int sockfd, char *buffer, int buf_size)
{
    char c;
    int i = 0;
    ssize_t nbytes;

    while (i < buf_size - 1)
    {
        nbytes = recv(sockfd, &c, 1, 0);
        if (nbytes <= 0){return -1;}
        buffer[i++] = c;
        if (c == '\n'){break;}
    }

    buffer[i] = '\0'; 
    return i; 
}

int ftp_send_command(int sockfd, const char *command, char *response, int resp_size)
{
    char cmd[BUFFER_SIZE];     

    if (command != NULL)
    {
        // Adicionar \r\n ao comando
        snprintf(cmd, sizeof(cmd), "%s\r\n", command);
        printf("> %s", cmd);
        if (send(sockfd, cmd, strlen(cmd), 0) < 0){return -1;}
    }

    // Receber resposta pode implicar ler várias linhas
    bool response_end = false;
    char response_code[4];
    memset(response_code, 0, sizeof(response_code));

    while (!response_end)
    {
        if (read_line(sockfd,response,resp_size) < 0) {return -1;}
        printf("< %s", response);

        if (strlen(response) < 4) continue;

        // resposta multi-linha
        if (response[3] == '-')
        {
            if (response_code[0] == '\0')
            {
                memcpy(response_code, response, 3);
                response_code[3] = '\0';
            }
        }

        // última linha da resposta
        else if (response[3] == ' ')
        {
            if (strncmp(response, response_code, 3) == 0 || strlen(response_code) == 0)
            {
                response_end = true;
            }
        }
    }
    return 0;
}

int parse_pasv(char* response, struct session* credentials)
{
    int ip1, ip2, ip3, ip4, p1, p2;
    char *start = strchr(response, '(');

    if (start == NULL){ return -1;}
    sscanf(start, "(%d,%d,%d,%d,%d,%d)", &ip1, &ip2, &ip3, &ip4, &p1, &p2);

    char temp_ip[MAX_SIZE];
    sprintf(temp_ip, "%d.%d.%d.%d", ip1, ip2, ip3, ip4);
    memcpy(credentials->pasv_ip, temp_ip, strlen(temp_ip));
    credentials->pasv_ip[strlen(temp_ip)] = '\0';
    
    credentials->pasv_port = (p1 * 256) + p2;
    return 0;
}

int main(int argc, char *argv[]) {
    if (argc < 2) 
    {
        printf("Uso: %s ftp://[user:pass]@host/path/file\n", argv[0]);
        return -1;
    }

    // PARSING
    struct session ftp;
    printf("--> A processar URL...\n");
    if (parse_url(argv[1], &ftp) < 0) 
    {
        printf("Erro: URL inválido.\n");
        return -1;
    }
    printf("Dados: Host=%s | User=%s | Pass=%s | Path=%s | File=%s\n", ftp.host, ftp.username, ftp.password, ftp.path, ftp.filename);

    // DNS
    printf("--> A resolver DNS...\n");
    if (get_ip(ftp.host,ftp.ip) < 0) 
    {
        printf("Erro: Não foi possível resolver o hostname.\n");
        return -1;
    }
    printf("IP do Servidor: %s\n", ftp.ip);

    // CONTROL SOCKET
    printf("--> A conectar ao servidor FTP...\n");
    sockfd = connect_socket(ftp.ip, FTP_PORT);
    if (sockfd < 0) 
    {
        printf("Erro: Não foi possível conectar ao servidor FTP.\n");
        return -1;
    }
    printf("Conectado ao servidor FTP.\n");

    // INITIAL RESPONSE
    char response[BUFFER_SIZE]; // buffer para respostas do server
    if ((ftp_send_command(sockfd, NULL, response, sizeof(response)) != 0) || (strncmp(response, "220", 3) != 0))
    {
        printf("Erro: Não foi possível ler a resposta inicial do servidor.\n");
        close(sockfd);
        return -1;
    }
    printf("< %s", response);

    // FTP AUTH
    char command_buf[BUFFER_SIZE];
    snprintf(command_buf, sizeof(command_buf), "USER %s", ftp.username);
    if ((ftp_send_command(sockfd, command_buf, response, sizeof(response)) != 0) || (strncmp(response, "331", 3) != 0))
    {
        printf("Erro: Não foi possível enviar USER\n");
        close(sockfd);
        return -1;
    }

    snprintf(command_buf, sizeof(command_buf), "PASS %s", ftp.password);
    if ((ftp_send_command(sockfd, command_buf, response, sizeof(response)) != 0) || (strncmp(response, "230", 3) != 0))
    {
        printf("Erro: Não foi possível enviar PASS\n");
        close(sockfd);
        return -1;
    }

    // TYPE I
    if ((ftp_send_command(sockfd, "TYPE I", response, sizeof(response)) != 0) || (strncmp(response, "200", 3) != 0))
    {
        printf("Erro: Não foi possível ativar o TYPE I\n");
        close(sockfd);
        return -1;
    }

    // PASV
    if ((ftp_send_command(sockfd, "PASV", response, sizeof(response)) != 0) || (strncmp(response, "227", 3) != 0))
    {
        printf("Erro: Não foi possível ativar o PASV\n");
        close(sockfd);
        return -1;
    }
    parse_pasv(response,&ftp);

    // DATA SOCKET
    datasockfd = connect_socket(ftp.pasv_ip, ftp.pasv_port);
    if (datasockfd < 0) 
    {
        printf("Erro: Não foi possível abrir a Data Socket.\n");
        close(datasockfd);
        close(sockfd);
        return -1;
    }

    // RETR
    snprintf(command_buf, sizeof(command_buf), "RETR %s", ftp.path);
    if ((ftp_send_command(sockfd, command_buf, response, sizeof(response)) != 0) || ((strncmp(response, "150", 3) != 0) && (strncmp(response, "125", 3) != 0)))
    {
        printf("Erro: Não foi possível ativar o RETR\n");
        close(datasockfd);
        close(sockfd);
        return -1;
    }

    // DOWNLOAD
    if (download_file(datasockfd,&ftp) != 0)
    {
        printf("Erro: Não foi possível baixar o ficheiro\n");
        close(datasockfd);
        close(sockfd);
        return -1;
    }
    close(datasockfd);

    if ((read_line(sockfd, response, sizeof(response)) < 0) || ((strncmp(response, "226", 3) != 0) && (strncmp(response, "250", 3) != 0)))
    {
        printf("Erro: Transferência não foi completa.\n");
        close(sockfd);
        return -1;
    }

    // QUIT
    if ((ftp_send_command(sockfd, "QUIT", response, sizeof(response)) != 0) || (strncmp(response, "221", 3) != 0))
    {
        printf("Erro: Houve problema ao sair\n");
        close(sockfd);
        return -1;
    }

    if (datasockfd != -1)
    {
        close(datasockfd);
    }

    if (sockfd != -1)
    {
        close(sockfd);
    }
  
    return 0;
}
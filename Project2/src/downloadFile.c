#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "getip.h"
#include "clientTCP.h"

#define FTP_PORT 21
#define MAX_SIZE 256

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

    return 0;
}
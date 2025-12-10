#include <stdio.h>
#include <stdlib.h>
#include <netdb.h>      
#include <netinet/in.h> 
#include <arpa/inet.h>  
#include <string.h>
#include "getip.h"

int get_ip(char *hostname, char *ip_buffer) {
    struct hostent *h;

    if ((h = gethostbyname(hostname)) == NULL) {
        herror("gethostbyname()");
        return -1;
    }

    char *ip = inet_ntoa(*((struct in_addr *) h->h_addr_list[0]));
    strcpy(ip_buffer, ip);

    return 0;
}
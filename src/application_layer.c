// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <signal.h>
#include <stddef.h>

#define ERROR -1
#define CTRL_START 1
#define CTRL_END   3
#define TYPE_FILESIZE 0
#define TYPE_FILENAME 1


LinkLayer connectionParameters;
const char *file_name;
int flags, tx_fd;
off_t file_size;

unsigned char ctrl_pck[MAX_PAYLOAD_SIZE];
unsigned char data_pck[MAX_PAYLOAD_SIZE + 3];   // 1 byte C + 2 bytes L2L1 + dados

// tx aux
int b_size = 0;
int r_size = 0;

// rx aux
int rx_fsize = 0;
int p_size = 0;
char rx_fname[MAX_PAYLOAD_SIZE];
FILE *rx_fptr;

enum state {OPENFILE, START_PACKET, DATA_PACKET, END_PACKET, END};

// aux
int parse_cmdLine();
int openFile();
int buildCtrlPck(unsigned char control);
int buildDataPck(unsigned char *frag, int frag_size);
int readFragFile(unsigned char *frag);

void appStateMachine(enum state s) {
    while(s != END)
    {
        unsigned char packet[MAX_PAYLOAD_SIZE];
        unsigned char frag[MAX_PAYLOAD_SIZE];
        switch (s) {
            case OPENFILE:
                switch(connectionParameters.role)
                {
                    case LlTx:
                        if (parse_cmdLine()) return;
                        if (openFile()) return;
                        break;
                    case LlRx:
                        break;
                    default:
                        return;
                }
                if (llopen(connectionParameters) == ERROR) return;
                s = START_PACKET;
                break;
            case START_PACKET:
            
                switch(connectionParameters.role)
                {
                    case LlTx:
                        b_size = buildCtrlPck(CTRL_START);
                        if (b_size < 0) {
                            llclose();
                            s = END;
                        }
                        if (llwrite(ctrl_pck, b_size) < 0) {
                            llclose();
                            s = END;
                        }
                        s = DATA_PACKET;
                        break;
                    case LlRx:
                        // ler o pacote de controlo escrito
                        if (llread(packet) > 0)
                        {
                            unsigned char control = packet[0];
                            unsigned char *packet_aux = packet + 1;
                            if (control == CTRL_START) // start packet
                            {
                                // extract name and size
                                unsigned char l1, l2;
                                packet_aux++;
                                if (*packet_aux == TYPE_FILESIZE)
                                {
                                    packet_aux++;
                                    l1 = *packet_aux;
                                    packet_aux++;
                                
                                    // o byte size vem em L1 bytes no pacote (Temos que os percorrer)
                                    // deslocar o último byte (octet) para a esquerda e adicionar o valor do novo byte
                                    for (int i = 0; i < l1; i++)
                                    {
                                        rx_fsize = (rx_fsize << 8) | (*packet_aux);
                                        packet_aux++;
                                    }
                                
                                    if (*packet_aux == TYPE_FILENAME)
                                    {
                                        packet_aux++;
                                        l2 = *packet_aux;
                                        packet_aux++;
                                    
                                        // como o nome também vem em vários bytes, fazemos um memcpy dá região
                                        memcpy(rx_fname, packet_aux, l2);
                                    
                                        // terminar a string
                                        rx_fname[l2] = '\0';
                                    }
                                }
                            
                                // create file
                                rx_fptr = fopen(rx_fname, "wb");
                                if (rx_fptr == NULL)
                                {
                                    perror("erro ao criar ficheiro");
                                    return;
                                }
                                
                                // tudo ok
                                s = DATA_PACKET;
                            }
                        }
                        // else fica aqui a esperar pelo start packet
                        break;
                    default:
                        return;
                }
                break;
            case DATA_PACKET:
                switch(connectionParameters.role)
                {
                    case LlTx:
                        r_size = readFragFile(frag);
                        if (r_size > 0)
                        {
                            b_size = buildDataPck(frag, r_size);
                            if ((b_size < 0) || (llwrite(data_pck, b_size) < 0))
                            {
                                llclose();
                                s = END;
                            }
                        }
                        else if (r_size < 0)
                        {
                            llclose();
                            s = END;     
                        }
                        else s = END_PACKET;
                        break;
                    case LlRx:
                        // ler o pacote de controlo escrito
                        p_size = llread(packet);
                        if (p_size > 0)
                        {
                            // if END packet, go to end
                            unsigned char control = packet[0];
                            if (control == CTRL_END) // end packet
                            {
                                llclose();
                                s = END;
                            }
                            // if DATA packet
                            else if (control == 2) // não é end nem start, é dados
                            {
                                unsigned char l2 = packet[1];
                                unsigned char l1 = packet[2];
                                int d_size = (l2 * 256) + l1;
                                unsigned char *packet_aux = packet + 3;
                                fwrite(packet_aux, 1, d_size, rx_fptr);
                            }
                        }
                        break;
                    default:
                        return;
                }
                break;
            case END_PACKET:
                b_size = buildCtrlPck(CTRL_END);
                if (llwrite(ctrl_pck, b_size) < 0)
                {
                    perror("erro a enviar END packet");
                }
                llclose();
                s = END;
                break;
            default:
                return;
        }
    }
    if (s == END)
    {
        switch(connectionParameters.role)
        {
            case LlTx:
                close(tx_fd);
                tx_fd = -1;
                break;
            case LlRx:
                fclose(rx_fptr);
                break;
        }
    }
    return;
}

int parse_cmdLine();

int openFile() {
    tx_fd = open(file_name, flags);
    if (tx_fd == -1) return 1;

    struct stat st;
    if (fstat(tx_fd, &st) == -1) {
        perror("Error obtaining file size");
        close(tx_fd);
        return -1;
    }

    file_size = st.st_size;

    return 0;
}

int buildCtrlPck(unsigned char control) {
    int idx = 0;

    ctrl_pck[idx++] = control;

    //parameter size (t,l,v)
    ctrl_pck[idx++] = TYPE_FILESIZE;
    ctrl_pck[idx++] = sizeof(file_size);

    for (int i = sizeof(file_size) - 1; i >= 0; i--) {
        ctrl_pck[idx++] = (file_size >> (8 * i)) & 0xFF;
    }

    //parameter file name (t,l,v)
    //O protocolo só suporta ficheiros cujo nome tem, no máximo, 255 caracteres.
    // Se file_name for superior, o campo L (1byte) não conseguirá representá-lo, pois haverá truncamento.
    unsigned char name_len = (unsigned char) strlen(file_name);

    ctrl_pck[idx++] = TYPE_FILENAME;
    ctrl_pck[idx++] = name_len;

    memcpy(&ctrl_pck[idx], file_name, name_len);
    idx += name_len;

    return idx; //tamanho total do pacote, em bytes.
}

int buildDataPck(unsigned char *frag, int frag_size) {
    int idx = 0;

    data_pck[idx++] = 2;
    data_pck[idx++] = (frag_size >> 8) & 0xFF;  // L2
    data_pck[idx++] = frag_size & 0xFF; //L1

    memcpy(&data_pck[idx], frag, (unsigned char) frag_size);
    idx += frag_size;
    
    return idx; //tamanho total do pacote, em bytes.
}

int readFragFile(unsigned char *frag) {
    int n = read(tx_fd, frag, MAX_PAYLOAD_SIZE);
    if (n < 0) {
        perror("Error reading file");
        return -1;
    }
    return n;   //número de bytes lidos
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
    {

    strcpy(connectionParameters.serialPort, serialPort); // não podemos atribuir array a um array direto... temos que copiar a mem

    if (strcmp(role, "tx") == 0)
    {
        connectionParameters.role = LlTx;
    }
    else if (strcmp(role, "rx") == 0)
    {
        connectionParameters.role = LlRx;
    }
    else 
    {
        perror("not a valid role");
        return;
    }

    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout; 

    file_name = filename;

    appStateMachine(OPENFILE);

    //Announces to the receiver that a file is going to be sent
    //Sends a START packet with name and size of file
    // This is a type of CONTROL packet 

    //Takes the file and breaks it into smaller chunks (data fragments)
    //Packs each segment into a DATA packet by adding an header 
    //Sends each DATA packet 

    //After sending the last DATA packet, announces to the receiver that the transfer is finalised
    //Sends an END packet (another CONTROL packet)

}

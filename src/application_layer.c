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
int tx_fd;
off_t file_size;

unsigned char ctrl_pck[MAX_PAYLOAD_SIZE];
unsigned char data_pck[MAX_PAYLOAD_SIZE];   // 1 byte C + 2 bytes L2L1 + data

// Tx aux
int b_size = 0;
int r_size = 0;

// Rx aux
int rx_fsize = 0;
int p_size = 0;
char rx_fname[MAX_PAYLOAD_SIZE];
FILE *rx_fptr;
const char *rx_file_name;

enum state {OPENFILE, START_PACKET, DATA_PACKET, END_PACKET, END};

void parse_cmdLine();
int openFile();
int buildCtrlPck(unsigned char control);
int buildDataPck(unsigned char *frag, int frag_size);
int readFragFile(unsigned char *frag);

// Test to compare TX and RX
void log_packet(const char *filename, unsigned char *packet, int size) {
    FILE *f = fopen(filename, "ab"); // append, não sobrescreve
    if (!f) {
        perror("fopen");
        return;
    }

    fwrite(&size, sizeof(int), 1, f);   // Saves packet size before the content
    for(int i=0;i<size;i++){
        fprintf(f,"%02X ", packet[i]);
    }
    fprintf(f,"\n");
    fclose(f);
}

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
                        if (openFile()) return;
                        break;
                    case LlRx:
                        break;
                    default:
                        return;
                }

                if (llopen(connectionParameters))
                {
                    s = END;
                    break;
                }

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
                        else
                        {
                            log_packet("tx_log.txt", ctrl_pck, b_size); 
                        }
                        s = DATA_PACKET;

                        break;
                    case LlRx:
                        int n = llread(packet);
                        if (n > 0)
                        {
                            unsigned char control = packet[0];
                            if (control == CTRL_START)
                            {
                                rx_fptr = fopen(rx_file_name, "wb");
                                if (rx_fptr == NULL)
                                {
                                    perror("Error creating file");
                                    return;
                                }
                                printf("Created file: '%s'\n", rx_file_name);
                                s = DATA_PACKET;
                            }
                        }
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
                            else
                            {
                                log_packet("tx_log.txt", data_pck, b_size);
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
                        p_size = llread(packet);

                        if (p_size > 0)
                        {
                            log_packet("rx_log.txt", packet, p_size); 
                            unsigned char control = packet[0];

                            if (control == CTRL_END)
                            {
                                llclose();
                                s = END;
                            }
                            else if (control == 2) // 1 means startpacket, 2 means datapacket, 3 means end packet
                            {
                                unsigned char l2 = packet[1];
                                unsigned char l1 = packet[2];
                                int d_size = (l2 << 8) | l1;
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
                    perror("Error sending END Packet");
                }
                llclose();
                s = END;
                break;
            case END:

                switch(connectionParameters.role)
                {
                case LlTx:
                    close(tx_fd);
                    tx_fd = -1;
                    return;
                case LlRx:
                    fclose(rx_fptr);
                    return;
                }
            default:
                return;
        }
    }
}

void parse_cmdLine(const char *serialPort, const char *role, int baudRate,int nTries, int timeout, const char *filename) 
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

    switch(connectionParameters.role)
    {
        case LlTx:
            file_name = filename;
            break;
        case LlRx:
            rx_file_name = filename;
            break;
        default:
            return;
    }

    return;
}

int openFile() {
    tx_fd = open(file_name, O_RDONLY);
    if (tx_fd == -1) {
        perror("Error opening serial port");
        exit(1);
    }

    struct stat st;
    if (fstat(tx_fd, &st) == -1) {
        perror("Error obtaining file size");
        close(tx_fd);
        return -1;
    }

    file_size = st.st_size;

    return 0;
}

// Receives a control byte
// Builds the Control Packet
// Returns the total length of the Control Packet, in bytes
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

    return idx;
}

// Receives a pointer to a fragment of the file and it's size
// Builds the Data Packet
// Returns the total length of the Data Packet, in bytes
int buildDataPck(unsigned char *frag, int frag_size) 
{
    memset(data_pck, 0, sizeof(data_pck));
    int idx = 0;
    
    data_pck[idx++] = 2;
    data_pck[idx++] = (frag_size >> 8) & 0xFF;  // L2
    data_pck[idx++] = frag_size & 0xFF; //L1

    memcpy(&data_pck[idx], frag, frag_size);
    idx += frag_size;
    
    return idx;
}

// Receives a pointer to a vector and reads into it a fragment of the file (up to a maximum of [MAX_PAYLOAD_SIZE - 3] bytes)
// Returns the number of bytes read, or -1 in case of error
int readFragFile(unsigned char *frag) {
    int n = read(tx_fd, frag, MAX_PAYLOAD_SIZE - 3);
    if (n < 0) {
        perror("Error reading file");
        return -1;
    }
    return n;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{

    parse_cmdLine(serialPort, role, baudRate, nTries, timeout, filename);
    appStateMachine(OPENFILE);

}

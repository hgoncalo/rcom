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
#define CTRL_DP 2
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

enum state
{
    OPENFILE, 
    START_PACKET, 
    DATA_PACKET, 
    END_PACKET, 
    END
};

void parse_cmdLine();
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
                            unsigned char control = packet[0];
                            if (control == CTRL_END)
                            {
                                llclose();
                                s = END;
                            }
                            else if (control == CTRL_DP)
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
    strcpy(connectionParameters.serialPort, serialPort);
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

int buildCtrlPck(unsigned char control) {
    int idx = 0;
    ctrl_pck[idx++] = control;
    ctrl_pck[idx++] = TYPE_FILESIZE;
    ctrl_pck[idx++] = sizeof(file_size);
    for (int i = sizeof(file_size) - 1; i >= 0; i--) {
        ctrl_pck[idx++] = (file_size >> (8 * i)) & 0xFF;
    }
    unsigned char name_len = (unsigned char) strlen(file_name);
    ctrl_pck[idx++] = TYPE_FILENAME;
    ctrl_pck[idx++] = name_len;
    memcpy(&ctrl_pck[idx], file_name, name_len);
    idx += name_len;
    return idx;
}

int buildDataPck(unsigned char *frag, int frag_size) 
{
    memset(data_pck, 0, sizeof(data_pck));
    int idx = 0;
    data_pck[idx++] = 2;
    data_pck[idx++] = (frag_size >> 8) & 0xFF;
    data_pck[idx++] = frag_size & 0xFF;
    memcpy(&data_pck[idx], frag, frag_size);
    idx += frag_size;
    return idx;
}

int readFragFile(unsigned char *frag) {
    int n = read(tx_fd, frag, MAX_PAYLOAD_SIZE - 3);
    if (n < 0) {
        perror("Error reading file");
        return -1;
    }
    return n;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename)
{
    parse_cmdLine(serialPort, role, baudRate, nTries, timeout, filename);
    appStateMachine(OPENFILE);
}

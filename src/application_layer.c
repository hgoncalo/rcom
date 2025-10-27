// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>

#define CTRL_START 1
#define CTRL_END   3
#define TYPE_FILESIZE 0
#define TYPE_FILENAME 1


LinkLayer connectionParameters;
const char *file_name;
int flags, fd;
off_t file_size;

unsigned char ctrl_pck[MAX_PAYLOAD_SIZE];
unsigned char data_pck[MAX_PAYLOAD_SIZE + 3];   // 1 byte C + 2 bytes L2L1 + dados

// rx aux
int rx_fsize = 0;
unsigned char rx_fname[MAX_PAYLOAD_SIZE];
FILE *rx_fptr;

enum state {OPENFILE, START_PACKET, DATA_PACKET, END_PACKET, END};

void stateMachine(enum state s) {
    while(s != END)
    {
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
                if (llopen(connectionParameters)) return;
                s = START_PACKET;
                break;
            case START_PACKET:
                // isto não funciona!! return é sempre >0 (retorna o tamanho do ficheiro) e assim termina a conexão!
                switch(connectionParameters.role)
                {
                    case LlTx:
                        int b;
                        if (b = buildCtrlPck(CTRL_START)) {
                            llclose();
                            s = END;
                        }
                        if (llwrite(ctrl_pck, b)) {
                            llclose();
                            s = END;
                        }
                        s = DATA_PACKET;
                        break;
                    case LlRx:
                        // ler o pacote de controlo escrito
                        unsigned char packet[MAX_PAYLOAD_SIZE];
                        if (llread(packet) > 0)
                        {
                            unsigned char control = packet[0];
                            if (control == CTRL_START) // start packet
                            {
                                // extract name and size
                                unsigned char l1, l2;
                                packet++;
                                if (*packet == TYPE_FILESIZE)
                                {
                                    packet++;
                                    l1 = *packet;
                                    packet++;
                                
                                    // o byte size vem em L1 bytes no pacote (Temos que os percorrer)
                                    // deslocar o último byte (octet) para a esquerda e adicionar o valor do novo byte
                                    for (int i = 0; i < l1; i++)
                                    {
                                        rx_fsize = (rx_fsize << 8) | (*packet);
                                        packet++;
                                    }
                                
                                    if (*packet == TYPE_FILENAME)
                                    {
                                        packet++;
                                        l2 = *packet;
                                        packet++;
                                    
                                        // como o nome também vem em vários bytes, fazemos um memcpy dá região
                                        memcpy(rx_fname, packet, l2);
                                    
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
                        unsigned char frag[MAX_PAYLOAD_SIZE];
                        int r;
                        if (r = readFragFile(frag)) {
                            int b;
                            // isto não funciona!! return é sempre >0 (retorna o tamanho do ficheiro) e assim termina a conexão!
                            if (b = buildDataPck(frag, r)) {
                                llclose();
                                s = END;
                            }
                            if (llwrite(data_pck, b)) {
                                llclose();
                                s = END;
                            }
                            s = END_PACKET;
                        }
                        break;
                    case LlRx:
                        // ler o pacote de controlo escrito
                        unsigned char packet[MAX_PAYLOAD_SIZE];
                        int p_size;
                        if ((p_size = llread(packet)) > 0)
                        {
                            // if END packet, go to end
                            unsigned char control = packet[0];
                            if (control == CTRL_END) // end packet
                            {
                                llclose();
                                s = END;
                            }
                            // if DATA packet
                            else
                            {
                                fwrite(packet, 1, p_size, rx_fptr);
                            }
                        }
                        break;
                    default:
                        return;
                }
                break;
            case END_PACKET:
                int b;
                b = buildCtrlPck(CTRL_END);
                llwrite(ctrl_pck, b);
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
                close(fd);
                fd = -1;
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
    fd = open(file_name, flags);
    if (fd == -1) return 1;

    struct stat st;
    if (fstat(fd, &st) == -1) {
        perror("Error obtaining file size");
        close(fd);
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
    int n = read(fd, frag, MAX_PAYLOAD_SIZE);
    if (n < 0) {
        perror("Error reading file");
        return -1;
    }
    return n;   //número de bytes lidos
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // TODO: Implement this function
    connectionParameters.serialPort = serialPort;
    connectionParameters.role = role;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    file_name = filename;

    stateMachine(OPENFILE);





    //Announces to the receiver that a file is going to be sent
    //Sends a START packet with name and size of file
    // This is a type of CONTROL packet 






    //Takes the file and breaks it into smaller chunks (data fragments)
    //Packs each segment into a DATA packet by adding an header 
    //Sends each DATA packet 






    //After sending the last DATA packet, announces to the receiver that the transfer is finalised
    //Sends an END packet (another CONTROL packet)





}

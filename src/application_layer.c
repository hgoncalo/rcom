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
unsigned char data_pck[MAX_PAYLOAD_SIZE];   // 1 byte C + 2 bytes L2L1 + dados

// tx aux
int b_size = 0;
int r_size = 0;

// rx aux
int rx_fsize = 0;
int p_size = 0;
char rx_fname[MAX_PAYLOAD_SIZE];
FILE *rx_fptr;
const char *rx_file_name;

enum state {OPENFILE, START_PACKET, DATA_PACKET, END_PACKET, END};

// aux
void parse_cmdLine();
int openFile();
int buildCtrlPck(unsigned char control);
int buildDataPck(unsigned char *frag, int frag_size);
int readFragFile(unsigned char *frag);

// teste para comparar RX e TX
void log_packet(const char *filename, unsigned char *packet, int size) {
    FILE *f = fopen(filename, "ab"); // append, não sobrescreve
    if (!f) {
        perror("fopen");
        return;
    }

    // opcional: grava tamanho do pacote antes do conteúdo
    fwrite(&size, sizeof(int), 1, f);
    for(int i=0;i<size;i++){
        fprintf(f,"%02X ", packet[i]);
    }
    fprintf(f,"\n");
    fclose(f);
}

void appStateMachine(enum state s) {
    while(s != END)
    {
        //printf("[ASM] CURRENT STATE = %d\n", s);
        unsigned char packet[MAX_PAYLOAD_SIZE];
        unsigned char frag[MAX_PAYLOAD_SIZE];
        switch (s) {
            case OPENFILE:

                printf("[ASM] Entered state OPENFILE\n");

                switch(connectionParameters.role)
                {
                    case LlTx:
                        //parse_cmdLine();
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

                printf("[ASM] Entered state START_PACKET\n");
            
                switch(connectionParameters.role)
                {
                    case LlTx:
                        b_size = buildCtrlPck(CTRL_START);

                        //printf("[TX] Built START packet, size = %d\n", b_size);

                        if (b_size < 0) {
                            //printf("[TX] llwrite START packet failed\n");
                            llclose();
                            s = END;
                        }
                        if (llwrite(ctrl_pck, b_size) < 0) {
                            //printf("[TX] llwrite START packet failed\n");
                            llclose();
                            s = END;
                        }
                        else
                        {
                            log_packet("tx_log.txt", ctrl_pck, b_size); 
                        }
                        //printf("[TX] START packet sent\n");
                        s = DATA_PACKET;

                        break;
                    case LlRx:
                        // ler o pacote de controlo escrito
                        //printf("HERE HERE HERE\n");
                        //printf("Role: %d\n", connectionParameters.role);
                        //printf("Serial port: %s\n", connectionParameters.serialPort);
                        int n = llread(packet);
                        //printf("[VAR] PACKET READ = ");
                        //for (int i = 0; i < n; i++) {
                        //    printf("%02X ", packet[i]);
                        //}
                        //printf("\nEND OF PACKET\n");
                        if (n > 0)
                        {
                            unsigned char control = packet[0];
                            //unsigned char *packet_aux = packet + 1;
                            //printf("First bytes: %02X %02X %02X\n", packet_aux[0], packet_aux[1], packet_aux[2]);
                            if (control == CTRL_START) // start packet
                            {
                                // extract name and size
                                /*
                                                                unsigned char l1, l2;
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
                                        
                                        // OBS: NESTE MOMENTO ISTO NÃO FAZ SENTIDO, PQ NOME FICHEIRO RECEBIDO É DIF. DO ENVIADO
                                        //printf("l2 = %u\n", l2);    
                                    //
                                        //// como o nome também vem em vários bytes, fazemos um memcpy dá região
                                        //memcpy(rx_fname, packet_aux, l2);
                                    //
                                        //// terminar a string
                                        //rx_fname[l2] = '\0';
                                    }
                                }
                                */
                            
                                // create file
                                //printf("Trying to create file: '%s'\n", rx_file_name);
                                rx_fptr = fopen(rx_file_name, "wb");
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

                printf("[ASM] Entered state DATA_PACKET\n");

                switch(connectionParameters.role)
                {
                    case LlTx:
                        r_size = readFragFile(frag);

                        //printf("[TX] Read fragment: %d bytes\n", r_size);

                        if (r_size > 0)
                        {
                            b_size = buildDataPck(frag, r_size);
                            if ((b_size < 0) || (llwrite(data_pck, b_size) < 0))
                            {
                                printf("[TX] llwrite DATA packet failed\n");
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
                            //printf("[TX] size DATA packet failed\n");
                            llclose();
                            s = END;     
                        }
                        else s = END_PACKET;
                        break;
                    case LlRx:
                        // ler o pacote de controlo escrito
                        p_size = llread(packet);

                        //printf("[RX] llread returned %d bytes\n", p_size);

                        if (p_size > 0)
                        {
                            log_packet("rx_log.txt", packet, p_size); 
                            // if END packet, go to end
                            unsigned char control = packet[0];

                            //printf("[RX] Control byte = %d\n", control);
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
                printf("[ASM] Entered state END_PACKET\n");

                b_size = buildCtrlPck(CTRL_END);
                if (llwrite(ctrl_pck, b_size) < 0)
                {
                    perror("erro a enviar END packet");
                }
                llclose();
                s = END;
                break;
            case END:

                printf("[ASM] Entered state END\n");

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
            default:
                return;
        }
    }
    //printf("[ASM] Exiting appStateMachine with state = %d\n", s);
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
            perror("role errada");
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

int buildDataPck(unsigned char *frag, int frag_size) 
{
    /*
    if (frag_size > MAX_PAYLOAD_SIZE) 
    {
        frag_size = MAX_PAYLOAD_SIZE;
    }
    */

    memset(data_pck, 0, sizeof(data_pck));
    int idx = 0;
    
    data_pck[idx++] = 2;
    data_pck[idx++] = (frag_size >> 8) & 0xFF;  // L2
    data_pck[idx++] = frag_size & 0xFF; //L1

    // se tivesse unsigned char convertia o frag a 8 bits!!! (de 0 a 256 no max)
    // estavamos a forçar um inteiro 1000 a caber em 8 bits...
    memcpy(&data_pck[idx], frag, frag_size);
    idx += frag_size;
    
    return idx; //tamanho total do pacote, em bytes.
}

int readFragFile(unsigned char *frag) {
    int n = read(tx_fd, frag, MAX_PAYLOAD_SIZE - 3);
    if (n < 0) {
        perror("Error reading file");
        return -1;
    }
    return n;   //número de bytes lidos
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
    {

    parse_cmdLine(serialPort, role, baudRate, nTries, timeout, filename);

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

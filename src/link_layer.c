// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stddef.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

// MACROS
#define FLAG 0x7E
#define A_TX_C 0x03
#define A_RX_R 0x03
#define A_RX_C 0x01
#define A_TX_R 0x01
#define A_COMMAND 0
#define A_REPLY 1
#define C_SET 0x03
#define C_UA 0x07
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define C_DISC 0x0B
#define C_FRAME0 0x00
#define C_FRAME1 0x80

// Atenção: .X_! onde . é T/R (transmissor ou recetor) 
// E ! é C/R (Command ou Reply)

enum state {START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP}; 
enum alarm_state {ALARM_OPEN,ALARM_WRITE};
enum machine_state {OPEN, WRITE, READ, CLOSE};

void writeSet();
void writeUa();
void alarmHandler();
// Tx state machine
int stateMachine(unsigned char byte, enum state s, enum machine_state ms, LinkLayerRole role, int type);

unsigned char buf[BUF_SIZE] = {0};
int alarmEnabled = FALSE;
int alarmCount = 0;

// LLOPEN AUX
const char *serialPort;
LinkLayerRole x_role;

// LLWRITE AUX FUNCTIONS
int tx_fn; // sender's frame number (I(0) would be 0, varies between 0,1)
int tx_buf_size = (MAX_PAYLOAD_SIZE * 2) + 6; // currently: max buffer size that will be sent (buf's data with stuffing + 6 flags), can be ajusted to only the necessary
unsigned char *tx_frame;
unsigned char rx_answer;

void byte_stuffing(unsigned char *currentbyte, unsigned char *vector, unsigned int *size);
int byte_destuffing(unsigned char *rx_packet, unsigned int *index, unsigned int *size); 
int getAType(LinkLayerRole role, int type);

int validResponse(unsigned char byte)
{
    // Expect COMMAND (A_RX_C) from receiver for RR/REJ during WRITE
    if (stateMachine(byte,START,WRITE,x_role,A_COMMAND)) return 1;
    else
    {
        // stateMachine got an valid answer
        if (rx_answer == C_REJ0 || rx_answer == C_REJ1)return 1;
        else if (rx_answer == C_RR0 || rx_answer == C_RR1) return 0;
        else return 1;
    }
}

// LLREAD AUX
unsigned char rx_packet[MAX_PAYLOAD_SIZE * 2 + 6];
unsigned int rx_packet_len = 0; // stuffed payload length captured by state machine (includes BCC2, excludes FLAGs)

/*
// data will already be destuffed

*/

// otimizar este código numa func. única?
void writeSet()
{

    //printf("[SM] Entered writeSet()\n");


    buf[0] = FLAG;
    buf[1] = A_TX_C;
    buf[2] = C_SET;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    writeBytesSerialPort(buf, 5);
    //printf("Sent SET\n");
    sleep(1);
}

void writeUa(int type)
{
    buf[0] = FLAG;
    buf[1] = getAType(x_role,type);
    buf[2] = C_UA;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    writeBytesSerialPort(buf, 5);
    //printf("SENT UA\n");
    sleep(1);
}

void writeDisc(int type)
{
    buf[0] = FLAG;
    buf[1] = getAType(x_role,type);
    buf[2] = C_DISC;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    writeBytesSerialPort(buf, 5);
    //printf("Sent SET\n");
    sleep(1);
}

int getAType(LinkLayerRole role, int type)
{
    if (type == A_COMMAND)
    {
        if (role == LlTx) return A_TX_C;
        else return A_RX_C;
    }
    else 
    {
        if (role == LlTx) return A_TX_R;
        else return A_RX_R;
    }
}

void writeI()
{
    //printf("[SM] Entered writeI()\n");

    tx_fn = (tx_frame[2] != 0);
    writeBytesSerialPort(tx_frame,tx_buf_size);
    sleep(1);

    //printf("[VAR] I WROTE FRAME = ");
    //for (int i = 0; i < tx_buf_size; i++) {
    //    printf("%02X ", tx_frame[i]);
    //}
    //printf("\n[SM] Exited writeI()\n");
}

void writeRR()
{
    buf[0] = FLAG;
    buf[1] = A_RX_C;
    buf[2] = ((tx_fn == 0) ? C_RR1 : C_RR0); // ask for FN+1 (Ns+1)
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    writeBytesSerialPort(buf, 5);
    //printf("Sent RR\n");
    sleep(1);
}

void writeREJ()
{
    buf[0] = FLAG;
    buf[1] = A_RX_C;
    buf[2] = ((tx_fn == 0) ? C_REJ0 : C_REJ1);
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    writeBytesSerialPort(buf, 5);
    //printf("Sent REJ\n");
    sleep(1);
}

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
    //printf("Alarm #%d received\n", alarmCount);
}

int txAlarm(unsigned char* byte, enum alarm_state as)
{
    //printf("[SM] Entered txAlarm()\n");

    while (alarmCount < 4)
    {
        if (alarmEnabled == FALSE)
        {
            switch(as)
            {
                case ALARM_OPEN:
                    writeSet();
                    break;
                case ALARM_WRITE:
                    writeI();
                    break;
                default:
                    //printf("NOT A DEFINED STATE\n");
                    return 1;
            }
            alarm(3); // Set alarm to be triggered in 3s
            alarmEnabled = TRUE;
        }
        else if (alarmEnabled == TRUE)
        {
            int bytesRead = readByteSerialPort(byte);
            switch(as)
            {
                case ALARM_WRITE:
                    //printf("[SM] Entered ALARM_WRITE Case\n");
                    if (bytesRead > 0 && (validResponse(*byte) == 0))
                    {
                        //printf("EXITED ALARM");
                        alarmEnabled = FALSE;
                        alarmCount = 0;

                        //printf("[SM] txAlarm exited with 0\n");
                        return 0;
                    }
                    else
                    {
                        //printf("INVALID RESPONSE\n");
                        pause(); // espera pelo handler
                    }
                    break;
                case ALARM_OPEN:
                    //printf("[SM] Entered ALARM_OPEN Case\n");
                    if (bytesRead > 0)
                    {
                        //printf("var = 0x%02X\n", *byte);
                        //printf("EXITED ALARM");
                        alarmEnabled = FALSE;
                        alarmCount = 0;
                        //printf("[SM] txAlarm exited with 0\n");
                        return 0;
                    }
                    break;
                default:
                    //printf("NOT A DEFINED STATE\n");
                    return 1;
            }
        }
        // in the end, the alarm_handle() disabled alarmEnabled again, meaning 3 seconds have passed
        // try again for 3 more times
    }
    if (alarmCount == 4)
    {
        //printf("ALL 4 TRIES FAILED!");
        return 1;
    }
    else return 0;
}

int stateMachine(unsigned char byte, enum state s, enum machine_state ms, LinkLayerRole role, int type)
{
    unsigned char a,c;
    unsigned char p[(MAX_PAYLOAD_SIZE * 2) + 6] = {0};
    int p_index = 0;

    while (s != STOP)
    {
        //printf("[VAR] SM RECIEVED VAR = 0x%02X\n", byte);
        switch(s)
        {
            case START:
                //printf("SM START\n");
                if (byte == FLAG)
                {
                    s = FLAG_RCV;
                }
                break;
            case FLAG_RCV:
                //printf("SM FLAG_RCV\n");
                a = byte;
                // se transmissor: quero ler COMANDOS (0x01) ou RESPOSTAS (0x03) do RECIEVER, se for Rx quero ler COMANDOS (0x03) ou RESPOSTAS (0x01) do TRANSMISSOR
                if ((byte == 0x01 && role == LlTx && type == A_COMMAND) || 
                (byte == 0x03 && role == LlRx && type == A_COMMAND) || 
                (byte == 0x03 && role == LlTx && type == A_REPLY) || 
                (byte == 0x01 && role == LlRx && type == A_REPLY))
                {
                    s = A_RCV;
                }
                else if (byte != FLAG)
                {
                    s = START;
                }
                break;
            case A_RCV:
                //printf("SM A_RCV\n");
                c = byte;
                switch(ms)
                {
                    case OPEN:
                        if ((byte == 0x07 && role == LlTx) || (byte == 0x03 && role == LlRx))
                        {
                            s = C_RCV;
                        }
                        else if (byte == FLAG)
                        {
                            s = FLAG_RCV;
                        }
                        else
                        {
                            s = START;
                        }
                        break;
                    case WRITE: //i.e: write treats packets sent by LLREAD()
                        if ((byte == 0xAA) || (byte == 0xAB) || (byte == 0x54) || (byte == 0x55))
                        {
                            s = C_RCV;
                            rx_answer = byte;
                        }
                        else
                        {
                            s = START; // houve erro, voltar ao inicio
                        }
                        break;
                    case READ: // i.e: read treats packets sent by LLWRITE()
                        if ((byte == 0x00) || (byte == 0x80))
                        {
                            s = C_RCV;
                        }
                        else
                        {
                            s = START;
                        }
                        break;
                    case CLOSE:
                        // ou seja: DISC aceita, mas só aceitar UA se for o Rx a ler (ou seja, o Tx mandou)
                        if ((byte == C_DISC) || (byte == C_UA && role == LlRx))
                        {
                            s = C_RCV;
                        }
                        else
                        {
                            s = START;
                        }
                        break;
                    default:
                        //printf("NOT A DEFINED STATE\n");
                        return 1;
                }
                break;
            case C_RCV:
                //printf("SM C_RCV\n");
                if (byte == (a ^ c))
                {
                    s = BCC_OK;
                }
                else if (byte == FLAG)
                {
                    s = FLAG_RCV;
                }
                else
                {
                    s = START;
                }
                break;
            case BCC_OK:
                //printf("SM BCC\n");
                switch(ms)
                {
                    // open,write have the same cases
                    case OPEN:
                    case WRITE:
                    case CLOSE:
                        if (byte == FLAG)
                        {
                            s = STOP;
                        }
                        else
                        {
                            s = START;
                        }
                        break;
                    case READ:
                        // CHECK NS (TX_FN), if NS is the expected (new frame, no dupe) then GO, else RR(NS)

                        // se for o expectavel (tx_fn tem de ser igual ao da trama atual)
                        if (tx_fn == (c != 0))
                        {
                            // save data to be processed by llread()
                            if (byte == FLAG)
                            {
                                // HALT
                                s = STOP;

                                p[p_index] = byte;
                                p_index++;

                                // Copy stuffed payload into global buffer and record its length
                                memcpy(rx_packet, p, p_index);
                                rx_packet_len = (unsigned int)p_index;
                            }
                            else
                            {
                                // keep reading
                                p[p_index] = byte;
                                p_index++;
                            }
                        }
                        else
                        {
                            // ignore or send RR(Ns)
                            return 1;
                            // voltar ao inicio ignora a trama (o emissor toma iniciativa de mandar de novo)
                        }
                        break;
                    default:
                        //printf("NOT A DEFINED STATE\n");
                        return 1;
                }
                break;
            default:
                //printf("NOT A DEFINED STATE\n");
                return 1;
        }
        if (s != STOP)
        {
            readByteSerialPort(&byte);
            //printf("var = 0x%02X\n", byte);
        }
    }
    //printf("STATE: STOP\n");
    return 0;
}


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    unsigned char byte;
    serialPort = connectionParameters.serialPort;
    int baudRate = connectionParameters.baudRate;
    x_role = connectionParameters.role;

    if (openSerialPort(serialPort, baudRate) < 0)
    {
        perror("openSerialPort");
        exit(-1);
    }
    //printf("Serial port %s opened\n", serialPort);

    switch(connectionParameters.role)
    {
        case LlTx:
            struct sigaction act = {0};
            act.sa_handler = &alarmHandler;
            if (sigaction(SIGALRM, &act, NULL) == -1)
            {
                perror("sigaction");
                exit(1);
            }
            //printf("Alarm configured\n");

            // write set
            if (txAlarm(&byte,ALARM_OPEN))
            {
                return 1;
            };

            // read ua
            stateMachine(byte,START,OPEN,x_role,A_REPLY);
            break;
        case LlRx:
            // read set
            readByteSerialPort(&byte);
            //printf("var = 0x%02X\n", byte);
            stateMachine(byte,START,OPEN,x_role,A_COMMAND);

            // write ua
            writeUa(A_REPLY);
            break;
        default:
            //printf("Invalid role\n");
            return 1;
    }

    //printf("[SM] LLOPEN() WAS SUCCESSFUL!\n");
    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{

    //printf("[SM] Entered llwrite()\n");

    // tamanho do vetor 2*max_payload_size + 6
    // max_payload_size é 1000
    // caso todos os caracteres sejam flag,sera necessario fazer o byte stuffing de todos eles, o que faria com que o tamanho duplicasse
    // a somar a tudo isto, o vetor precisa de mais 6 bytes para as flags

    int countframe = 0;
    unsigned char bcc2 = 0x00;

    //CONFIGURAR AS FLAGS ANTES DO DATA
    unsigned int frame_size = 0;
    unsigned char frame[(2*MAX_PAYLOAD_SIZE) + 6] = {0};
    frame[0] = FLAG;
    frame[1] = A_TX_C;

    if (countframe == 0) {
        frame[2] = C_FRAME0;
        countframe = 1;
        frame[3] = A_TX_C ^ C_FRAME0; //BCC1
    } else {
        frame[2] = C_FRAME1;
        countframe = 0;
        frame[3] = A_TX_C ^ C_FRAME1; //BCC1
    }
    frame_size = 4;

    //PERCORRER OS DADOS E FAZER O BYTE STUFFING
    unsigned char currentByte;

    // Percorrer os dados e fazer o byte stuffing
    for (int i = 0; i < bufSize; i++) {
        currentByte = buf[i];
        bcc2 ^= currentByte; // calcular BCC
        //printf("[VAR] STUFFED CURRENT VAR = 0x%02X\n", currentByte);
        byte_stuffing(&currentByte, frame, &frame_size);
    }

    // adiciona o BCC2 e faz o stuffing do BCC2
    //printf("[VAR] BCC2 BEFORE STUFFING = 0x%02X\n", bcc2);
    byte_stuffing(&bcc2, frame, &frame_size);
    frame[frame_size++] = 0x7E;
    //O VETOR ESTA PRONTO

    // Escrever Frame e Receber o READ
    unsigned char byte;
    tx_buf_size = frame_size;
    tx_frame = frame;

    // implementar alarm e esperar pelo read (ACK)
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        exit(1);
    }
    //printf("Alarm configured\n");
    
    // write I and expect ACK
    if (txAlarm(&byte, ALARM_WRITE)) return 1;

    //printf("[SM] LLWRITE() WAS SUCCESSFUL!\n");
    return 0;

}

void byte_stuffing (unsigned char *currentbyte, unsigned char *vector, unsigned int *size) {

    //printf("[SM] Entered byte_stuffing()\n");

    if (*currentbyte == FLAG) {
        vector[*size] = 0x7D; //ESC
        vector[*size+1] = 0x5E;
        (*size) += 2;
    } else if (*currentbyte == 0x7D){
        vector[*size] = 0x7D; //ESC
        vector[*size+1] = 0x5D;
        (*size) += 2;
    } else {
        vector[*size] = *currentbyte;
        (*size)++;
    }

    //printf("[SM] Left byte_stuffing()\n");
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////


int byte_destuffing (unsigned char *rx_packet, unsigned int *index, unsigned int *size) {
    if (rx_packet[*index] == 0x7D) {  //se detetar 0x7D inspeciona o proximo byte
        if (*index + 1 >= (MAX_PAYLOAD_SIZE * 2 + 6)) return -1;
        if ((rx_packet[*index + 1] == 0x5E)) {
            rx_packet[*size] = 0x7E;
        } else if (rx_packet[*index + 1] == 0x5D) {
            rx_packet[*size] = 0x7D;
        }
        (*index) += 2;
        (*size)++;

    } else {
        if (rx_packet[*index] == FLAG) {
            return 1;
        }
        rx_packet[*size] = rx_packet[*index];
        (*size)++;
        (*index)++;
    }

    return 0;
}

int checkBCC2(unsigned char* rx_packet, int size)
{
    unsigned char bcc2_acc = 0;

    for (int i = 0; i < size; i++)
    {
        unsigned char byte = rx_packet[i];
        //printf("[VAR] DESTUFFED CURRENT VAR = 0x%02X\n", byte);
        bcc2_acc ^= byte; 
    }

    // i.e: tem de ser 0 porque o último byte foi o BCC2, pelo que se estiver certo (sum DATA ^ BCC2 = 0)
    // um pacote XOR com ele próprio dá 0...
    if (bcc2_acc == 0) return 0;
    else return 1;
}

int llread(unsigned char *packet)
{
    // wait for something
    unsigned char byte;
    readByteSerialPort(&byte);

    if (stateMachine(byte,START,READ,x_role,0))
    {
        writeREJ();
        //printf("[SM] LLREAD() SENT REJ VIA SM!\n");
        return -1;
    }
    //else printf("[SM] LLREAD GOT AN OKAY SM!\n");

    // received something
    int flag = 0;
    unsigned int index = 0, size = 0; // destuffed payload size will be stored in 'size'

    //printf("RX_PACKET_LEN: %d\n", rx_packet_len);
    while (flag != 1) {
        if (size >= (MAX_PAYLOAD_SIZE * 2 + 6)) return 1;
        flag = byte_destuffing(rx_packet, &index, &size);
    }

    if (checkBCC2(rx_packet,size)) // bcc2 not correct
    {
        // send REJ(tx_fn)
        writeREJ();
        //printf("[SM] LLREAD() SENT REJ!\n");
        return -1;
    }
    else
    {
        // send RR(tx_fn + 1)
        writeRR();

        // o problema vem daqui... isto prob está a truncar dados
        int copy_size = (size - 1 > MAX_PAYLOAD_SIZE) ? MAX_PAYLOAD_SIZE : size - 1;
        memcpy(packet, rx_packet, copy_size);
        //printf("[SM] LLREAD() WAS SUCCESSFUL!\n");
        return copy_size;
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    unsigned char byte;
    switch(x_role)
    {
        case LlTx:
            // write DISC (0x03)
            writeDisc(A_COMMAND);

            // read DISC + confirm
            // acabar SM para CLOSE
            readByteSerialPort(&byte);
            stateMachine(byte,START,CLOSE,x_role,A_COMMAND);

            // write ua (reply to Rx, 0x01) 
            writeUa(A_REPLY);
            break;
        case LlRx:
            // read DISC + confirm
            readByteSerialPort(&byte);
            stateMachine(byte,START,CLOSE,x_role,A_COMMAND);

            // write DISC (0x01)
            writeDisc(A_COMMAND);

            // read UA + confirm
            readByteSerialPort(&byte);
            stateMachine(byte,START,CLOSE,x_role,A_REPLY);
            break;
        default:
            //printf("Invalid role\n");
            return 1;
    }

    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        exit(-1);
    }

    //printf("Serial port %s closed\n", serialPort);
    //printf("[SM] LLCLOSE() WAS SUCCESSFUL!\n");

    return 0;
}


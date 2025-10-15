// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define BUF_SIZE 256

enum state {START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP};

void writeSet();
void writeUa();
void alarmHandler();
// Tx state machine
int stateMachine();

unsigned char buf[BUF_SIZE] = {0};
int alarmEnabled = FALSE;
int alarmCount = 0;

void writeSet()
{
    buf[0] = 0x7E;
    buf[1] = 0x03;
    buf[2] = 0x03;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = 0x7E;
    int bytes = writeBytesSerialPort(buf, BUF_SIZE);
    //printf("Sent SET\n");
    sleep(1);
}

void writeUa()
{
    buf[0] = 0x7E;
    buf[1] = 0x01;
    buf[2] = 0x07;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = 0x7E;
    int bytes = writeBytesSerialPort(buf, BUF_SIZE);
    //printf("SENT UA\n");
    sleep(1);
}

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d received\n", alarmCount);
}

int txAlarm(unsigned char* byte)
{
    while (alarmCount < 4)
    {

        if (alarmEnabled == FALSE)
        {
            writeSet();
            alarm(3); // Set alarm to be triggered in 3s
            alarmEnabled = TRUE;
        }
        else if (alarmEnabled == TRUE)
        {
            int bytesRead = readByteSerialPort(&byte);
            if (bytesRead > 0)
            {
                printf("var = 0x%02X\n", byte);
                printf("EXITED ALARM");
                alarmEnabled = FALSE;
                alarmCount = 0;
                break;
            }
        }
        // in the end, the alarm_handle() disabled alarmEnabled again, meaning 3 seconds have passed
        // try again for 3 more times
    }
    if (alarmCount == 4)
    {
        printf("ALL 4 TRIES FAILED!");
        return 1;
    }
    else return 0;
}

int stateMachine(unsigned char* byte, enum state s, LinkLayerRole role)
{
    unsigned char a,c;
    while (s != STOP)
    {
        switch(s)
        {
            case START:
                //printf("STATE: START\n");
                if (byte == 0x7E)
                {
                    s = FLAG_RCV;
                }
                break;
            case FLAG_RCV:
                //printf("STATE: FLAG_RCV\n");
                if ((byte == 0x01 && role == LlTx) || (byte == 0x03 && role == LlRx))
                {
                    s = A_RCV;
                    a = byte;
                }
                else if (byte != 0x7E)
                {
                    s = START;
                }
                break;
            case A_RCV:
                //printf("STATE: A_RCV\n");
                if ((byte == 0x07 && role == LlTx) || (byte == 0x03 && role == LlRx))
                {
                    s = C_RCV;
                    c = byte;
                }
                else if (byte == 0x7E)
                {
                    s = FLAG_RCV;
                }
                else
                {
                    s = START;
                }
                break;
            case C_RCV:
                //printf("STATE: C_RCV\n");
                if (byte == (a ^ c))
                {
                    s = BCC_OK;
                }
                else if (byte == 0x7E)
                {
                    s = FLAG_RCV;
                }
                else
                {
                    s = START;
                }
                break;
            case BCC_OK:
                //printf("STATE: BCC_OK\n");
                if (byte == 0x7E)
                {
                    s = STOP;
                }
                else
                {
                    s = START;
                }
                break;
            default:
                printf("NOT A DEFINED STATE\n");
                return 1;
        }
        readByteSerialPort(&byte);
        printf("var = 0x%02X\n", byte);
    }
    printf("STATE: STOP\n");
    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    unsigned char byte;
    switch(connectionParameters.role)
    {
        const char *serialPort = connectionParameters.serialPort;
        int baudRate = connectionParameters.baudRate;

        if (openSerialPort(serialPort, baudRate) < 0)
        {
            perror("openSerialPort");
            exit(-1);
        }
        printf("Serial port %s opened\n", serialPort);

        case LlTx:
            struct sigaction act = {0};
            act.sa_handler = &alarmHandler;
            if (sigaction(SIGALRM, &act, NULL) == -1)
            {
                perror("sigaction");
                exit(1);
            }
            printf("Alarm configured\n");

            // write set
            if (txAlarm(&byte))
            {
                return 1;
            };

            // read ua
            stateMachine(&byte,START,LlTx);
            break;
        case LlRx:
            // read set
            readByteSerialPort(&byte);
            printf("var = 0x%02X\n", byte);
            stateMachine(&byte,START,LlRx);

            // write ua
            writeUa();
            break;
        default:
            printf("Invalid role\n");
            return 1;
    }

    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{

    // tamanho do vetor 2*max_payload_size + 6
    // max_payload_size é 1000
    // caso todos os caracteres sejam flag,sera necessario fazer o byte stuffing de todos eles, o que faria com que o tamanho duplicasse
    // a somar a tudo isto, o vetor precisa de mais 6 bytes para as flags

    int countframe = 0;
    unsigned char bcc2 = 0x00;

    //CONFIGURAR AS FLAGS ANTES DO DATA
    unsigned int frame_size = 0;
    unsigned char frame[(2*MAX_PAYLOAD_SIZE) + 6] = {0};
    unsigned char frame[0] = 0x7E; //flag_start
    unsigned char frame[1] = 0x01; //Address transmiter

    unsigned char c_frameN = 0x00;
    if (countframe == 0) {
        unsigned char frame[2] = 0x00;
        countframe = 1;
        unsigned char frame[3] = 0x01 ^ 0x00; //BCC1
    } else {
        unsigned char frame[2] = 0x80;
        countframe = 0;
        unsigned char frame[3] = 0x01 ^ 0x80; //BCC1
    }
    frame_size = 4;

    //PERCORRER OS DADOS E FAZER O BYTE STUFFING
    unsigned char currentByte;
    for (int i=1; i<bufSize;) { //vou ler 2 de em 2 bytes
        currentByte = buf[i];
        bcc2 ^= currentByte;

        byte_stuffing(&currentByte, &frame, &frame_size);
    }


    //ADICIONAR AS FLAGS NO FINAL
    frame[frame_size-1] = bcc2;
    frame[frame_size] = 0x7E;
    frame_size += 2;
    //O VETOR ESTA PRONTO


    //ESCREVER O VETOR PARA A SERIAL PORT
    if (writeBytesSerialPort(frame, frame_size)) return 1;

    return 0;

}

void byte_stuffing (unsigned char *currentbyte, unsigned char *vector, unsigned int *size) {

    if (*currentbyte == 0x7E) {
        (*size) += 2;
        vector[*size-1] = 0x7D; //ESC
        vector[*size] = 0x5E;
    } else if (*currentbyte == 0x7D){
        (*size) += 2;
        vector[*size-1] = 0x7D; //ESC
        vector[*size] = 0x5D;
    } else {
        (*size)++;
        vector[*size-1] = *currentbyte;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////


void byte_destuffing (unsigned char *currentbyte, unsigned char *vector, unsigned int *size) {
    if (vector[*size-1] == 0x7D) {
        if ((*currentbyte == 0x5E)) {
            vector[*size-1] = 0x7E;
        } else if (*currentbyte == 0x5D) {
            vector[*size-1] = 0x7D;
        }

    } else {
        (*size)++;
        vector[*size-1] = *currentbyte;
    }
}


int llread(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        exit(-1);
    }

    printf("Serial port %s closed\n", serialPort);

    return 0;
}

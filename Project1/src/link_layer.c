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

// States of the frame transmission
enum state
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
};

// Type of alarm
enum alarm_state
{
    ALARM_OPEN,
    ALARM_WRITE,
    ALARM_CLOSE
};

// State of the program
enum machine_state
{
    OPEN,
    WRITE,
    READ,
    CLOSE
};

int n_tries = 0, timeout = 0;

void writeSet();
void writeUa();
void alarmHandler();
int checkBCC2();

// Tx state machine
int stateMachine(unsigned char byte, enum state s, enum machine_state ms, LinkLayerRole role, int type);
unsigned char buf[BUF_SIZE] = {0};
int alarmEnabled = FALSE;
int alarmCount = 0;

// LLOPEN AUX
const char *serialPort;
LinkLayerRole x_role;

// LLWRITE AUX FUNCTIONS
int tx_fn, rx_fn;                             
int tx_buf_size = (MAX_PAYLOAD_SIZE * 2) + 6;
unsigned char *tx_frame;
unsigned char rx_answer;

void byte_stuffing(unsigned char *currentbyte, unsigned char *vector, unsigned int *size);
int byte_destuffing(unsigned char *rx_packet, unsigned int *index, unsigned int *size);
int getAType(LinkLayerRole role, int type);

int validResponse(unsigned char byte)
{
    if (stateMachine(byte, START, WRITE, x_role, A_COMMAND))
        return -1;
    else
    {
        if (rx_answer == C_REJ0 || rx_answer == C_REJ1)
            return 1;
        if (rx_answer == C_RR0 || rx_answer == C_RR1)
            return 0;
        else
            return -1;
    }
}

// LLREAD AUX
unsigned char rx_packet[MAX_PAYLOAD_SIZE * 2 + 6];
unsigned int rx_packet_len = 0; 

void writeSet()
{
    buf[0] = FLAG;
    buf[1] = A_TX_C;
    buf[2] = C_SET;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    writeBytesSerialPort(buf, 5);
    sleep(1);
}

void writeUa(int type)
{
    buf[0] = FLAG;
    buf[1] = getAType(x_role, type);
    buf[2] = C_UA;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    writeBytesSerialPort(buf, 5);
    sleep(1);
}

void writeDisc(int type)
{
    buf[0] = FLAG;
    buf[1] = getAType(x_role, type);
    buf[2] = C_DISC;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    writeBytesSerialPort(buf, 5);
    sleep(1);
}

int getAType(LinkLayerRole role, int type)
{
    if (type == A_COMMAND)
    {
        if (role == LlTx)
            return A_TX_C;
        else
            return A_RX_C;
    }
    else
    {
        if (role == LlTx)
            return A_TX_R;
        else
            return A_RX_R;
    }
}

void writeI()
{
    tx_fn = (tx_frame[2] != 0);
    writeBytesSerialPort(tx_frame, tx_buf_size);
    sleep(1);
}

void writeRR()
{
    unsigned char buf[5];
    buf[0] = FLAG;
    buf[1] = A_RX_C;
    buf[2] = (rx_fn == 0 ? C_RR1 : C_RR0);
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    writeBytesSerialPort(buf, 5);
    sleep(1);
}

void writeREJ()
{
    unsigned char buf[5];
    buf[0] = FLAG;
    buf[1] = A_RX_C;
    buf[2] = (rx_fn == 0 ? C_REJ0 : C_REJ1);
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    writeBytesSerialPort(buf, 5);
    sleep(1);
}

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
}

int txAlarm(unsigned char *byte, enum alarm_state as)
{
    while (alarmCount < n_tries)
    {
        if (alarmEnabled == FALSE)
        {
            switch (as)
            {
            case ALARM_OPEN:
                writeSet();
                break;
            case ALARM_WRITE:
                writeI();
                break;
            case ALARM_CLOSE:
                writeDisc(A_COMMAND);
                break;
            default:
                return 1;
            }
            alarm(timeout); 
            alarmEnabled = TRUE;
        }
        else if (alarmEnabled == TRUE)
        {
            int bytesRead = readByteSerialPort(byte);
            switch (as)
            {
            case ALARM_WRITE:
                if (bytesRead > 0)
                {
                    int vr = validResponse(*byte);
                    if (vr == 0)
                    {
                        alarmEnabled = FALSE;
                        alarmCount = 0;
                        return 0;
                    }
                    else if (vr == 1)
                    {
                        alarmEnabled = FALSE;
                    }
                    else
                    {
                        pause();
                    }
                }
                break;
            case ALARM_OPEN:
                if (bytesRead > 0)
                {
                    alarmEnabled = FALSE;
                    alarmCount = 0;
                    return 0;
                }
                break;
            case ALARM_CLOSE:
                if (bytesRead > 0)
                {
                    alarmEnabled = FALSE;
                    alarmCount = 0;
                    return 0;
                }
                break;
            default:
                return 1;
            }
        }
    }
    if (alarmCount == n_tries) return 1;
    return 0;
}

int stateMachine(unsigned char byte, enum state s, enum machine_state ms, LinkLayerRole role, int type)
{
    unsigned char a, c;
    unsigned char p[(MAX_PAYLOAD_SIZE * 2) + 6] = {0};
    int p_index = 0;
    while (s != STOP)
    {
        switch (s)
        {
        case START:
            if (byte == FLAG)
            {
                s = FLAG_RCV;
            }
            break;
        case FLAG_RCV:
            a = byte;
            if ((byte == A_RX_C && role == LlTx && type == A_COMMAND) ||
                (byte == A_TX_C && role == LlRx && type == A_COMMAND) ||
                (byte == A_RX_R && role == LlTx && type == A_REPLY) ||
                (byte == A_TX_R && role == LlRx && type == A_REPLY))
            {
                s = A_RCV;
            }
            else if (byte != FLAG)
            {
                s = START;
            }
            break;
        case A_RCV:
            c = byte;
            switch (ms)
            {
            case OPEN:
                if ((byte == C_UA && role == LlTx) || (byte == C_SET && role == LlRx))
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
            case WRITE:
                if ((byte == C_RR0) || (byte == C_RR1) || (byte == C_REJ0) || (byte == C_REJ1))
                {
                    s = C_RCV;
                    rx_answer = byte;
                }
                else
                {
                    s = START;
                }
                break;
            case READ: 
                if ((byte == C_FRAME0) || (byte == C_FRAME1))
                {
                    s = C_RCV;
                }
                else
                {
                    s = START;
                }
                break;
            case CLOSE:
                if (byte == C_DISC)
                {
                    if (role == LlTx)
                    {
                        s = STOP;
                        return 0;
                    }
                }
                else if (byte == C_UA) {
                    if (role == LlRx) {
                        s = STOP;
                        return 0;
                    }
                }
            default:
                return 1;
            }
            break;
        case C_RCV:
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
            switch (ms)
            {
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
                if (tx_fn == (c != 0))
                {
                    if (byte == FLAG)
                    {
                        s = STOP;
                        p[p_index] = byte;
                        p_index++;
                        memcpy(rx_packet, p, p_index);
                        rx_packet_len = (unsigned int)p_index;
                    }
                    else
                    {
                        p[p_index] = byte;
                        p_index++;
                    }
                }
                else
                {
                    return 1;
                }
                break;
            default:
                return 1;
            }
            break;
        default:
            return 1;
        }
        if (s != STOP)
        {
            readByteSerialPort(&byte);
        }
    }
    return 0;
}

int llopen(LinkLayer connectionParameters)
{
    unsigned char byte;
    serialPort = connectionParameters.serialPort;
    int baudRate = connectionParameters.baudRate;
    x_role = connectionParameters.role;
    n_tries = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;
    if (openSerialPort(serialPort, baudRate) < 0)
    {
        perror("openSerialPort");
        exit(-1);
    }
    switch (connectionParameters.role)
    {
    case LlTx:
        struct sigaction act = {0};
        act.sa_handler = &alarmHandler;
        if (sigaction(SIGALRM, &act, NULL) == -1)
        {
            perror("sigaction");
            exit(1);
        }
        if (txAlarm(&byte, ALARM_OPEN))
        {
            return 1;
        };
        stateMachine(byte, START, OPEN, x_role, A_REPLY);
        break;
    case LlRx:
        readByteSerialPort(&byte);
        stateMachine(byte, START, OPEN, x_role, A_COMMAND);
        writeUa(A_REPLY);
        break;
    default:
        return 1;
    }
    return 0;
}

int llwrite(const unsigned char *buf, int bufSize)
{
    int countframe = 0;
    unsigned char bcc2 = 0x00;
    unsigned int frame_size = 0;
    unsigned char frame[(2 * MAX_PAYLOAD_SIZE) + 6] = {0};
    frame[0] = FLAG;
    frame[1] = A_TX_C;
    if (countframe == 0)
    {
        frame[2] = C_FRAME0;
        countframe = 1;
        frame[3] = A_TX_C ^ C_FRAME0; 
    }
    else
    {
        frame[2] = C_FRAME1;
        countframe = 0;
        frame[3] = A_TX_C ^ C_FRAME1; 
    }
    frame_size = 4;
    unsigned char currentByte;
    for (int i = 0; i < bufSize; i++)
    {
        currentByte = buf[i];
        bcc2 ^= currentByte;
        byte_stuffing(&currentByte, frame, &frame_size);
    }
    byte_stuffing(&bcc2, frame, &frame_size);
    frame[frame_size++] = 0x7E;
    unsigned char byte;
    tx_buf_size = frame_size;
    tx_frame = frame;
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        exit(1);
    }
    if (txAlarm(&byte, ALARM_WRITE)) return 1;
    return 0;
}

void byte_stuffing(unsigned char *currentbyte, unsigned char *vector, unsigned int *size)
{
    if (*currentbyte == FLAG)
    {
        vector[*size] = 0x7D;
        vector[*size + 1] = 0x5E;
        (*size) += 2;
    }
    else if (*currentbyte == 0x7D)
    {
        vector[*size] = 0x7D;
        vector[*size + 1] = 0x5D;
        (*size) += 2;
    }
    else
    {
        vector[*size] = *currentbyte;
        (*size)++;
    }
}

int llread(unsigned char *packet)
{
    unsigned char byte;
    readByteSerialPort(&byte);
    if (stateMachine(byte, START, READ, x_role, 0))
    {
        writeREJ();
        return -1;
    }
    int flag = 0;
    unsigned int index = 0, size = 0;
    while (flag != 1)
    {
        if (size >= (MAX_PAYLOAD_SIZE * 2 + 6))
            return 1;
        flag = byte_destuffing(rx_packet, &index, &size);
    }
    if (checkBCC2(rx_packet, size))
    {
        writeREJ();
        return -1;
    }
    else
    {
        rx_fn = (rx_fn == 0) ? 1 : 0;
        writeRR();
        int copy_size = (size - 1 > MAX_PAYLOAD_SIZE) ? MAX_PAYLOAD_SIZE : size - 1;
        memcpy(packet, rx_packet, copy_size);
        return copy_size;
    }
}

int byte_destuffing(unsigned char *rx_packet, unsigned int *index, unsigned int *size)
{
    if (rx_packet[*index] == 0x7D)
    {
        if (*index + 1 >= (MAX_PAYLOAD_SIZE * 2 + 6))
            return -1;
        if ((rx_packet[*index + 1] == 0x5E))
        {
            rx_packet[*size] = 0x7E;
        }
        else if (rx_packet[*index + 1] == 0x5D)
        {
            rx_packet[*size] = 0x7D;
        }
        (*index) += 2;
        (*size)++;
    }
    else
    {
        if (rx_packet[*index] == FLAG) return 1;
        rx_packet[*size] = rx_packet[*index];
        (*size)++;
        (*index)++;
    }
    return 0;
}

int checkBCC2(unsigned char *rx_packet, int size)
{
    unsigned char bcc2_acc = 0;
    for (int i = 0; i < size; i++)
    {
        unsigned char byte = rx_packet[i];
        bcc2_acc ^= byte;
    }
    if (bcc2_acc == 0) return 0;
    return 1;
}

int llclose()
{
    unsigned char byte;
    switch (x_role)
    {
    case LlTx:
        struct sigaction act = {0};
        act.sa_handler = &alarmHandler;
        if (sigaction(SIGALRM, &act, NULL) == -1)
        {
            perror("sigaction");
            exit(1);
        }
        if (txAlarm(&byte, ALARM_CLOSE))
        {
            return 1;
        };
        readByteSerialPort(&byte);
        stateMachine(byte, A_RCV, CLOSE, x_role, A_REPLY);
        writeUa(A_REPLY);
        break;

    case LlRx:
        readByteSerialPort(&byte);
        stateMachine(byte, A_RCV, CLOSE, x_role, A_COMMAND);
        writeDisc(A_COMMAND);
        readByteSerialPort(&byte);
        stateMachine(byte, A_RCV, CLOSE, x_role, A_REPLY);
        break;
    default:
        return 1;
    }
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        exit(-1);
    }
    return 0;
}

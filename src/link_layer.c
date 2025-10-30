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
int tx_fn, rx_fn;                             // sender's frame number (I(0) would be 0, varies between 0,1)
int tx_buf_size = (MAX_PAYLOAD_SIZE * 2) + 6; // currently: max buffer size that will be sent (buf's data with stuffing + 6 flags), can be ajusted to only the necessary
unsigned char *tx_frame;
unsigned char rx_answer;

void byte_stuffing(unsigned char *currentbyte, unsigned char *vector, unsigned int *size);
int byte_destuffing(unsigned char *rx_packet, unsigned int *index, unsigned int *size);
int getAType(LinkLayerRole role, int type);

int validResponse(unsigned char byte)
{
    // Expect COMMAND (A_RX_C) from receiver for RR/REJ during WRITE
    if (stateMachine(byte, START, WRITE, x_role, A_COMMAND))
        return -1;
    else
    {
        // stateMachine got an valid answer
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
unsigned int rx_packet_len = 0; // stuffed payload length captured by state machine (includes BCC2, excludes FLAGs

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
    //printf("[TX] Timeout -> retransmitting last frame, alarmCount: %d\n", alarmCount);
}

int txAlarm(unsigned char *byte, enum alarm_state as)
{
    while (alarmCount < 4)
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
            alarm(3); // Set alarm to be triggered in 3s
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
                        pause(); // waits for the handler
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
        // in the end, the alarm_handle() disabled alarmEnabled again, meaning 3 seconds have passed
        // try again for 3 more times
    }
    if (alarmCount == 4) return 1;
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
            // As a Transmitter I want to read Recevier's Commands or Responses
            // As a Receiver I want to read Transmitters Commands or Responses
            //se for Rx quero ler COMANDOS (0x03) ou RESPOSTAS (0x01) do TRANSMISSOR
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
            case WRITE: // i.e: write treats packets sent by LLREAD()
                if ((byte == C_RR0) || (byte == C_RR1) || (byte == C_REJ0) || (byte == C_REJ1))
                {
                    s = C_RCV;
                    rx_answer = byte;
                }
                else
                {
                    s = START;  //There was an error, must go back to the initial state
                }
                break;
            case READ: // i.e: read treats packets sent by LLWRITE()
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
                // ou seja: DISC aceita, mas só aceitar UA se for o Rx a ler (ou seja, o Tx mandou)
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
                // CHECK NS (TX_FN): If NS is the expected (new frame, no duplicate) then GO, else RR(NS)
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
                    return 1;   // ignore or send RR(Ns)
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

        // Write Set
        if (txAlarm(&byte, ALARM_OPEN))
        {
            return 1;
        };

        // Read UA
        stateMachine(byte, START, OPEN, x_role, A_REPLY);
        break;
    case LlRx:
        // Read Set
        readByteSerialPort(&byte);
        stateMachine(byte, START, OPEN, x_role, A_COMMAND);

        // Write Ua
        writeUa(A_REPLY);
        break;
    default:
        return 1;
    }

    printf("[SM] LLOPEN() WAS SUCCESSFUL!\n");
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

    // CONFIGURAR AS FLAGS ANTES DO DATA
    unsigned int frame_size = 0;
    unsigned char frame[(2 * MAX_PAYLOAD_SIZE) + 6] = {0};
    frame[0] = FLAG;
    frame[1] = A_TX_C;

    if (countframe == 0)
    {
        frame[2] = C_FRAME0;
        countframe = 1;
        frame[3] = A_TX_C ^ C_FRAME0; // BCC1
    }
    else
    {
        frame[2] = C_FRAME1;
        countframe = 0;
        frame[3] = A_TX_C ^ C_FRAME1; // BCC1
    }
    frame_size = 4;

    // PERCORRER OS DADOS E FAZER O BYTE STUFFING
    unsigned char currentByte;

    // Percorrer os dados e fazer o byte stuffing
    for (int i = 0; i < bufSize; i++)
    {
        currentByte = buf[i];
        bcc2 ^= currentByte; // calcular BCC
        byte_stuffing(&currentByte, frame, &frame_size);
    }

    // adiciona o BCC2 e faz o stuffing do BCC2
    byte_stuffing(&bcc2, frame, &frame_size);
    frame[frame_size++] = 0x7E;
    // O VETOR ESTA PRONTO

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

    // write I and expect ACK
    if (txAlarm(&byte, ALARM_WRITE))
        return 1;

    return 0;
}

// Used to escape the flags inside a frame
// Receives a byte, a vector and it's current size
// Appends the stuffed byte to the vector and updated its size
void byte_stuffing(unsigned char *currentbyte, unsigned char *vector, unsigned int *size)
{
    if (*currentbyte == FLAG)
    {
        vector[*size] = 0x7D; // ESC
        vector[*size + 1] = 0x5E;
        (*size) += 2;
    }
    else if (*currentbyte == 0x7D)
    {
        vector[*size] = 0x7D; // ESC
        vector[*size + 1] = 0x5D;
        (*size) += 2;
    }
    else
    {
        vector[*size] = *currentbyte;
        (*size)++;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
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
    unsigned int index = 0, size = 0; // destuffed payload size will be stored in 'size'

    while (flag != 1)
    {
        if (size >= (MAX_PAYLOAD_SIZE * 2 + 6))
            return 1;
        flag = byte_destuffing(rx_packet, &index, &size);
    }

    if (checkBCC2(rx_packet, size)) // bcc2 not correct
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

// Used to destuff the frame on the receiver side
// Receives a packet, it's size, and the current size of the destuffed packet
// Destuffs the byte at the given index of the packet given. Retuns 0 if there wasn't any errors, returns -1 otherwise
int byte_destuffing(unsigned char *rx_packet, unsigned int *index, unsigned int *size)
{
    if (rx_packet[*index] == 0x7D)
    { // se detetar 0x7D inspeciona o proximo byte
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

    // i.e: tem de ser 0 porque o último byte foi o BCC2, pelo que se estiver certo (sum DATA ^ BCC2 = 0)
    // um pacote XOR com ele próprio dá 0...
    if (bcc2_acc == 0) return 0;
    return 1;
}


////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
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

        // Write DISC (0x03)
        if (txAlarm(&byte, ALARM_CLOSE))
        {
            return 1;
        };

        // Read DISC
        readByteSerialPort(&byte);
        stateMachine(byte, A_RCV, CLOSE, x_role, A_REPLY);

        // Write UA
        writeUa(A_REPLY);

        break;

    case LlRx:
        // Confirm Read DISC
        readByteSerialPort(&byte);
        stateMachine(byte, A_RCV, CLOSE, x_role, A_COMMAND);

        // Write DISC (0x01)
        writeDisc(A_COMMAND);

        // Confirm Read UA
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

    printf("[SM] LLCLOSE() WAS SUCCESSFUL!\n");

    return 0;
}

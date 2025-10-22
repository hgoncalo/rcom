// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

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
enum alarm_state {OPEN,WRITE};
enum machine_state {OPEN, WRITE, READ, CLOSE};

void writeSet();
void writeUa();
void alarmHandler();
// Tx state machine
int stateMachine();

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

// LLREAD AUX
unsigned char *rx_packet;

/*
// data will already be destuffed
if (byte == FLAG)
{
    // se recebeu flag, então o acumulador chegou ao fim (o ultimo valor acumulado foi BCC2)
    // mas ele tb já acumulou o BCC2 estipulado/recebido
    // ou seja: DATA XOR BCC2, teoricamente, devia dar 0 (ou seja 1 XOR 1, daria 0), porque a XOR ACC DATA == BCC2
    // pelo que, se não forem iguais, o BCC2 ou a DATA estão errados (voltar ao inicio e não aceitar o frame)
    if (bcc2_acc == 0)
    {
        s = STOP; // BCC2_ACC = BCC2 (then OK)
        // RR(NS+1)
        // RETURN DATA PACKET
    }
    else
    {
        s = START;
        // SEND REJ
    }
}
else
{
    bcc2_acc ^= *byte;
}
*/

// otimizar este código numa func. única?
void writeSet()
{
    buf[0] = FLAG;
    buf[1] = A_TX_C;
    buf[2] = C_SET;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    int bytes = writeBytesSerialPort(buf, BUF_SIZE);
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
    int bytes = writeBytesSerialPort(buf, BUF_SIZE);
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
    int bytes = writeBytesSerialPort(buf, BUF_SIZE);
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
    tx_fn = (tx_frame[2] != 0);
    writeBytesSerialPort(tx_frame,tx_buf_size);
    sleep(1);
}

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d received\n", alarmCount);
}

int txAlarm(unsigned char* byte, enum alarm_state as)
{
    while (alarmCount < 4)
    {
        if (alarmEnabled == FALSE)
        {
            switch(as)
            {
                case OPEN:
                    writeSet();
                    break;
                case WRITE:
                    writeI();
                    break;
                default:
                    printf("NOT A DEFINED STATE\n");
                    return 1;
            }
            
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

int stateMachine(unsigned char byte, enum state s, enum machine_state ms, LinkLayerRole role, int type)
{
    unsigned char a,c;
    unsigned char p[(MAX_PAYLOAD_SIZE * 2) + 6] = {0};
    int p_index = 0;

    while (s != STOP)
    {
        switch(s)
        {
            case START:
                if (byte == FLAG)
                {
                    s = FLAG_RCV;
                }
                break;
            case FLAG_RCV:
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
                        printf("NOT A DEFINED STATE\n");
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

                                // Copiar p/ buffer global, pq P é local e será destruido
                                memcpy(rx_packet, p, p_index);
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
                            s = START;
                            // voltar ao inicio ignora a trama (o emissor toma iniciativa de mandar de novo)
                        }
                        break;
                    default:
                        printf("NOT A DEFINED STATE\n");
                        return 1;
                }
                break;
            default:
                printf("NOT A DEFINED STATE\n");
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

    if (openSerialPort(serialPort, baudRate) < 0)
    {
        perror("openSerialPort");
        exit(-1);
    }
    printf("Serial port %s opened\n", serialPort);

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
            printf("Alarm configured\n");

            // write set
            if (txAlarm(&byte))
            {
                return 1;
            };

            // read ua
            stateMachine(byte,START,OPEN,x_role,A_COMMAND);
            break;
        case LlRx:
            // read set
            readByteSerialPort(&byte);
            //printf("var = 0x%02X\n", byte);
            stateMachine(&byte,START,OPEN,x_role,A_COMMAND);

            // write ua
            writeUa(A_COMMAND);
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
    unsigned char frame[0] = FLAG;
    unsigned char frame[1] = A_TX_C;

    if (countframe == 0) {
        unsigned char frame[2] = C_FRAME0;
        countframe = 1;
        unsigned char frame[3] = A_TX_C ^ C_FRAME0; //BCC1
    } else {
        unsigned char frame[2] = C_FRAME1;
        countframe = 0;
        unsigned char frame[3] = A_TX_C ^ C_FRAME1; //BCC1
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
    frame[frame_size] = FLAG;
    frame_size += 2;
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
    printf("Alarm configured\n");
    
    // write I and expect ACK
    if (txAlarm(&byte,WRITE))
    {
        return 1;
    }
    // logica de verificação do byte (verificar FLAG,A,C,BCC)
    // fazer máquina de estados again, fazer verificação com alarm_state, role
    // ou seja, em vez das multiplas opções, maybe um switch pelo alarm_state/phase (OPEN,WRITE,READ) não era mal pensado, para distinguir o C

    // Falta SendREJ(n) ou SendRR(n)

    // retornar se tudo certo
    return 0;
}

void byte_stuffing (unsigned char *currentbyte, unsigned char *vector, unsigned int *size) {
    if (*currentbyte == FLAG) {
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
        rx_packet[*size] = rx_packet[*index];
        (*size)++;
        (*index)++;
        if (rx_packet[*size - 1] == FLAG) {
            return 1;
        }
    }

    return 0;
}


int llread(unsigned char *packet)
{
    int flag = 0;
    unsigned int index = 1, size = 1;
     
    while (flag != 1) {
        if (size >= (MAX_PAYLOAD_SIZE * 2 + 6)) return 1;
        flag = byte_destuffing(rx_packet, &index, &size);
    }

    return 0;
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
            printf("Invalid role\n");
            return 1;
    }

    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        exit(-1);
    }

    printf("Serial port %s closed\n", serialPort);

    return 0;
}

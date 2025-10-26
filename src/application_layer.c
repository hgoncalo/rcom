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


enum state {OPENFILE, START_PACKET, DATA_PACKET, END_PACKET, END};

void stateMachine(enum state s) {
    switch (s) {
        case OPENFILE:
            if (parse_cmdLine()) return;
            if (openFile()) return;
            if (llopen(connectionParameters)) return;
            s = START_PACKET;
            break;
        case START_PACKET:
            if (buildCtrlPck()) {
                llclose();
                s = END;
            }
            if (llwrite()) {
                llclose();
                s = END;
            }
            s = DATA_PACKET;
            break;
        case DATA_PACKET:
            if (readFragFile()) {
                if (buildDataPck()) {
                    llclose();
                    s = END;
                }
                if (llwrite()) {
                    llclose();
                    s = END;
                }
                s = END_PACKET;
            }
            break;
        case END_PACKET:
            buildCtrlPck();
            llwrite();
            llclose();
            s = END;
            break;
        default:
            break;
    }
}

int parse_cmdLine();

int openFile() {
    fd = open(pathname, flags);
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
    ctrl_pck[2] = (unsigned int) file_size;
    ctrl_pck[2] = control;
    //parameter file name (t,l,v)
}




int buildDataPck();
int readFragFile();

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

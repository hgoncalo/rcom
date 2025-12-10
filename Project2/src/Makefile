CC = gcc
CFLAGS = -Wall

downloadFile: downloadFile.o getip.o clientTCP.o
	$(CC) $(CFLAGS) -o downloadFile downloadFile.o getip.o clientTCP.o

downloadFile.o: downloadFile.c getip.h clientTCP.h
	$(CC) $(CFLAGS) -c downloadFile.c

getip.o: getip.c getip.h
	$(CC) $(CFLAGS) -c getip.c

clientTCP.o: clientTCP.c clientTCP.h
	$(CC) $(CFLAGS) -c clientTCP.c

clean:
	rm -f *.o downloadFile
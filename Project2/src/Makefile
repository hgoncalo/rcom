CC = gcc
CFLAGS = -Wall

download: download.o getip.o clientTCP.o
	$(CC) $(CFLAGS) -o download download.o getip.o clientTCP.o

download.o: download.c getip.h clientTCP.h
	$(CC) $(CFLAGS) -c download.c

getip.o: getip.c getip.h
	$(CC) $(CFLAGS) -c getip.c

clientTCP.o: clientTCP.c clientTCP.h
	$(CC) $(CFLAGS) -c clientTCP.c

clean:
	rm -f *.o download
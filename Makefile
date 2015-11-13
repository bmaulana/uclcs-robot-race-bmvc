CC = gcc
CFlags = -g -Wall -lm
all: final
final: final.o picomms.o
	$(CC) $(CFlags) -o final picomms.o final.o
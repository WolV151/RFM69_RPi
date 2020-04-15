prog: main.o RFM69.o
	gcc -o prog main.o RFM69.o

main.o: main.c
	gcc -c main.c

RFM69.o: RFM69.c
	gcc -c RFM69.c
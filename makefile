prog: example.o RFM69.o
	gcc -o prog example.o RFM69.o -lwiringPi

example.o: example.c
	gcc -c example.c

RFM69.o: RFM69.c
	gcc -c RFM69.c
CC=arm-linux-gnueabihf-gcc
ZLOG_PATH:=/home/fashion/armzlog

exe=101-code

all : $(exe) clean
$(exe) : siec101_2002.o siec101r_2002.o siec101t_2002.o rs485-test.o 

	$(CC) -O2 -g -o $@ $^ -L$(ZLOG_PATH)/lib -lzlog -lpthread

.c.o:
	$(CC) -O2 -g -Wall -D_GNU_SOURCE -o $@ -c $< -I$(ZLOG_PATH)/include


clean:
	rm -f *.o 

.PHONT:clean all

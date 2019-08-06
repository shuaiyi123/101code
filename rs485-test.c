/* Copyright 2018 Tronlong Elec. Tech. Co. Ltd. All Rights Reserved. */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <libgen.h>
#include <signal.h>
#include <getopt.h>
#include <sys/mman.h>
#include <sys/types.h>

#include "zlog.h"
#include "siec101_2002.h"

#define INADEQUATE_CONDITIONS 3

enum Mode { READ, WRITE };


#define RS485_DEPIN  29  // bit29

#define GPIO_BASE_ADDR         0x41200000
#define GPIO_DATA_OFFSET       0x0
#define GPIO_OE_OFFSET         0x4

#define MEM_NODE   "/dev/mem"
#define MAP_SIZE   4096UL
#define MAP_MASK   (MAP_SIZE - 1)

#define RECVBUFF_LENTH 128
#define TRANSBUFF_LENTH 512

/* Global Variable */
extern RxdFrame_buf m_RxdFrame ; //接收报文缓冲区
TxdFrame_buf m_Txd;    //发送报文缓存区
void *gpio_map_base, *gpio_virt_addr;
int mem_fd;

int fd1;

enum gpio_mode {
	GPIO_OUT_MODE = 0,
	GPIO_IN_MODE
};

enum gpio_state {
	GPIO_LOW = 0,
	GPIO_HIGH
};

/* Exit flag */
volatile bool g_quit = false;

int baudrate = B115200;

zlog_category_t *zc;

/* Function List */
int mmap_gpio(void);
int SetGpioDirect(char gpio, bool direct);
int GpioOutput(char gpio, bool state);

/* Short option names */
static const char g_shortopts [] = ":d:s:b:rwvh";

/* Option names */
static const struct option g_longopts [] = {
    { "device",      required_argument,      NULL,        'd' },
    { "read",        no_argument,            NULL,        'r' },
    { "write",       no_argument,            NULL,        'w' },
    { "size",        required_argument,      NULL,        's' },
    { "baud",        required_argument,      NULL,        'b' },
    { "version",     no_argument,            NULL,        'v' },
    { "help",        no_argument,            NULL,        'h' },
    { 0, 0, 0, 0 }
};

static void usage(FILE *fp, int argc, char **argv) {
    fprintf(fp,
            "Usage: %s [options]\n\n"
            "Options:\n"
            " -d | --device        Device such as '/dev/ttyS0'\n"
            " -r | --read          Read\n"
            " -w | --write         Write\n"
            " -s | --size          Read size\n"
            " -b | --baud          Baudrate\n"
            " -v | --version       Display version information\n"
            " -h | --help          Show help content\n"
            " e.g. ./tl-uart-rw -d /dev/ttyS1 -r -s 256 -b 115200\n"
            "      ./tl-uart-rw -d /dev/ttyS1 -w -s 1024 -b 115200\n\n"
            "", basename(argv[0]));
}

static void opt_parsing_err_handle(int argc, char **argv, int flag) {
    /* Exit if no input parameters are entered  */
    int state = 0;
    if (argc < 2) {
        printf("No input parameters are entered, please check the input.\n");
        state = -1;
    } else {
        /* Feedback Error parameter information then exit */
        if (optind < argc || flag) {
            printf("Error:  Parameter parsing failed\n");
            if (flag)
                printf("\tunrecognized option '%s'\n", argv[optind-1]);

            while (optind < argc) {
                printf("\tunrecognized option '%s'\n", argv[optind++]);
            }

            state = -1;
        }
    }

    if (state == -1) {
        printf("Tips: '-h' or '--help' to get help\n\n");
        exit(2);
    }
}

void sig_handle(int arg) {
    g_quit = true;
}

int mmap_gpio(void)
{
    mem_fd = open(MEM_NODE, O_RDWR | O_SYNC); 
    if (mem_fd < 0) {
        printf("open %s is error\n", MEM_NODE);
        return -1;
    }

    gpio_map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, GPIO_BASE_ADDR & ~MAP_MASK);

    if(gpio_map_base == (void *) -1)
        return -1;

    gpio_virt_addr = gpio_map_base + (GPIO_BASE_ADDR & MAP_MASK);

	return 0;
}

int SetGpioDirect(char gpio, bool direct)
{
	unsigned int addr;
	unsigned int val;

	addr = (unsigned int)gpio_virt_addr;
	val = *(unsigned int *)(addr + GPIO_OE_OFFSET);

	if (direct == GPIO_OUT_MODE)
	{
		val &= ~(1 << gpio);
	}
	else if (direct == GPIO_IN_MODE)
	{
		val |= (1 << gpio);
	}

	*(unsigned int *)(addr + GPIO_OE_OFFSET) = val;

	return 0;
}

int GpioOutput(char gpio, bool state)
{
	unsigned int addr;
	unsigned int val;

	addr = (unsigned int)gpio_virt_addr;
	val = *(unsigned int *)(addr + GPIO_DATA_OFFSET);

	if (state == GPIO_LOW) {
		val &= ~(1 << gpio);	
	}
	else if (state == GPIO_HIGH) {
		val |= (1 << gpio);
	}

	*(unsigned int *)(addr + GPIO_DATA_OFFSET) = val;

    return 0;
}

void enable_rx()
{
	GpioOutput(RS485_DEPIN, GPIO_LOW);
}

void enable_tx()
{
	GpioOutput(RS485_DEPIN, GPIO_HIGH);
}

int find_baudrate(int rate)
{
	int baudr;

    switch(rate)
    {
		case    9600 : baudr = B9600;
                   break;
		case   19200 : baudr = B19200;
                   break;
		case   38400 : baudr = B38400;
                   break;
		case   57600 : baudr = B57600;
                   break;
		case  115200 : baudr = B115200;
                   break;
		case  230400 : baudr = B230400;
                   break;
		case  460800 : baudr = B460800;
                   break;
		case  500000 : baudr = B500000;
                   break;
		case  576000 : baudr = B576000;
                   break;
		case  921600 : baudr = B921600;
                   break;
		case 1000000 : baudr = B1000000;
                   break;
		case 1152000 : baudr = B1152000;
                   break;
		case 1500000 : baudr = B1500000;
                   break;
		case 2000000 : baudr = B2000000;
                   break;
		case 2500000 : baudr = B2500000;
                   break;
		case 3000000 : baudr = B3000000;
                   break;
		case 3500000 : baudr = B3500000;
                   break;
		case 4000000 : baudr = B4000000;
                   break;
		default      : printf("invalid baudrate, set baudrate to 115200\n");
					baudr = B115200;
                   break;
    }

	return baudr;
}

int init_serial(int *fd, const char *dev, int rate) {
    struct termios opt;

    /* open serial device */
    if ((*fd = open(dev, O_RDWR)) < 0) {
        perror("open()");
        return -1;
    }

    /* define termois */
    if (tcgetattr(*fd, &opt) < 0) {
        perror("tcgetattr()");
        return -1;
    }

    opt.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_oflag  &= ~OPOST;

    /* Character length, make sure to screen out this bit before setting the data bit */
    opt.c_cflag &= ~CSIZE;

    /* No hardware flow control */
    opt.c_cflag &= ~CRTSCTS;

    /* 8-bit data length */
    opt.c_cflag |= CS8;

    /* 1-bit stop bit */
    opt.c_cflag &= ~CSTOPB;

    /* No parity bit */
    opt.c_iflag |= IGNPAR;

    /* Output mode */
    opt.c_oflag = 0;
    
    /* No active terminal mode */
    opt.c_lflag = 0;

	baudrate = find_baudrate(rate);

    /* Input baud rate */
    if (cfsetispeed(&opt, baudrate) < 0)
        return -1;

    /* Output baud rate */
    if (cfsetospeed(&opt, baudrate) < 0)
        return -1;

    /* Overflow data can be received, but not read */
    if (tcflush(*fd, TCIFLUSH) < 0)
        return -1;

    if (tcsetattr(*fd, TCSANOW, &opt) < 0)
        return -1;

    return 0;
}

int serial_write(int *fd, const char *data, size_t size) {
    int ret = write(*fd, data, size);
    if ( ret < 0 ) {
        perror("write");
        tcflush(*fd, TCOFLUSH);
    }

    return ret;
}

int serial_read(int *fd, char *data, size_t size) {
    size_t read_left=size;
    size_t read_size = 0;
    char *read_ptr = data;
    struct timeval timeout = {1, 0};

    memset(data, 0, size);

    fd_set rfds;
    while (!g_quit) {
        FD_ZERO(&rfds);
        FD_SET(*fd, &rfds);
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
	
    if(read_size>0){  //接收到数据
    	memcpy(m_RxdFrame.buf,read_ptr,read_size);
    	RxdMonitor(); //报文解析处理
    	// enable_tx();
        // serial_write(fd,m_RxdFrame.buf,read_size);
        // usleep(10000);
        // enable_rx();
        read_size=0;
    }
        switch (select(*fd+1, &rfds, NULL, NULL, &timeout)) {
        case -1:
            //perror("select()");
            break;
        case 0:
            //perror("timeout and retry");
            break;
        default:
	    usleep(10000);
            if (FD_ISSET(*fd,&rfds)) {  //测试fd是否可读。即是否有串口数据
                read_size = read(*fd, read_ptr, read_left);
                if (read_size == 0)
                    break;
                printf("read_size: %d\n", read_size);
            }
        }
    }

    return strlen(data);
}

/**
 * @description:发送报文 
 * @param {type} 
 * @return: 1，发送成功；0，发送失败
 */
uint8 StartTrans(uint8 *buff,uint8 buff_Length)
{
    size_t write_size=0;
    enable_tx();
    
  //  while(buff_Length>0)
  //  {
        write_size=write(fd1,buff,buff_Length);
   //   buff_Length-=write_size;
        usleep(50000);
 //   }
    

    if(write_size==0){
        enable_rx();
        return 0;
    }
    enable_rx();
    return 1;
}

int main(int argc, char *argv[]) {
    int c = 0;
    int flag = 0;
    int fd = -1;
    int mode = -1;
    int ret = -1;
    size_t size = 0;
    char *buf = NULL;
    char *dev = NULL;
    int rate = 115200;
    int rc;
    RTU_Init();
    rc = zlog_init("/etc/zlog.conf");
    if (rc)
    {
        printf("zlog init failed\n");
        return -1;
    }

    zc = zlog_get_category("my_cat");
    if (!zc)
    {
        printf("zlog get cat fail\n");
        zlog_fini();
        return -2;
    }

    /* Parsing input parameters */
    while ((c = getopt_long(argc, argv, g_shortopts, g_longopts, NULL)) != -1) {
        switch (c) {
        case 'd':
            dev = optarg;
            break;

        case 'r':
            mode = READ;
            break;

        case 'w':
            mode = WRITE;
            break;

        case 's':
            size = atoi(optarg);
            break;

        case 'b':
            rate = atoi(optarg);
            printf("rate: %d\n", rate);
            break;
        case 'v':
            /* Display the version */
            printf("version : 1.0\n");
            exit(0);

        case 'h':
            usage(stdout, argc, argv);
            exit(0);
                
        default :
            flag = 1;
            break;
        }
    }

    opt_parsing_err_handle(argc, argv, flag);

    /* Ctrl+c handler */
    signal(SIGINT, sig_handle);

    ret = init_serial(&fd, dev, rate);
    if (ret < 0) {
        goto release;
    }

	ret = mmap_gpio();
	if (ret) {
		printf("mmap gpio failed!\n");
		return ret;
	}

	SetGpioDirect(RS485_DEPIN, GPIO_OUT_MODE);

    switch (mode) {
    case READ:
        //printf("Mode : read\n");
		enable_rx();
        if (size <= 0) {
            zlog_error(zc, "Error : Incorrect size settings");
            exit(INADEQUATE_CONDITIONS);
        }

        buf = (char*)malloc(size);  //分配接收数据
        m_RxdFrame.buf=(char*)malloc(RECVBUFF_LENTH);  //分配报文接收缓存
        m_Txd.buf=(char*)malloc(TRANSBUFF_LENTH);      //分配报文发送缓存

	    fd1=fd;
        ret = serial_read(&fd1, buf, size);
        //zlog_info(zc, "recv: %s size: %d", buf, ret);
        free(buf);
        free(m_RxdFrame.buf);
        free(m_Txd.buf);

        break;

    case WRITE:
        //printf("Mode : write\n");
		enable_tx();
        if (size <= 0) {
            zlog_error(zc, "Error : Incorrect size settings");
            exit(INADEQUATE_CONDITIONS);
        }

        int i = 0;
        char context;
        size_t write_size = 0;
        while (!g_quit) {
            if (i > 7)
                i = 0;
            context = (char)('0' + i);
                
            write_size += serial_write(&fd, &context, sizeof(context));
            i ++;

            if (size == write_size)
                break;
        }

        zlog_info(zc, "send size: %d", write_size);
        break;

    default:
        break;
    }

release:
    close(fd);

    if(ret < 0)
        return INADEQUATE_CONDITIONS;

	close(mem_fd);
	munmap(gpio_map_base, MAP_SIZE);

    return 0;
}

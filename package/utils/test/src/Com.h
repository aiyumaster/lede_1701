

#ifndef _COM_H_
#define _COM_H_

#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h>  
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> 
#include <termios.h>
#include <errno.h>   
#include <limits.h> 
#include <asm/ioctls.h>
#include <time.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/epoll.h> 
#include <string.h>




#define EV_MAX_NUM	6



struct Com_command
{
	unsigned char header[2];
	unsigned short len;
	unsigned char direction;
	unsigned char command;
	unsigned char data[100];
	unsigned char checksum;
	unsigned char flag;
	unsigned short count;
};




extern	int OpenSerial(char *cSerialName, int nSpeed, int nBits, char nEvent, int nStop);
extern	int EpollSerialRead(int epfd, int fd, char * ReadBuff, const int ReadLen);
extern	void EpollSerialParse(int epfd,  struct epoll_event *event, struct Com_command *command);





#endif



#include <stdio.h>
#include <string.h>

#include "Com.h"










int main(void)
{

	int fd, epfd;//�ļ�������

	struct epoll_event event, *events;

	struct Com_command com_data;



	fd = OpenSerial("/dev/ttyS0", 19200, 8, "N", 1);
	
	if(fd < 0)
	{
		printf("open com fail!\n");
		
		return 0;
	}



	epfd = epoll_create(EV_MAX_NUM);  //���ڳ�ʼ��
	event.events = EPOLLET | EPOLLIN;
	event.data.fd = fd;
	if (epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &event) != 0)  //�����¼���ӵ�epoll���¼�������
	{
		printf("Set Com epoll error!\n");
		return -1;
	}
	printf("set epoll ok!\n");

	EpollSerialParse(epfd, &event, &com_data);
	
	
	
	close(epfd);
	close(fd);

	return 0;
}



#include "Com.h"





/********************************************************************************************
Epoll Serial ��ȡ   DTSЭ�����







********************************************************************************************/



int OpenSerial(char *cSerialName, int nSpeed, int nBits, char nEvent, int nStop)
{
    int fd;

    struct termios options;   // �������ýṹ��

    fd = open(cSerialName, O_RDWR | O_NOCTTY |O_NONBLOCK);
	//fd = open(cSerialName, O_RDWR | O_NOCTTY | O_NDELAY);//���� |O_RSYNC
	
    if(fd < 0) 
	{
        perror(cSerialName);
        return -1;
    }


	tcgetattr(fd,&options); //��ȡ��ǰ����
	bzero(&options,sizeof(options));

	

	switch( nSpeed )
	{
		case 50:
			cfsetispeed(&options, B50);
			cfsetospeed(&options, B50);
			break;
			
		case 75:
			cfsetispeed(&options, B75);
			cfsetospeed(&options, B75);
			break;
			
		case 110:
			cfsetispeed(&options, B110);
			cfsetospeed(&options, B110);
			break;
			
		case 134:
			cfsetispeed(&options, B134);
			cfsetospeed(&options, B134);
			break;
			
		case 150:
			cfsetispeed(&options, B150);
			cfsetospeed(&options, B150);
			break;
			
		case 200:
			cfsetispeed(&options, B200);
			cfsetospeed(&options, B200);
			break;
			
		case 300:
			cfsetispeed(&options, B300);
			cfsetospeed(&options, B300);
			break;
			
		case 600:
			cfsetispeed(&options, B600);
			cfsetospeed(&options, B600);
			break;
			
		case 1200:
			cfsetispeed(&options, B1200);
			cfsetospeed(&options, B1200);
			break;
			
		case 1800:
			cfsetispeed(&options, B1800);
			cfsetospeed(&options, B1800);
			break;
			
		case 2400:
			cfsetispeed(&options, B2400);
			cfsetospeed(&options, B2400);
			break;
			
		case 4800:
			cfsetispeed(&options, B4800);
			cfsetospeed(&options, B4800);
			break;
			
		case 9600:
			cfsetispeed(&options, B9600);
			cfsetospeed(&options, B9600);
			break;
			
		case 14400:
			cfsetispeed(&options, B9600);
			cfsetospeed(&options, B9600);
			break;
			
		case 19200:
			cfsetispeed(&options, B19200);
			cfsetospeed(&options, B19200);
			break;
			
		case 38400:
			cfsetispeed(&options, B38400);
			cfsetospeed(&options, B38400);
			break;
			
		case 57600:
			cfsetispeed(&options, B57600);
			cfsetospeed(&options, B57600);
			break;
			
		case 115200:
			cfsetispeed(&options, B115200);
			cfsetospeed(&options, B115200);
			break;
			
		case 230400:
			cfsetispeed(&options, B230400);
			cfsetospeed(&options, B230400);
			break;
			
		case 460800:
			cfsetispeed(&options, B460800);
			cfsetospeed(&options, B460800);
			break;
			
		case 500000:
			cfsetispeed(&options, B500000);
			cfsetospeed(&options, B500000);
			break;
			
		case 576000:
			cfsetispeed(&options, B576000);
			cfsetospeed(&options, B576000);
			break;
			
		case 921600:
			cfsetispeed(&options, B921600);
			cfsetospeed(&options, B921600);
			break;
			
		case 1000000:
			cfsetispeed(&options, B1000000);
			cfsetospeed(&options, B1000000);
			break;
			
		case 1152000:
			cfsetispeed(&options, B1152000);
			cfsetospeed(&options, B1152000);
			break;
			
		case 1500000:
			cfsetispeed(&options, B1500000);
			cfsetospeed(&options, B1500000);
			break;
			
		case 2000000:
			cfsetispeed(&options, B2000000);
			cfsetospeed(&options, B2000000);
			break;
			
		case 2500000:
			cfsetispeed(&options, B2500000);
			cfsetospeed(&options, B2500000);
			break;
			
		case 3000000:
			cfsetispeed(&options, B3000000);
			cfsetospeed(&options, B3000000);
			break;
			
		case 3500000:
			cfsetispeed(&options, B3500000);
			cfsetospeed(&options, B3500000);
			break;
			
		case 4000000:
			cfsetispeed(&options, B4000000);
			cfsetospeed(&options, B4000000);
			break;
			
		default:
			cfsetispeed(&options, B9600);
			cfsetospeed(&options, B9600);
			break;
	}

	switch( nBits )
	{
		case 7:
			options.c_cflag |= CS7;
			break;
			
		case 8:
			options.c_cflag |= CS8;
			break;
	}

	switch( nEvent )
	{
		case 'o':
		case 'O':					   //��У��
			options.c_cflag |= PARENB;
			options.c_cflag |= PARODD;
			options.c_iflag |= (INPCK | ISTRIP);
			break;

		case 'e':
		case 'E':					   //żУ��
			options.c_iflag |= (INPCK | ISTRIP);
			options.c_cflag |= PARENB;
			options.c_cflag &= ~PARODD;
			break;

		case 'n':
		case 'N':					  //��У��
			options.c_cflag &= ~PARENB;
			break;
	}

	switch( nStop )
	{
		case 1:	
			options.c_cflag &=	~CSTOPB;
			break;
			
		case 2: 
			options.c_cflag |=	CSTOPB;
			break;

	}


	 /*
     * ֻ������Ϊ����ʱ��������������Ч��������ڶ�������
     */
	options.c_cc[VTIME] = 0; // �ȴ�ʱ�䣬��λ�ٺ���
	options.c_cc[VMIN] = 0; // ��С�ֽ��� ������

	
	tcflush(fd, TCIOFLUSH); // TCIFLUSHˢ��������С�
							// TCOFLUSHˢ��������С� 
							// TCIOFLUSHˢ�����롢������С�



	
    if (tcsetattr(fd, TCSANOW, &options) < 0)  // TCSANOW������Ч��
	{
		printf("Com set options Erro!\n");
		return -1;
    }

}




void CloseEpollSerial(struct epoll_event *event)
{
	/*
	close(epoll_data->epid);
	close(epoll_data->event.data.fd);
	*/
}


int EpollSerialRead(int epfd, int fd, char * ReadBuff, const int ReadLen)
{
	
	int read_len = 0;
	int len = 0;
	int rdlen = 0;

	//���濪ʼepoll�ȴ�
	
	int i = 0, witeNum = 0;

	struct epoll_event *events;
	events = calloc (EV_MAX_NUM+1, sizeof(events));
	
	while (1) 
	{
		witeNum = epoll_wait(epfd, fd, EV_MAX_NUM, 50);
		printf("witeNum0 = %d\n   ", witeNum);
		if( witeNum == 0)
		{
			//usleep(100);
			return 0;
		}

		
		//printf("witeNum = %d\n", witeNum);
		
		for (i = 0; i < witeNum; i++) 
		{

			if ((events[i].events & EPOLLERR) || (events[i].events & EPOLLHUP) || (!(events[i].events & EPOLLIN))) 
			{
				printf("no data!\n");
				break;
			} 
			else if (events[i].events & EPOLLIN)  //�����ݽ���  ��������
			{
				
				len = read(events[i].data.fd, ReadBuff, ReadLen);
				tcdrain(fd);//�ȴ����������������
			    tcflush(fd,TCIOFLUSH);//ˢ��(����)���뻺��(�ն����������ѽ��յ������û�������δ��)���������(�û������Ѿ�д������δ����)

				if(len != ReadLen) //��α�֤ÿ�ζ�������Щ�ֽ��ֲ������� 
				{
					memset(ReadBuff, 0, sizeof(ReadBuff));
				}
				
				if( len == ReadLen)
					return len;


				/*
				while(true){
					rdlen = read(events[i].data.fd, ReadBuff + len, ReadLen);
					len+=rdlen;
					if(len==ReadLen)
					{
						return len;
					}
				}
				*/


				
			}
		}
	}

	return len ;
}


void EpollSerialParse(int epfd,  struct epoll_event *event, struct Com_command *command)
{
	char ReadBuff;
	int ret;
	
	static int parse_step = 0;
	
	while(1)
	{
		ret = EpollSerialRead(epfd, event, ReadBuff, 1); 
		if(ret == 0)
		{
			
			memset(command, 0, sizeof(command));  //????????????????
		}
		else
		{
			switch(parse_step)
			{
				case 0: //  ǰ���� AA
				if(ReadBuff == 0xAA)
				{
					memset(command, 0, sizeof(command));
					command->header[0] = ReadBuff;
					parse_step = 1;
				};
				break;
				
				
				case 1: //  ǰ���� 55
				if(ReadBuff == 0x55)
				{
					command->header[1] = ReadBuff;
					parse_step = 2;
				}
				else
				{
					if(ReadBuff == 0xAA)
					{
						parse_step = 1;
					}
					else
					{
						memset(command, 0, sizeof(command));
						parse_step = 0;
					}	
				};
				break;
				
				
				case 2: //  ����
				
				if(++command->count < 2)
				{
					command->len = ((unsigned short)ReadBuff << 8);
					command->checksum += ReadBuff;
				}
				else
				{
					command->len |= (unsigned short)ReadBuff;
					command->checksum += ReadBuff;
					command->count = 0;
					parse_step = 4;
				}
				break;
				
				
				case 3: //  ����
				command->direction = ReadBuff;
				command->checksum += ReadBuff;
				parse_step = 4;
				break;
				
				
				case 4: //  ����
				command->command = ReadBuff;
				command->checksum += ReadBuff;
				parse_step = 5;
				break;
				
				
				case 5: //  ����
				command->data[command->count] = ReadBuff;
				command->checksum += ReadBuff;
				if(++command->count == (command->len))
				{
					command->count = 0;
					parse_step = 6;		
				}
				break;
				
				
				case 6: //  У���ֽ�
				command->checksum +=(0xAA + 0x55);
				if(command->checksum == ReadBuff)
				{
					command->flag = 1;
					parse_step = 0;
				}
				else
				{
					printf("COM_COMMAND ERR :BAD CKECKSUM! \n");
					memset(command, 0, sizeof(command));
					parse_step = 0;
				}			
				break;
				
				default : //  reset
				memset(command, 0, sizeof(command));
				parse_step = 0;
				break;
			}
		}
	
	}
		
}







/*
parse()
{
		switch(command->direction)
		{
			case 0x00://����

			break;



			case 0x80://��Ӧ

			break;

			default:
			printf("COM_COMMAND ERR :BAD DIRECTION! \n");
			memset(command, 0, sizeof(command));
			parse_step = 0;
			break;
		}
		
		switch(command->command)
		{
			case 0x00://������

			break;	

			case 0x01://д����

			break;

			case 0x02://������

			break;

			case 0x03://д����

			break;

			case 0x04://����

			break;

			case 0x05://д��

			break;				

			case 0x30://͸��
			case 0x31:
			case 0x32:

			break;	


			default:
			printf("COM_COMMAND ERR :BAD DIRECTION! \n");
			memset(command, 0, sizeof(command));
			parse_step = 0;
			break;
		}
	
		
}
*/



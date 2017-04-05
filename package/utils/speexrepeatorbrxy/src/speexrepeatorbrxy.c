#include <stdio.h>
#include <pthread.h>
#include <sys/time.h>
#include <time.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include<string.h>
#include <sys/socket.h>
#include <netinet/in.h>





struct dts_client_struct{
    int power_weight;
    int count;
    int inuse;
    int is_last;
    struct sockaddr_in *client_addr;
    struct play_task *next;
};

struct dts_client_header{
    int count;
    struct dts_client_struct *next;
};
struct global_data_struct{
    pthread_mutex_t dts_use_mutex;
    int server_socket;
    struct sockaddr_in server_addr;
};

struct global_data_struct global_data;
struct dts_client_header dts_header;

int init_server_socket(void)
{
    global_data.server_addr.sin_family = AF_INET;
    global_data.server_addr.sin_port = htons(8735);
    global_data.server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if ( (global_data.server_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("ERROR--init_server_socket:  open server_socket fail: ");
        return -1;
    }
    if (bind(global_data.server_socket, (struct sockaddr *)&global_data.server_addr, sizeof(global_data.server_addr)) < 0)
    {
        perror("ERROR--init_server_socket:  bind server_socket fail: ");
        close(global_data.server_socket);
        return -1;
    }
    return 1;
}

void *receive_dts_client_connect(void *ptr)
{
    int i;
    int ret;
    char buff[512]={0};
    int sockaddr_in_length;
    char *returnstring="brxyrepeator";
    int returnstringlength;
    int recvflag;//0不正确 1正确的客户端
    struct dts_client_struct *dts_client;
    struct dts_client_struct *a,*b;
    returnstringlength=strlen(returnstring);
    struct sockaddr_in *client_addr;
    sockaddr_in_length=sizeof(struct sockaddr_in);
    init_server_socket();
    while(1)
    {
        client_addr=malloc(sockaddr_in_length);
        recvflag=0;
        memset(buff,0,512);
        //ret = recvfrom(sock, buff, 511, 0, (struct sockaddr*)&clientAddr, &len);
        ret=recvfrom(global_data.server_socket,buff,512,0,(struct sockaddr*)client_addr,sockaddr_in_length);
        if(ret>0)
        {
            if(strcmp(buff,"brxydts")==0)
            {
                //n = sendto(sock, buff, n, 0, (struct sockaddr *)&clientAddr, sizeof(clientAddr));
                if((ret=sendto(global_data.server_socket,returnstring,returnstringlength,0,(struct sockaddr *)client_addr,sockaddr_in_length))<0)
                {
                    recvflag=0;
                    printf("ERROR--receive_dts_client_connect:  sendto ret=%d\n",ret);
                }
                else
                {
                    recvflag=1;

                }
            }
        }
        else
        {
            recvflag=0;
            printf("ERROR--receive_dts_client_connect:  recvfrom ret=%d\n",ret);
        }
        if(recvflag==1)
        {
            dts_client=(struct dts_client_struct*)malloc(sizeof(struct dts_client_struct));
            dts_client->client_addr=client_addr;
            dts_client->is_last=1;
            dts_client->inuse=1;
            pthread_mutex_lock(&global_data.dts_use_mutex);
            if(dts_header.count==0)
            {
                dts_header.next=dts_client;
                dts_header.count++;
            }
            else
            {
                b=dts_header.next;
                while(1)
                {
                    if(b->is_last==1)
                    {
                        b->next=dts_client;

                        b->is_last=0;
                        dts_header.count++;
                        break;
                    }
                    else
                    {
                        b=b->next;
                    }
                }
            }
            pthread_mutex_unlock(&global_data.dts_use_mutex);

        }
    }

    return;

}


void *receive_from_speexsend(void *ptr)
{
    int i,ret;
    int serversocket;
    char buf[512];
    struct sockaddr_in client_addr,server_addr;
    struct dts_client_struct *a,*b;

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(8736);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if ( (serversocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("ERROR--init_server_socket:  open server_socket fail: ");
        return ;
    }
    if (bind(serversocket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("ERROR--init_server_socket:  bind server_socket fail: ");
        close(serversocket);
        return ;
    }

    while(1)
    {

        ret=recvfrom(serversocket,buf,512,0,(struct sockaddr*)&client_addr,sizeof(struct sockaddr));
        if(ret>0)
        {
            pthread_mutex_lock(&global_data.dts_use_mutex);
            if(dts_header.count==0)
            {
                printf("WARING--receive_from_speexsend:  none client\n");
            }
            else
            {
                b=dts_header.next;
                while(1)
                {
                    if(b->inuse==1)
                    {
                        if(sendto(serversocket,buf,ret,0,(struct sockaddr *)b->client_addr,sizeof(struct sockaddr))<0)
                        {
                            b->inuse=0;
                        }
                    }

                    if(b->is_last==1)
                    {
                        break;
                    }
                    else
                    {
                        b=b->next;
                    }
                }
            }
            pthread_mutex_unlock(&global_data.dts_use_mutex);
        }
    }
    return;
}





int main(int argc,char *argv[])
{

    int i,ret,threadret;
    pthread_t dtsthread,recvthread;




    dts_header.count=0;


    ret=pthread_create(&dtsthread, NULL, &receive_dts_client_connect, NULL);
    if(ret!=0)
    {
        printf("ERROR--main:  Create receive_dts_client_connect pthread error!\n");
        return -1;
    }

    ret=pthread_create(&dtsthread, NULL, &receive_from_speexsend, NULL);
    if(ret!=0)
    {
        printf("ERROR--main:  Create receive_from_speexsend pthread error!\n");
        return -1;
    }

    return 0;



}






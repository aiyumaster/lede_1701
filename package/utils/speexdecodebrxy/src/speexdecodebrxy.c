#include <stdio.h>
#include <speex/speex_header.h>
#include <speex/speex_stereo.h>
//#include <speex/speex_preprocess.h>
#include <speex/speex.h>
#include "wav_io.h"
#include<string.h>
//#include <alsa/asoundlib.h>
#include <pthread.h>
#include <sys/time.h>
#include <time.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <linux/soundcard.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define MAX_FRAME_SIZE 2000
#define MAX_FRAME_BYTES 2000
struct speexfilehead{
    char name[6];
    int channel;
};
struct speexdatahead{
    int nbytes;
    int channel;
};
typedef struct speexfilehead speexfileheader;
typedef struct speexdatahead speexdataheader;
speexfileheader sfh;
speexdataheader sdh;
char *outfile;
int fout;
struct brxy_speex_decode_play_struct{
    char *filename;
    short filebuf[2000];
    short playbuf[2000];
    short playbackbuf[2000];
    int fileflay;//0  可以写入   1  可以读出
    int filechannel;
    int pcm_bytes;
    int playchannel;
    int filethreadend;
    int playthreadend;
    pthread_t filethread;
    pthread_t playthread;
    //struct snd_pcm_t *handle;
    //struct snd_pcm_hw_params_t *params;
    int frame_period_second;
    pthread_mutex_t mutex;//互斥锁
    int fd;
    int rate;
    int format;
    int channel;
    int time_per_frame;
    char *serverip;
    char *serverport;
};
struct brxy_speex_decode_play_struct brxy_speex_global_data;
void *speex_read_thread(void * ptr)
{
    FILE *infile;
    char inbuf[2000]={0};
    char *inbufpointer;
    short decodebuf[2000]={0};
    short outbuf[2000]={0};
    SpeexBits speexbits;
    SpeexStereoState stereo = SPEEX_STEREO_STATE_INIT;
    int chan=1;
    void *st;
    spx_int32_t frame_size;
    int fileheaderlength;
    int dataheaderlength;
    int ret;
    int i;
    int usetime=19;
    int readlength;

    int fromserverlength;
    int dtsclient;
    struct sockaddr_in serveraddr;
    int sockaddrlength;
    char buf[36]={0};
    char *sendok="brxydts";
    int n;

    struct timeval tv;

    unsigned int ms,oldms,usems,usems1;


    sockaddrlength=sizeof(struct sockaddr_in);

    if ((dtsclient=socket(AF_INET, SOCK_DGRAM, 0)) <0)
    {
        perror("ERROR--speex_read_thread:  open client_socket fail: ");
        return ;
    }
    //int nZero = 0;
    //setsockopt( dtsclient, SOL_SOCKET, SO_RCVBUF, ( char * )&nZero, sizeof( int ) );
    //int nRecvBufLen =512; //设置为1K
    //setsockopt( dtsclient, SOL_SOCKET, SO_RCVBUF, ( const char* )&nRecvBufLen, sizeof( int ) );
    int nZero = 0;
    setsockopt( dtsclient, SOL_SOCKET, SO_RCVBUF, ( char * )&nZero, sizeof( int ) );
    //int nRecvBufLen =512; //设置为1K
    //setsockopt( dtsclient, SOL_SOCKET, SO_RCVBUF, ( const char* )&nRecvBufLen, sizeof( int ));

    //int nZero = 0;
    setsockopt( dtsclient, SOL_SOCKET, SO_SNDBUF, ( char * )&nZero, sizeof( nZero ) );
    int nSendBufLen = 512; //设置为32K
    setsockopt( dtsclient, SOL_SOCKET, SO_SNDBUF, ( const char* )&nSendBufLen, sizeof( int ) );
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(atoi(brxy_speex_global_data.serverport));
    serveraddr.sin_addr.s_addr = inet_addr(brxy_speex_global_data.serverip);
    if (serveraddr.sin_addr.s_addr == INADDR_NONE)
    {
        printf("ERROR--speex_read_thread: serveraddr.sin_addr.s_addr == INADDR_NONE\n  ");
        return ;
    }





    sfh.name[0]=0;

    dataheaderlength=sizeof(speexdataheader);
    fileheaderlength=sizeof(speexfileheader);

    //infile=fopen(brxy_speex_global_data.filename,"rb");
   // if(infile==NULL)
   // {
    //    printf("ERROR--speex_read_thread:  open file %s fail\n",brxy_speex_global_data.filename);
   //     brxy_speex_global_data.filethreadend=1;
   // }
   // fread((void *)&sfh,fileheaderlength,1,infile);
   // if(strcmp(sfh.name,"brxy1")!=0)
   // {
    //    printf("ERROR--speex_read_thread:  file is not brxyspeex file");
    //    goto err1;
   // }
    speex_bits_init(&speexbits);
    st = speex_decoder_init(speex_lib_get_mode(SPEEX_MODEID_WB));
    speex_encoder_ctl(st, SPEEX_GET_FRAME_SIZE, &frame_size);





    n = sendto(dtsclient, sendok, strlen(sendok), 0, (struct sockaddr *)&serveraddr,sockaddrlength);
    if(n<0)
    {
        printf("ERROR--speex_read_thread:  cannot connect to server sendto\n");
        brxy_speex_global_data.filethreadend=1;
        return;
    }
    n=recvfrom(dtsclient,buf,36,0,(struct sockaddr*)&serveraddr,&sockaddrlength);
    if(n<0)
    {
        printf("ERROR--speex_read_thread:  cannot connect to server recvfrom\n");
        brxy_speex_global_data.filethreadend=1;
        return;
    }
    else
    {
        printf("NOTE--speex_read_thread:  get buf from server =%s\n",buf);
        if(strcmp(buf,"brxyrepeator")!=0)
        {
            printf("ERROR--speex_read_thread:  cannot connect to server not brxyrepeator\n");
            brxy_speex_global_data.filethreadend=1;
            return;
        }

    }

    while(1)
    {
        gettimeofday(&tv, NULL);
        oldms=(tv.tv_sec*1000)+(tv.tv_usec/1000);
        while(brxy_speex_global_data.fileflay)
        {
            usleep(1000);
        }
        /*if(feof(infile))
        {
            printf("ENDFILE--speex_read_thread: feof  end of the file %s\n",brxy_speex_global_data.filename);
            break;
        }
        if(fread((void *)&sdh,dataheaderlength,1,infile)!=1)
        {
            printf("ENDFILE--speex_read_thread:  end of the file %s\n",brxy_speex_global_data.filename);
            break;
        }
        if((readlength=fread(inbuf,1,sdh.nbytes,infile))!=sdh.nbytes)
        {
            printf("ERROR--speex_read_thread:  read speex data faile readlength= %d  sdh.nbytes=%d \n",readlength,sdh.nbytes);
            break;
        }*/
        //n=recvfrom(dtsclient,inbuf,512,0,(struct sockaddr*)&serveraddr,&sockaddrlength);
        n=recv(dtsclient,inbuf,512,0);
        if(n<0)
        {
            //printf("ERROR--speex_read_thread:  cannot connect to server recvfrom\n");
            continue;
        }
        else
        {
            //printf("NOTE--speex_read_thread:  get buf n=%d\n",n);
            memmove((void *)&sdh,inbuf,dataheaderlength);
            inbufpointer=inbuf+dataheaderlength;
            if((sdh.nbytes+dataheaderlength)!=n)
            {
                printf("WARNING--speex_read_thread:  (sdh.nbytes+dataheaderlength)!=n\n");
                continue;
            }
        }
        speex_bits_read_from(&speexbits,inbufpointer,sdh.nbytes);
        ret=speex_decode_int(st,&speexbits,decodebuf);
        speex_bits_reset(&speexbits);
        if (sdh.channel==2)
        {
            speex_decode_stereo_int(decodebuf, frame_size, &stereo);
        }

        for (i=0;i<frame_size*sdh.channel;i++)
        {
            outbuf[i]=le_short(decodebuf[i]);
        }
        pthread_mutex_lock(&brxy_speex_global_data.mutex);
        brxy_speex_global_data.pcm_bytes=frame_size*sdh.channel*2;
        memmove(brxy_speex_global_data.filebuf,outbuf,brxy_speex_global_data.pcm_bytes);
        brxy_speex_global_data.filechannel=sdh.channel;
        brxy_speex_global_data.fileflay=1;
        usetime=brxy_speex_global_data.time_per_frame;
        pthread_mutex_unlock(&brxy_speex_global_data.mutex);

        gettimeofday(&tv, NULL);
        ms=(tv.tv_sec*1000)+(tv.tv_usec/1000);
        usems=ms-oldms;
        //printf("----speexdata  length = %d  usems=%d\n",sdh.nbytes,usems);
        //printf("----speexdata  dataheaderlength = %d  fileheaderlength=%d\n",dataheaderlength,fileheaderlength);
        /*if(0<usems<brxy_speex_global_data.time_per_frame)
        {
            usems1=(brxy_speex_global_data.time_per_frame-usems)*1000;
            usleep(usems1);
        }*/
        //usleep(17000);
        //printf("----------speexdata  length = %d  usems=%d\n",sdh.nbytes,usems1);


        //printf("NOTE--speex_read_thread:  move data frame_size=%d   channel=%d\n",frame_size,sdh.channel);
    }
    speex_decoder_destroy(st);
    speex_bits_destroy(&speexbits);
err1:
    fclose(infile);
    brxy_speex_global_data.filethreadend=1;
    return ;
}
void *speex_read_thread_fromfile(void * ptr)
{
    FILE *infile;
    char inbuf[2000]={0};
    short decodebuf[2000]={0};
    short outbuf[2000]={0};
    SpeexBits speexbits;
    SpeexStereoState stereo = SPEEX_STEREO_STATE_INIT;
    int chan=1;
    void *st;
    spx_int32_t frame_size;
    int fileheaderlength;
    int dataheaderlength;
    int ret;
    int i;
    int usetime=19;
    int readlength;

    struct timeval tv;

    unsigned int ms,oldms,usems,usems1;

    sfh.name[0]=0;

    dataheaderlength=sizeof(speexdataheader);
    fileheaderlength=sizeof(speexfileheader);

    infile=fopen(brxy_speex_global_data.filename,"rb");
    if(infile==NULL)
    {
        printf("ERROR--speex_read_thread:  open file %s fail\n",brxy_speex_global_data.filename);
        brxy_speex_global_data.filethreadend=1;
    }
    fread((void *)&sfh,fileheaderlength,1,infile);
    if(strcmp(sfh.name,"brxy1")!=0)
    {
        printf("ERROR--speex_read_thread:  file is not brxyspeex file");
        goto err1;
    }
    speex_bits_init(&speexbits);
    st = speex_decoder_init(speex_lib_get_mode(SPEEX_MODEID_WB));
    speex_encoder_ctl(st, SPEEX_GET_FRAME_SIZE, &frame_size);
    while(1)
    {
        gettimeofday(&tv, NULL);
        oldms=(tv.tv_sec*1000)+(tv.tv_usec/1000);
        while(brxy_speex_global_data.fileflay)
        {
            usleep(1000);
        }
        if(feof(infile))
        {
            printf("ENDFILE--speex_read_thread: feof  end of the file %s\n",brxy_speex_global_data.filename);
            break;
        }
        if(fread((void *)&sdh,dataheaderlength,1,infile)!=1)
        {
            printf("ENDFILE--speex_read_thread:  end of the file %s\n",brxy_speex_global_data.filename);
            break;
        }
        if((readlength=fread(inbuf,1,sdh.nbytes,infile))!=sdh.nbytes)
        {
            printf("ERROR--speex_read_thread:  read speex data faile readlength= %d  sdh.nbytes=%d \n",readlength,sdh.nbytes);
            break;
        }
        speex_bits_read_from(&speexbits,inbuf,sdh.nbytes);
        ret=speex_decode_int(st,&speexbits,decodebuf);
        speex_bits_reset(&speexbits);
        if (sdh.channel==2)
        {
            speex_decode_stereo_int(decodebuf, frame_size, &stereo);
        }

        for (i=0;i<frame_size*sdh.channel;i++)
        {
            outbuf[i]=le_short(decodebuf[i]);
        }
        pthread_mutex_lock(&brxy_speex_global_data.mutex);
        brxy_speex_global_data.pcm_bytes=frame_size*sdh.channel*2;
        memmove(brxy_speex_global_data.filebuf,outbuf,brxy_speex_global_data.pcm_bytes);
        brxy_speex_global_data.filechannel=sdh.channel;
        brxy_speex_global_data.fileflay=1;
        usetime=brxy_speex_global_data.time_per_frame;
        pthread_mutex_unlock(&brxy_speex_global_data.mutex);

        gettimeofday(&tv, NULL);
        ms=(tv.tv_sec*1000)+(tv.tv_usec/1000);
        usems=ms-oldms;
        printf("----speexdata  length = %d  usems=%d\n",sdh.nbytes,usems);
        printf("----speexdata  dataheaderlength = %d  fileheaderlength=%d\n",dataheaderlength,fileheaderlength);
        /*if(0<usems<brxy_speex_global_data.time_per_frame)
        {
            usems1=(brxy_speex_global_data.time_per_frame-usems)*1000;
            usleep(usems1);
        }*/
        //usleep(17000);
        //printf("----------speexdata  length = %d  usems=%d\n",sdh.nbytes,usems1);


        printf("NOTE--speex_read_thread:  move data frame_size=%d   channel=%d\n",frame_size,sdh.channel);
    }
    speex_decoder_destroy(st);
    speex_bits_destroy(&speexbits);
err1:
    fclose(infile);
    brxy_speex_global_data.filethreadend=1;
    return ;
}
void *speex_read_thread_to_dsp(void * ptr)
{
    FILE *infile;
    char inbuf[2000]={0};
    short decodebuf[2000]={0};
    short outbuf[2000]={0};
    SpeexBits speexbits;
    SpeexStereoState stereo = SPEEX_STEREO_STATE_INIT;
    int chan=1;
    void *st;
    spx_int32_t frame_size;
    int fileheaderlength;
    int dataheaderlength;
    int ret;
    int i;

    struct timeval tv;

    unsigned int ms,oldms,usems;

    sfh.name[0]=0;

    dataheaderlength=sizeof(speexdataheader);
    fileheaderlength=sizeof(speexfileheader);

    infile=fopen(brxy_speex_global_data.filename,"rb");
    if(infile==NULL)
    {
        printf("ERROR--speex_read_thread:  open file %s fail\n",brxy_speex_global_data.filename);
        brxy_speex_global_data.filethreadend=1;
    }
    fread((void *)&sfh,fileheaderlength,1,infile);
    if(strcmp(sfh.name,"brxy1")!=0)
    {
        printf("ERROR--speex_read_thread:  file is not brxyspeex file");
        goto err1;
    }
    speex_bits_init(&speexbits);
    st = speex_decoder_init(speex_lib_get_mode(SPEEX_MODEID_WB));
    speex_encoder_ctl(st, SPEEX_GET_FRAME_SIZE, &frame_size);
    while(1)
    {
        gettimeofday(&tv, NULL);
        oldms=(tv.tv_sec*1000)+(tv.tv_usec/1000);
        if(feof(infile))
        {
            printf("ENDFILE--speex_read_thread: feof  end of the file %s\n",brxy_speex_global_data.filename);
            break;
        }
        if(fread((void *)&sdh,dataheaderlength,1,infile)!=1)
        {
            printf("ENDFILE--speex_read_thread:  end of the file %s\n",brxy_speex_global_data.filename);
            break;
        }
        if(fread(inbuf,1,sdh.nbytes,infile)!=sdh.nbytes)
        {
            printf("ERROR--speex_read_thread:  read speex data faile\n");
            break;
        }
        speex_bits_read_from(&speexbits,inbuf,sdh.nbytes);
        ret=speex_decode_int(st,&speexbits,decodebuf);
        speex_bits_reset(&speexbits);
        if (sdh.channel==2)
        {
            speex_decode_stereo_int(decodebuf, frame_size, &stereo);
        }

        for (i=0;i<frame_size*sdh.channel;i++)
        {
            outbuf[i]=le_short(decodebuf[i]);
        }
        //pthread_mutex_lock(&brxy_speex_global_data.mutex);
        brxy_speex_global_data.pcm_bytes=frame_size*sdh.channel*2;
        //memmove(outbuf,brxy_speex_global_data.filebuf,brxy_speex_global_data.pcm_bytes);
       // brxy_speex_global_data.filechannel=sdh.channel;
       // brxy_speex_global_data.fileflay=1;
        //pthread_mutex_unlock(&brxy_speex_global_data.mutex);
        pthread_mutex_lock(&brxy_speex_global_data.mutex);
        brxy_speex_global_data.pcm_bytes=frame_size*sdh.channel*2;
        memmove(brxy_speex_global_data.filebuf,outbuf,brxy_speex_global_data.pcm_bytes);
        brxy_speex_global_data.filechannel=sdh.channel;
        brxy_speex_global_data.fileflay=1;
        pthread_mutex_unlock(&brxy_speex_global_data.mutex);
        write(brxy_speex_global_data.fd,brxy_speex_global_data.filebuf,brxy_speex_global_data.pcm_bytes);
        gettimeofday(&tv, NULL);
        ms=(tv.tv_sec*1000)+(tv.tv_usec/1000);
        usems=ms-oldms;
       // if(0<usems<20)
       // {
        //    usems=(20-usems)*1000;
         //   usleep(usems);
       // }
        printf("speexdata  length = %d  usems=%d\n",sdh.nbytes,usems);
        //usleep(20000);
        printf("NOTE--speex_read_thread:  move data frame_size=%d   channel=%d\n",frame_size,sdh.channel);
    }
    speex_decoder_destroy(st);
    speex_bits_destroy(&speexbits);
err1:
    fclose(infile);
    //close(fout);
    brxy_speex_global_data.filethreadend=1;
    return ;
}
void *speex_play_thread(void * ptr)
{
    int r=1;
    int play_frame_size;
    struct timeval tv;
    int usetime=19;
    unsigned int ms,oldms,usems;
    while(1)
    {
        gettimeofday(&tv, NULL);
        oldms=(tv.tv_sec*1000)+(tv.tv_usec/1000);
        //while(brxy_speex_global_data.fileflay==0)
       // {
        //    usleep(100);
       // }
        pthread_mutex_lock(&brxy_speex_global_data.mutex);
        if(brxy_speex_global_data.fileflay==1)
        {
            memmove(brxy_speex_global_data.playbuf,brxy_speex_global_data.filebuf,brxy_speex_global_data.pcm_bytes);
            //play_frame_size=brxy_speex_global_data.pcm_bytes/(2*brxy_speex_global_data.filechannel);
            brxy_speex_global_data.fileflay=0;
            //pthread_mutex_unlock(&brxy_speex_global_data.mutex);
           // printf("NOTE--speex_play_thread:  play_frame_size=%d\n",play_frame_size);
            //printf("NOTE--speex_play_thread:  use file data bytes=%d   channel=%d\n",brxy_speex_global_data.pcm_bytes,brxy_speex_global_data.filechannel);
        }
        else
        {
            memmove(brxy_speex_global_data.playbuf,brxy_speex_global_data.playbackbuf,brxy_speex_global_data.pcm_bytes);
            //printf("NOTE--speex_play_thread:  use playback data\n");
            //usleep(10000);
        }
        brxy_speex_global_data.time_per_frame=usetime;
        pthread_mutex_unlock(&brxy_speex_global_data.mutex);
        //r=snd_pcm_writei(brxy_speex_global_data.handle,(char*)brxy_speex_global_data.playbuf,play_frame_size);

        //if(!(r>0))
       // {
           // printf("ERROR--speex_play_thread:  write alsa pcm device fail r=%d\n",r);
           // sleep(1);
           // snd_pcm_prepare(brxy_speex_global_data.handle);
           // printf("schema_play_info  snd_pcm_prepare--------- r = %d\n",r);
        //}
        write(brxy_speex_global_data.fd, (char*)brxy_speex_global_data.playbuf,brxy_speex_global_data.pcm_bytes);
        if(brxy_speex_global_data.filethreadend==1)
        {
            printf("ENDTHREAD--speex_play_thread:  speex_play_thread is end\n");
            break;
        }
        usleep(100);
        gettimeofday(&tv, NULL);
        ms=(tv.tv_sec*1000)+(tv.tv_usec/1000);
        usems=ms-oldms;
        if(15<usems<24)
        {
            usetime=usems;
        }

        //printf("write usems=%d\n",usems);
    }
    //snd_pcm_close(brxy_speex_global_data.handle);
    close(brxy_speex_global_data.fd);
    return;
}
/*void *speex_play_thread_alsa(void * ptr)
{
    int r=1;
    int play_frame_size;
    struct timeval tv;

    unsigned int ms,oldms,usems;
    while(1)
    {
        gettimeofday(&tv, NULL);
        oldms=(tv.tv_sec*1000)+(tv.tv_usec/1000);
        //pthread_mutex_lock(&brxy_speex_global_data.mutex);
        if(brxy_speex_global_data.fileflay==1)
        {
            memmove(brxy_speex_global_data.filebuf,brxy_speex_global_data.playbuf,brxy_speex_global_data.pcm_bytes);
            play_frame_size=brxy_speex_global_data.pcm_bytes/(2*brxy_speex_global_data.filechannel);
            brxy_speex_global_data.fileflay=0;
            //pthread_mutex_unlock(&brxy_speex_global_data.mutex);
            printf("NOTE--speex_play_thread:  play_frame_size=%d\n",play_frame_size);
            r=snd_pcm_writei(brxy_speex_global_data.handle,(char*)brxy_speex_global_data.playbuf,play_frame_size);

            if(!(r>0))
            {
                printf("ERROR--speex_play_thread:  write alsa pcm device fail r=%d\n",r);
                sleep(1);
                snd_pcm_prepare(brxy_speex_global_data.handle);
                printf("schema_play_info  snd_pcm_prepare--------- r = %d\n",r);
            }
            printf("NOTE--speex_play_thread:  use file data bytes=%d   channel=%d\n",brxy_speex_global_data.pcm_bytes,brxy_speex_global_data.filechannel);
        }
        else
        {
            memmove(brxy_speex_global_data.playbackbuf,brxy_speex_global_data.playbuf,brxy_speex_global_data.pcm_bytes);
            printf("NOTE--speex_play_thread:  use playback data\n");
            usleep(10000);
        }

        if(brxy_speex_global_data.filethreadend==1)
        {
            printf("ENDTHREAD--speex_play_thread:  speex_play_thread is end\n");
            break;
        }
        gettimeofday(&tv, NULL);
        ms=(tv.tv_sec*1000)+(tv.tv_usec/1000);
        usems=ms-oldms;
        printf("write usems=%d\n",usems);
    }
    snd_pcm_close(brxy_speex_global_data.handle);
    return;
}*/
int set_dsp_parameters()
{
    int wavfd; /* wav文件的描述符 */
    int arg;   /* ioctl参数 */
    int ret;   /* 返回值 */
    int parm;
    parm=0x02<<16+0x08;
    ret = ioctl(brxy_speex_global_data.fd, SNDCTL_DSP_SETFRAGMENT, &parm);
    if (ret==-1)
    {
        printf("ERROR--init_dsp:  set format %d fail\n",brxy_speex_global_data.format);
        close(brxy_speex_global_data.fd);
        return -1;
    }
    ret = ioctl(brxy_speex_global_data.fd, SOUND_PCM_WRITE_BITS, &brxy_speex_global_data.format);
    if (ret==-1)
    {
        printf("ERROR--init_dsp:  set format %d fail\n",brxy_speex_global_data.format);
        close(brxy_speex_global_data.fd);
        return -1;
    }
    ret = ioctl(brxy_speex_global_data.fd, SOUND_PCM_WRITE_CHANNELS, &brxy_speex_global_data.channel);
    if (ret==-1)
    {
        printf("ERROR--init_dsp:  set channel %d fail\n",brxy_speex_global_data.channel);
        close(brxy_speex_global_data.fd);
        return -1;
    }
    ret = ioctl(brxy_speex_global_data.fd, SOUND_PCM_WRITE_RATE, &brxy_speex_global_data.rate);
    if (ret==-1)
    {
        printf("ERROR--init_dsp:  set rate %d fail\n",brxy_speex_global_data.rate);
        close(brxy_speex_global_data.fd);
        return -1;
    }
    return 1;
}
int init_dsp()
{
    int wavfd; /* wav文件的描述符 */
    int arg;   /* ioctl参数 */
    int ret;   /* 返回值 */
    brxy_speex_global_data.fd=open("/dev/dsp", O_WRONLY);
    if (brxy_speex_global_data.fd < 0) {
        printf("open of /dev/dsp failed\n");
        return -1;
    }
    if(set_dsp_parameters()<0)
    {
        close(brxy_speex_global_data.fd);
        return -1;
    }

    return 1;
}
int init_outfile()
{
    int wavfd; /* wav文件的描述符 */
    int arg;   /* ioctl参数 */
    int ret;   /* 返回值 */
    fout = open(outfile, O_WRONLY|O_CREAT );
    if (fout < 0) {
        printf("open of %s failed\n",outfile);
        return -1;
    }
    return 1;
}
/*int init_alsa(void *ptr)
{
    //int k=16000;
    int rate=16000;
    int format=16;
    int channel=2;
    if(snd_pcm_open(&brxy_speex_global_data.handle,"dmix",SND_PCM_STREAM_PLAYBACK,0)<0)
    {
        printf("open alsa dmix false\n");

        return -1;
    }
    if(snd_pcm_hw_params_malloc(&brxy_speex_global_data.params)!=0)
    {
        snd_pcm_close(brxy_speex_global_data.handle);
        return -1;
    }
    if(snd_pcm_hw_params_any(brxy_speex_global_data.handle,brxy_speex_global_data.params)!=0)
    {
        snd_pcm_close(brxy_speex_global_data.handle);
        return -1;
    }
    if(snd_pcm_hw_params_set_access(brxy_speex_global_data.handle,brxy_speex_global_data.params,SND_PCM_ACCESS_RW_INTERLEAVED)!=0)
    {
        snd_pcm_close(brxy_speex_global_data.handle);
        return -1;
    }
    if(snd_pcm_hw_params_set_format(brxy_speex_global_data.handle,brxy_speex_global_data.params,SND_PCM_FORMAT_S16_LE)!=0)
    {
        snd_pcm_close(brxy_speex_global_data.handle);
        return -1;
    }
    if(snd_pcm_hw_params_set_rate_near(brxy_speex_global_data.handle,brxy_speex_global_data.params,&rate,0)!=0)
    {
        snd_pcm_close(brxy_speex_global_data.handle);
        return -1;
    }
    if(snd_pcm_hw_params_set_channels(brxy_speex_global_data.handle,brxy_speex_global_data.params,channel)!=0)
    {
        snd_pcm_close(brxy_speex_global_data.handle);
        return -1;
    }
    if(snd_pcm_hw_params_get_period_size(brxy_speex_global_data.params,&brxy_speex_global_data.frame_period_second,0)!=0)
    {
        return -1;
    }

    if(snd_pcm_hw_params(brxy_speex_global_data.handle,brxy_speex_global_data.params)!=0)
    {
        snd_pcm_close(brxy_speex_global_data.handle);
        return -1;
    }
    snd_pcm_hw_params_free(brxy_speex_global_data.params);
    //snd_pcm_close(brxy_speex_global_data.handle);
    return 1;
}*/


int main(int argc,char *argv[])
{
    int brxy_threadptr;
    int ret=0;
    int i;
    void *thrd_ret;
    if(argc<3)
    {
        printf("ERROR--main:  argc is less than 3\n");
        return 0;
    }
    //outfile=argv[2];
    pthread_mutexattr_init(&brxy_speex_global_data.mutex);
    brxy_speex_global_data.filethreadend=0;
    brxy_speex_global_data.playthreadend=0;
    brxy_speex_global_data.filename=argv[1];
    brxy_speex_global_data.pcm_bytes=1280;
    brxy_speex_global_data.filechannel=2;
    brxy_speex_global_data.channel=2;
    brxy_speex_global_data.rate=16000;
    brxy_speex_global_data.format=16;
    brxy_speex_global_data.time_per_frame=19;
    brxy_speex_global_data.fileflay=0;
    brxy_speex_global_data.serverip=argv[1];
    brxy_speex_global_data.serverport=argv[2];
    for(i=0;i<2000;i++)
    {
        brxy_speex_global_data.playbackbuf[i]=0;
    }
    //brxy_speex_global_data.playbackbuf[2000]={0};
    //init_alsa(&brxy_threadptr);
    //if(init_outfile()<0)
   // {
    //    return -1;
   // }
    if(init_dsp()<0)
    {
        return -1;
    }
    ret=pthread_create(&brxy_speex_global_data.filethread, NULL, &speex_read_thread, NULL);
    if(ret!=0)
    {
        printf("ERROR--main:  Create speex_read_thread pthread error!\n");
        return -1;
    }
    ret=pthread_create(&brxy_speex_global_data.playthread, NULL, &speex_play_thread, NULL);
    if(ret!=0)
    {
        printf("ERROR--main:  Create speex_play_thread pthread error!\n");
        return -1;
    }
    while(brxy_speex_global_data.filethreadend!=1)
    {
        sleep(1);
    }
    pthread_join(brxy_speex_global_data.filethread,&thrd_ret);
    pthread_join(brxy_speex_global_data.playthread,&thrd_ret);
    printf("main end\n");
    return 1;
}



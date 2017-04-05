#include <stdio.h>
#include <speex/speex_header.h>
#include <speex/speex_stereo.h>
//#include <speex/speex_preprocess.h>
#include <speex/speex.h>
#include "wav_io.h"
#include<string.h>


#include <pthread.h>
#include <sys/time.h>
#include <time.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <linux/soundcard.h>



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
    int quit;
};
struct brxy_speex_decode_play_struct brxy_speex_global_data;
FILE *infile,*outfile;
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
    brxy_speex_global_data.fd=open("/dev/dsp",O_RDONLY);
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

void *read_dsp_thread(void *ptr)
{
    struct timeval tv;
    short inshort[2000];
    char outdata[2000];
    short readdata[2000];
    int readnbytes,neednbytes;
    unsigned int ms,oldms,usems;
    int fileheaderlength;
    int dataheaderlength;
    SpeexBits speexbits;
    void *st;
    spx_int32_t quality=-1;
    spx_int32_t frame_size;
    spx_int32_t rate=0;
    int chan=2;
    int fmt=16;
    int i;
    spx_int32_t size=0;
    if(init_dsp()<0)
    {
        printf("ERROR-main:  init_dsp fail\n");
        return;
    }


    st = speex_encoder_init(speex_lib_get_mode(SPEEX_MODEID_WB));
    speex_encoder_ctl(st, SPEEX_GET_FRAME_SIZE, &frame_size);
    speex_encoder_ctl(st, SPEEX_SET_SAMPLING_RATE, &rate);
    quality=4;
    speex_encoder_ctl(st, SPEEX_SET_QUALITY, &quality);
    speex_bits_init(&speexbits);
    //nb_samples = read_samples(infile,frame_size,fmt,chan,1,inshort, NULL, &size);
    fwrite((void *)&sfh,sizeof(speexfileheader),1,outfile);

    fileheaderlength=sizeof(speexfileheader);
    //printf("fileheader  length = %d\n",fileheaderlength);
    dataheaderlength=sizeof(speexdataheader);
    //printf("dataheader  length = %d\n",dataheaderlength);

    neednbytes=frame_size*2*2;


    while(1)
    {
        gettimeofday(&tv, NULL);
        oldms=(tv.tv_sec*1000)+(tv.tv_usec/1000);
        readnbytes=read(brxy_speex_global_data.fd,(char *)readdata,neednbytes);
        printf("NOTE--read_dsp_thread:  neednbytes = %d   readnbytes = %d\n",neednbytes,readnbytes);
        if(readnbytes<neednbytes)
        {

            continue;
        }
        for(i=0;i<frame_size*2;i++)
        {
            inshort[i]=le_short(readdata[i]);
        }
        if(2==2)
        {
            speex_encode_stereo_int(inshort, frame_size, &speexbits);
        }
        speex_encode_int(st, inshort, &speexbits);
        speex_bits_insert_terminator(&speexbits);
        sdh.nbytes = speex_bits_write(&speexbits,outdata,MAX_FRAME_BYTES);
        fwrite((void *)&sdh,sizeof(speexdataheader),1,outfile);


        fwrite(outdata,1,sdh.nbytes,outfile);
        speex_bits_reset(&speexbits);

        //nb_samples = read_samples(infile,frame_size,fmt,chan,1,inshort, NULL, &size);

        if (brxy_speex_global_data.quit==1)
        {
            break;
        }
        gettimeofday(&tv, NULL);
        ms=(tv.tv_sec*1000)+(tv.tv_usec/1000);
        usems=ms-oldms;
        printf("speexdata  length = %d  usems=%d\n",sdh.nbytes,usems);
    }
    speex_encoder_destroy(st);
    speex_bits_destroy(&speexbits);


}
int main(int argc,char **argv)
{
    char *outfilename;
    int fileheaderlength;
    int dataheaderlength;
    int ret;
    int *re;
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
    brxy_speex_global_data.quit=0;
    if(argc!=2)
    {
        fprintf(stdout,"ERROR:  paramiters number argc!=3\n");
        return 0;
    }
    strcpy(sfh.name,"brxy1");
    sfh.channel=2;
    sdh.nbytes=0;
    sdh.channel=2;

    outfilename=argv[1];
    outfile=fopen(outfilename,"w");
    if(outfile==NULL)
    {
        printf("ERROR:  %s is not exist\n",outfilename);
        return 0;
    }
    fwrite((void *)&sfh,sizeof(speexfileheader),1,outfile);
    ret=pthread_create(&brxy_speex_global_data.filethread, NULL, &read_dsp_thread, NULL);
    if(ret!=0)
    {
        printf("ERROR--main:  Create speex_read_thread pthread error!\n");
        return -1;
    }
    while(1)
    {
        if(getchar()=='q')
        {
            break;
        }
        sleep(1);
    }
    brxy_speex_global_data.quit=1;
    pthread_join(brxy_speex_global_data.filethread,&re);
    fclose(outfile);
    return 1;
}



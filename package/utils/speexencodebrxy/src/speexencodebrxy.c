#include <stdio.h>
#include <speex/speex_header.h>
#include <speex/speex_stereo.h>
//#include <speex/speex_preprocess.h>
#include <speex/speex.h>
#include "wav_io.h"
#include<string.h>
#include <sys/time.h>
#include <time.h>

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


/* Convert input audio bits, endians and channels */
static int read_samples(FILE *fin,int frame_size, int bits, int channels, int lsb, short * input, char *buff, spx_int32_t *size)
{
   unsigned char in[MAX_FRAME_BYTES*2];
   int i;
   short *s;
   int nb_read;

   if (size && *size<=0)
   {
      return 0;
   }
   /*Read input audio*/
   if (size)
      *size -= bits/8*channels*frame_size;
   if (buff)
   {
      for (i=0;i<12;i++)
         in[i]=buff[i];
      nb_read = fread(in+12,1,bits/8*channels*frame_size-12, fin) + 12;
      if (size)
         *size += 12;
   } else {
      nb_read = fread(in,1,bits/8*channels* frame_size, fin);
   }
   nb_read /= bits/8*channels;

   /*fprintf (stderr, "%d\n", nb_read);*/
   if (nb_read==0)
      return 0;

   s=(short*)in;
   if(bits==8)
   {
      /* Convert 8->16 bits */
      for(i=frame_size*channels-1;i>=0;i--)
      {
         s[i]=(in[i]<<8)^0x8000;
      }
   } else
   {
      /* convert to our endian format */
      for(i=0;i<frame_size*channels;i++)
      {
         if(lsb)
            s[i]=le_short(s[i]);
         else
            s[i]=be_short(s[i]);
      }
   }

   /* FIXME: This is probably redundent now */
   /* copy to float input buffer */
   for (i=0;i<frame_size*channels;i++)
   {
      input[i]=(short)s[i];
   }

   for (i=nb_read*channels;i<frame_size*channels;i++)
   {
      input[i]=0;
   }


   return nb_read;
}



int main(int argc,char *argv[])
{
    FILE *infile,*outfile;
    char *infilename,*outfilename;
    short inshort[2000];
    char outdata[2000];
    int fileheaderlength;
    int dataheaderlength;
    struct timeval tv;

    unsigned int ms,oldms,usems;
    SpeexBits speexbits;
    void *st;
    spx_int32_t rate=0;
    int chan=1;
    int fmt=16;
    spx_int32_t size=0;
    char first_bytes[12]={0};
    spx_int32_t frame_size;
    spx_int32_t quality=-1;
    int nb_samples, total_samples=0, nb_encoded;

    strcpy(sfh.name,"brxy1");
    sfh.channel=0;
    sdh.nbytes=0;
    sdh.channel=0;
    if(argc!=3)
    {
        fprintf(stdout,"ERROR:  paramiters number argc!=3\n");
        return 0;
    }
    infilename=argv[1];
    outfilename=argv[2];
    infile=fopen(infilename,"rb");
    if(infile==NULL)
    {
        printf("ERROR:  %s is not exist\n",infilename);
        return 0;
    }
    outfile=fopen(outfilename,"w");
    if(outfile==NULL)
    {
        printf("ERROR:  %s is not exist\n",outfilename);
        fclose(infile);
        return 0;
    }
    fread(first_bytes, 1, 12, infile);
    if(strncmp(first_bytes,"RIFF",4)==0 && strncmp(first_bytes,"RIFF",4)==0)
    {
        if(read_wav_header(infile, &rate, &chan, &fmt, &size)==-1)
        {
            printf("ERROR:  read_wav_header fail\n");
            goto errr_1;
        }
        /* CHECK: exists big-endian .wav ?? */
    }
    st = speex_encoder_init(speex_lib_get_mode(SPEEX_MODEID_WB));
    speex_encoder_ctl(st, SPEEX_GET_FRAME_SIZE, &frame_size);
    speex_encoder_ctl(st, SPEEX_SET_SAMPLING_RATE, &rate);
    quality=10;
    speex_encoder_ctl(st, SPEEX_SET_QUALITY, &quality);
    speex_bits_init(&speexbits);
    nb_samples = read_samples(infile,frame_size,fmt,chan,1,inshort, NULL, &size);
    printf("frame_size=%d    fmt=%d    chan=%d    size=%d\n",frame_size,fmt,chan,size);
    sfh.channel=chan;
    sdh.channel=chan;
    fwrite((void *)&sfh,sizeof(speexfileheader),1,outfile);
    fileheaderlength=sizeof(speexfileheader);

    printf("fileheader  length = %d\n",fileheaderlength);
    dataheaderlength=sizeof(speexdataheader);
    printf("dataheader  length = %d\n",dataheaderlength);
    while(1)
    {
        gettimeofday(&tv, NULL);
        oldms=(tv.tv_sec*1000)+(tv.tv_usec/1000);
        if(chan==2)
        {
            speex_encode_stereo_int(inshort, frame_size, &speexbits);
        }
        speex_encode_int(st, inshort, &speexbits);

        nb_encoded += frame_size;

        speex_bits_insert_terminator(&speexbits);

        sdh.nbytes = speex_bits_write(&speexbits,outdata,MAX_FRAME_BYTES);
        fwrite((void *)&sdh,sizeof(speexdataheader),1,outfile);


        fwrite(outdata,1,sdh.nbytes,outfile);
        speex_bits_reset(&speexbits);

        nb_samples = read_samples(infile,frame_size,fmt,chan,1,inshort, NULL, &size);

        if (nb_samples==0)
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
errr_1:
    fclose(infile);
    fclose(outfile);
    return 1;
}









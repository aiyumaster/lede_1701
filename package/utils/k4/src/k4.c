


#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include "./json-c/json.h"//json库头文件
#include <uuid/uuid.h>//生成UUID相关头文件
//#include<sqlite3.h>//sqlite3库文件
#include <netinet/in.h>   
#include <arpa/inet.h>   
#include <linux/sockios.h>
//websocket 头文件
//#include <nopoll.h>
//#include <nopoll_private.h>

//串口相关头文件 
#include <sys/types.h>  
#include <sys/stat.h>  
#include <fcntl.h> //文件控制定义  
#include <termios.h>//终端控制定义  
#include <errno.h>  
#include <sys/signal.h>  
#include <pthread.h>
//时间相关函数头文件
#include <sys/time.h>
#include <time.h>
//deviceid相关头文件
#include <errno.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>        //for struct ifreq
#include <uci.h>
#include <curl/curl.h>
#include <sys/time.h>
#include <linux/i2c.h> 
#include <linux/i2c-dev.h>
#include <alsa/asoundlib.h>
#include <linux/input.h>
#include <semaphore.h>
#include <nopoll/nopoll.h>
#include <nopoll/nopoll_private.h>
#include <mad.h>

#include <speex/speex_header.h>
#include <speex/speex_stereo.h>
#include <speex/speex.h>
#include <linux/soundcard.h>




#include "TXSDKCommonDef.h"
#include "TXDeviceSDK.h"
#include "TXOTA.h"
#include "TXFileTransfer.h"


#include "TXDeviceSDK.h"

#include "SSLKernelItem.h"




/* cJSON */
/* JSON parser in C. */
#include <math.h>
#include <float.h>
#include <limits.h>
#include <ctype.h>
#include "cJSON.h"
/*********MD5 Checksum**********/
//#include "md5Checksum.h"

//#include "hashmd5.h"
//#include "SSLKernelItem.h"


typedef int nopoll_bool;


nopoll_bool debug = nopoll_false;




int gettimeofday(struct timeval *tv, struct timezone *tz);





/****************************************************************************
RC522全局变量



******************************************************************************/

/////////////////////////////////////////////////////////////////////
//MF522命令字
/////////////////////////////////////////////////////////////////////
#define PCD_IDLE              0x00               //取消当前命令
#define PCD_AUTHENT           0x0E               //验证密钥
#define PCD_RECEIVE           0x08               //接收数据
#define PCD_TRANSMIT          0x04               //发送数据
#define PCD_TRANSCEIVE        0x0C               //发送并接收数据
#define PCD_RESETPHASE        0x0F               //复位
#define PCD_CALCCRC           0x03               //CRC计算

/////////////////////////////////////////////////////////////////////
//Mifare_One卡片命令字
/////////////////////////////////////////////////////////////////////
#define PICC_REQIDL           0x26               //寻天线区内未进入休眠状态
#define PICC_REQALL           0x52               //寻天线区内全部卡
#define PICC_ANTICOLL1        0x93               //防冲撞
#define PICC_ANTICOLL2        0x95               //防冲撞
#define PICC_AUTHENT1A        0x60               //验证A密钥
#define PICC_AUTHENT1B        0x61               //验证B密钥
#define PICC_READ             0x30               //读块
#define PICC_WRITE            0xA0               //写块
#define PICC_DECREMENT        0xC0               //扣款
#define PICC_INCREMENT        0xC1               //充值
#define PICC_RESTORE          0xC2               //调块数据到缓冲区
#define PICC_TRANSFER         0xB0               //保存缓冲区中数据
#define PICC_HALT             0x50               //休眠

/////////////////////////////////////////////////////////////////////
//MF522 FIFO长度定义
/////////////////////////////////////////////////////////////////////
#define DEF_FIFO_LENGTH       64                 //FIFO size=64byte
#define MAXRLEN  18

/////////////////////////////////////////////////////////////////////
//MF522寄存器定义
/////////////////////////////////////////////////////////////////////
// PAGE 0
#define     RFU00                 0x00    
#define     CommandReg            0x01    
#define     ComIEnReg             0x02    
#define     DivlEnReg             0x03    
#define     ComIrqReg             0x04    
#define     DivIrqReg             0x05
#define     ErrorReg              0x06    
#define     Status1Reg            0x07    
#define     Status2Reg            0x08    
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     RFU0F                 0x0F
// PAGE 1     
#define     RFU10                 0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     RFU1A                 0x1A
#define     RFU1B                 0x1B
#define     MifareReg             0x1C
#define     RFU1D                 0x1D
#define     RFU1E                 0x1E
#define     SerialSpeedReg        0x1F
// PAGE 2    
#define     RFU20                 0x20  
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     RFU23                 0x23
#define     ModWidthReg           0x24
#define     RFU25                 0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsCfgReg            0x28
#define     ModGsCfgReg           0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
// PAGE 3      
#define     RFU30                 0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39  
#define     TestDAC2Reg           0x3A   
#define     TestADCReg            0x3B   
#define     RFU3C                 0x3C   
#define     RFU3D                 0x3D   
#define     RFU3E                 0x3E   
#define     RFU3F		  		  0x3F

/////////////////////////////////////////////////////////////////////
//和MF522通讯时返回的错误代码
/////////////////////////////////////////////////////////////////////
//#define 	MI_OK                 0x26
//#define 	MI_NOTAGERR           0xcc
//#define 	MI_ERR                0xbb
#define MI_OK                     0
#define MI_NOTAGERR               1//(-1)
#define MI_ERR                    2//(-2)


unsigned char aIDType[2];
unsigned char rfidnum[6]={0x04,0x00,0xa5,0x04,0x5e,0x74};
unsigned char rfbuf[16]={0};
unsigned char rfcnt = 0;
unsigned char DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
unsigned char MLastSelectedSnr[4];
unsigned char RevBuffer[30]; 
unsigned int RFIDSection;
unsigned char readbuff[11];
unsigned char get_from_rfid[10];







/****************************************************************************
定义基本数据数据结构体



******************************************************************************/
struct k4_base_info{
	char ip[36];
	char name[128];
	char id[36];
	unsigned char qqlicense[1024];
	int qq_init_success;
	char sn[17];
	char sn_mcu[40];
	char mediaa[36];
	char mediab[36];
	char *version;
	int poe;//关机是否关一体机电源，0:关  1:不关
	int openlimit;
	int pc_automatic;
	sem_t qq_init_sem_t;
	int qq_init_flag;
	int qq_lisence_get_from_mcu_is_ok;
	int have_new_lisence;
	pthread_t qq_pthread;
	
};
struct k4_devices_status{
	int k4;
	int pc;
	int projector;
	int ep;//ep通道显示
	int stand_lock;//展台锁 0没有锁 1锁着的

	int projecter_power;
	int projecter_channel;

	

	int machine_flag;//是否正在开、关机中
	int projector_flag;//是否正在开、关投影仪
	int pc_flag;//是否在开pc；
};
struct k4_run_info{
	int temperature;//温度
	int illumination;//光照强度
	int humidity;//湿度
};
struct k4_projector_info
{
	int baudrate;
	int databyte;
	char check;//N:无校验  O:奇校验  E:偶校验
	int stop;
	
	char start_code[512];
	char shutdown_code[512];
	char show_channel[512];

	unsigned char get_qq_license[525];
	int get_qq_license_return_flag;
	int get_qq_license_busy;
	int get_qq_sn_return_flag;
	int get_qq_sn_busy;

	int start_delay;
	int shutdown_delay;
	int interval;

	int start_wait;//开机标志
	int shutdown_wait;//关机标志
	int interval_wait;//间隔时间
	pthread_t serial_read_pthread;
	pthread_mutex_t mutex;//互斥锁
	pthread_mutexattr_t attr;
	int send_return_status;
	int serial_fd;
};
struct k4_volume{
	int high;
	int low;
	unsigned int volume;
	int volume_default;
	int mute;
};
struct local_play_thread{
	int file_serial;
	int last;
	int del;
	pthread_t threak_id;
	pthread_attr_t attr;
	struct local_play_thread *next;
};
struct local_play_thread_header{
	int number;
	pthread_mutex_t mutex;//互斥锁
	pthread_mutexattr_t attr;
	pthread_t play_thread_free;
	struct local_play_thread *next;
};
struct k4_rfid_card_id{
	int last;
	int del;
	unsigned char id[5];
	unsigned char id1[9];
	char name[128];
	struct k4_rfid_card_id *next;
};
struct k4_rfid_info{
	unsigned char projector[10];
	unsigned int section;
	pthread_t play_thread_free;
	pthread_t rfid_close_thread;
	int rfid_flag;
	unsigned int opentime;
	int workmode;
	int flag;//倒计时标志
	int use_time;//剩余时间
	int use_mode;//是否要到计时
	int read_card_id_return;
	unsigned char rfid_card_id[4];
	unsigned char rfid_card_projector_id_read[16];
	unsigned char rfid_card_projector_id[10];
	unsigned char sector;
	unsigned char key_a[6];
	unsigned char control_data[16];
	int number_local;
	int number_net;
	struct k4_rfid_card_id *next_local;
	struct k4_rfid_card_id *next_net;
};

struct k4_tca_info{
	unsigned char port0_in;
	unsigned char port1_in;
	unsigned char port0_out;
	unsigned char port1_out;
	unsigned char port0_polarity;
	unsigned char port1_polarity;
	unsigned char port0_configuration;
	unsigned char port1_configuration;

	unsigned int mation_out;
	unsigned int pc_out;
	unsigned int mation_status;
	unsigned int projector_status;
	unsigned int ep_status;
	unsigned int projector_out;
	unsigned int lock_enable;
	unsigned int lock_out;
	
};
struct k4_luci_socket_info{
	struct sockaddr_in my_addr;
	struct sockaddr_in remote_addr;
	int client_sockfd;
	int server_sockfd;
	pthread_t luci_socket_thread;
};
struct k4_key_button_info{
	sem_t wake_key_deal;
	pthread_t key_read_thread;
	pthread_t key_deal_thread;
	int key_deal_flag;//0没处理，1在处理
	unsigned int code;
	unsigned int value;
	int fd;

	int count;
	
	struct input_event event;
};

struct k4_led_glint{
	sem_t led_standy_sem_t;
	sem_t led_start_sem_t;
	pthread_t led_standy_pthread;
	pthread_t led_start_pthread;
	int led_standy_flag;
	int led_start_flag;
};

struct k4_nopoll_info{
	noPollCtx  *ctx;
	noPollConn *conn;
	int request_rfidinfo;
	int nopoll_create_ctx_first;

	noPollConnOpts *opts;
	int nopoll_isok_flag;
	int connect_report_flag;
	int nopoll_create_first;
	int count_of_failure;
	pthread_mutex_t mutex;//互斥锁
	pthread_mutexattr_t attr;

	pthread_t creat_connection_thread;
	pthread_t report_server_thread;
};

struct k4_schema_play_info{
	unsigned char recivebuf[81920];//存储区域
	unsigned char libmad_mp3_buf[12288];
	unsigned char get_ptr[12800];
	unsigned char *write_pointer;//写指针
	unsigned char *recive_pointer;//接受指针
	unsigned int recive_size;//接收到的大小，解码后减少，接收到后增加

	int recivesizebigerr;
	int sync;//两个线程同步
	int stream_over_flag;//告诉解码线程已经退出并重新初始化
	int stream_over_flag_re;//判断是否解码了返回
	int first_decode;//初始化后还没有解码
	int first_save;//初始化后第一次解码
	int recive_buf_not_enough;//接收到的数据不够，不能进行播放则为1
	char ip[36];

	int is_working;
	int first_run;
	sem_t wake_write;
	sem_t schedule_play_wait_server;
	pthread_t recive_pthread;
	pthread_t write_pthread;

	struct snd_pcm_t *handle;


	CURL *curl;
	CURLcode res;
	char libcurl_url[256];
	struct mad_stream    stream;
	struct mad_frame    frame;
	struct mad_synth    synth;
	struct mad_header   header;

	int sample;
	int frame_period_second;
	int byte_period_second;
	
	struct snd_pcm_hw_params_t *params;
	
};

struct k4_broadcast_play_mp3_info{
	unsigned char recivebuf[81920];//存储区域
	unsigned char libmad_mp3_buf[12288];
	unsigned char get_ptr[12800];
	unsigned char *write_pointer;//写指针
	unsigned char *recive_pointer;//接受指针
	unsigned int recive_size;//接收到的大小，解码后减少，接收到后增加

	int recivesizebigerr;
	int sync;//两个线程同步
	int stream_over_flag;//告诉解码线程已经退出并重新初始化
	int stream_over_flag_re;//判断是否解码了返回
	int first_decode;//初始化后还没有解码
	int first_save;//初始化后第一次解码
	int recive_buf_not_enough;//接收到的数据不够，不能进行播放则为1
	char ip[36];

	int is_working;
	int first_run;
	sem_t wake_write;
	sem_t broadcast_play_mp3_wait_server;
	pthread_t recive_pthread;
	pthread_t write_pthread;

	struct snd_pcm_t *handle;


	CURL *curl;
	CURLcode res;
	char libcurl_url[256];
	struct mad_stream    stream;
	struct mad_frame    frame;
	struct mad_synth    synth;
	struct mad_header   header;

	int sample;
	int frame_period_second;
	int byte_period_second;
	
	struct snd_pcm_hw_params_t *params;
	
};

struct k4_pc_det_struct{
	int fd;
	pthread_t pc_detect_thread;
};
struct k4_mcu_i2c_info{
	unsigned int key_control_register;
	unsigned int block_read_write_register;
	unsigned int status_register;
	unsigned int version_register;
	unsigned int block_operation_register;//0xaa 写    0xbb+1-7 读具体的块     一共八块


	unsigned int x86open_off;
	unsigned int lock_out;
	unsigned int lock_enable;
	unsigned int ope_power;
	unsigned int x84_power;//x86等设备供电
	unsigned int projector_power;//投影仪供电
	unsigned int ep_power;//ep92的3.3v供电
	unsigned int buzzer;//蜂鸣器
	unsigned int reset_ep92;//ep92reset
	unsigned int chanel_status;
	unsigned int projector_status;
	unsigned int system_status;
	unsigned int lock_status;
	unsigned int system_status_init;


	unsigned int x86open_off_config;
	unsigned int lock_out_config;
	unsigned int lock_enable_config;
	unsigned int ope_power_config;
	unsigned int x84_power_config;//x86等设备供电
	unsigned int projector_power_config;//投影仪供电
	unsigned int ep_power_config;//ep92的3.3v供电
	unsigned int buzzer_config;//蜂鸣器
	unsigned int reset_ep92_config;//ep92reset
	unsigned int chanel_status_config;
	unsigned int projector_status_config;
	unsigned int system_status_config;
	unsigned int lock_status_config;
	unsigned int system_status_init_config;



	
};
struct k4_power_control_status{
	int schedule_broadcast;
	int microphone_broadcast;
	int system_status;
	pthread_mutex_t mutex;//互斥锁
	pthread_mutexattr_t attr;
};
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



struct speex_basedata_struct{
	char playbuf[2000];
	char inbuf[2000];
	char playbackbuf[2000];
	int inflag;
	pthread_mutex_t mutex;//互斥锁
	int port;
	pthread_t recivethread;
    pthread_t playthread;
	pthread_t sendthread;
	int fd;
    int rate;
    int format;
    int channel;
	
	int dtsclient;
    struct sockaddr_in serveraddr;
    int sockaddrlength;
	int nbytes;

	speexdataheader sdh;
};
struct speex_basedata_struct speex_basedata;




char *qqkeydata = "-----BEGIN EC PARAMETERS-----\nBgUrgQQACg==\n-----END EC PARAMETERS-----\n-----BEGIN EC PRIVATE KEY-----\nMHQCAQEEIC9glMhvxOOa9NpXMdx9ZLuHRNnb0KwOS1S0FvEZTSYmoAcGBSuBBAAK\noUQDQgAEyOU/YwYNR2o/v78x95hGW/zI8iaF3RPWaHH2li5YCi4UtTpKGEf5lxDb\nXXYoN/b4jRjUJ3w89kXDBttnLhfo9Q==\n-----END EC PRIVATE KEY-----\n";
/****************************************************************************
定义全局变量



******************************************************************************/

#define uart "/dev/ttyS0"
#define brxy "/usr/share/brxydata/brxy"

pthread_mutex_t i2c_mutex;//互斥锁
pthread_mutexattr_t i2c_attr;

pthread_mutex_t action_mutex;//互斥锁
pthread_mutexattr_t action_attr;

pthread_mutex_t key__mutex;//互斥锁
pthread_mutexattr_t key__attr;



pthread_mutex_t rfid_card_uci_mutex;//互斥锁
pthread_mutexattr_t rfid_card_uci_attr;



struct k4_base_info base_info;
struct k4_devices_status device_status={
	.machine_flag=0,
	.projector_flag=0,
	.pc_flag=0,
};
struct k4_projector_info projector_info;
struct k4_run_info run_info;
struct k4_volume volume_3105;
struct local_play_thread_header local_play_threadheader={
	.number=0,
};
struct k4_rfid_info rfid_info={
	.rfid_flag=0,
	.flag=2,
};
struct k4_tca_info tca_info={
	.port0_in=0x00,
	.port1_in=0x01,
	.port0_out=0x02,
	.port1_out=0x03,
	.port0_polarity=0x04,
	.port1_polarity=0x05,
	.port0_configuration=0x06,
	.port1_configuration=0x07,

	.mation_out=1,
	.pc_out=0,//x86开关机
	.mation_status=3,
	.projector_status=4,
	.ep_status=5,
	.projector_out=2,//X86电源
	.lock_enable=7,
	.lock_out=6,
};
struct k4_luci_socket_info luci_socket_info;
struct k4_key_button_info key_button_info={
	.key_deal_flag=1,
	.code=0,
	.value=0,
	.count=0,
};
struct k4_led_glint led_glint={
	.led_standy_flag=0,
	.led_start_flag=0,
};
struct k4_nopoll_info nopoll_info;
struct k4_schema_play_info schema_play_info;
struct k4_broadcast_play_mp3_info broadcast_play_mp3_info;
struct k4_pc_det_struct pc_det;
struct k4_mcu_i2c_info mcu_i2c_info={
	.key_control_register=0x20,
	.block_read_write_register=0x32,
	.status_register=0x2f,
	.version_register=0x30,
	.block_operation_register=0x21,
	.x86open_off=0x10,
	.lock_out=0x12,
	.lock_enable=0x13,
	.ope_power=0x17,
	.x84_power=0x11,
	.projector_power=0x1e,
	.ep_power=0x1f,
	.buzzer=0x1c,
	.reset_ep92=0x1d,
	.chanel_status=0x14,
	.projector_status=0x15,
	.system_status=0x16,
	.lock_status=0x1a,
	.system_status_init=0x1b,
	.x86open_off_config=0x00,
	.lock_out_config=0x02,
	.lock_enable_config=0x03,
	.ope_power_config=0x07,
	.x84_power_config=0x01,
	.projector_power_config=0x0e,
	.ep_power_config=0x0f,
	.buzzer_config=0x0c,
	.reset_ep92_config=0x0d,
	.chanel_status_config=0x04,
	.projector_status_config=0x05,
	.system_status_config=0x06,
	.lock_status_config=0x0a,
	.system_status_init=0x0b,
};
struct k4_power_control_status power_control_status;

















/************************************************************************************
下面是  wifi物联网全局变量




		
************************************************************************************/




//switch制作json结构体是需要
struct switch_json{
	json_object *switch_switch;
	json_object *switch_array;
	json_object **lamps;
	int lamp_number;
	int last;
	struct switch_json *next;
};

struct switch_json_header{
	int number;
	struct switch_json *next;
};


//**************************************************************wifi设备mac地址结构体

struct wifi_internetthings_mac
{
	char mac[24];
	char name[104];
	int del;//是否已经删除1；删除 0 未删除
	int status;//为1则不再加网，为0则继续加网，查询设备超时后复位
	int aready_add;//是否已经加过
	int last;//是否是最后一个，对mac意义不大
	int type;//所加设备类型  1为wifi_switchheader，2为wifi_sensorheader，3为wifi_screenheader 4为wifi_rfidheader，5为wifi_macheader
	void *thing;//所加设备结构体指针，方便直接控制设备，
	struct wifi_internetthings_mac *next;//下一个mac结构体
};

struct wifi_mac_header
{
	int number;
	struct wifi_internetthings_mac *next;
};

struct wifi_mac_header wifi_macheader;

//*******************************************************************************switch
struct wifi_switch
{
	char name[104];
	char mac[24];
	char ip[24];
	int del;
	int name_number;
	int status;				//状态
	int lamp_number;
	int lamp[10];			//灯泡i是否亮
	int last;				//是否还有下一个
	struct wifi_internetthings_mac *mac_struct;
	struct wifi_switch *next;
};

struct wifi_switch_header
{
	int number;
	struct wifi_switch *next;
};
//*******************************************************************************sensor

struct wifi_sensor
{
	char name[104];
	char mac[24];
	char ip[24];
	int del;
	int name_number;
	int status;
	int temperature;//温度
	int illumination;//光强
	int humidity;//湿度
	int last;
	struct wifi_internetthings_mac *mac_struct;
	struct wifi_sensor *next;
};

struct wifi_sensor_header
{
	int number;
	struct wifi_sensor *next;
};
//**********************************************************************************screen

struct wifi_screen
{
	char name[104];
	char mac[24];
	char ip[24];
	int del;
	int name_number;
	int status;
	int last;
	struct wifi_internetthings_mac *mac_struct;
	struct wifi_screen *next;
};

struct wifi_screen_header
{
	int number;
	struct wifi_screen *next;
};
//*********************************************************************************rfid

struct wifi_rfid
{
	char name[104];
	char mac[24];
	char ip[24];
	int del;
	int name_number;
	int status;
	int last;
	struct wifi_internetthings_mac *mac_struct;
	struct wifi_rfid *next;
};

struct wifi_rfid_header
{
	int number;
	struct wifi_rfid *next;
};


//******************************************************实例化
struct wifi_switch_header wifi_switchheader;
struct wifi_sensor_header wifi_sensorheader;
struct wifi_screen_header wifi_screenheader;
struct wifi_rfid_header wifi_rfidheader;






//*******************************************************物联网curl、线程、广播全局变量
int global_libcurl_init_return=0;
char libcurl_url[256];
char *libcurl_select_header="http://";
char *libcurl_select_end="/config?command=Device";
char libcurl_select_recive[512];

pthread_t add_wifi_things_thread;
pthread_t select_wifi_things_thread;
pthread_t recive_rfid_data_thread;

struct sockaddr_in wifi_client,wifi_serve,init_wifi_socket;
int wifi_sock_fd;
int broadcast_select_turn=0;
int delete_ito_success_flag=0;//删除物联网成功标志位
int add_ito_success_flag=0;//物联网添加成功标志位



/************************************************************************************
上面是  wifi物联网全局变量

	
************************************************************************************/

/************************************************************************************
以下是i2c控制全局变量

************************************************************************************/
int i2c_fd;
struct i2c_rdwr_ioctl_data i2c_write_ioctl_data;
struct i2c_rdwr_ioctl_data i2c_read_ioctl_data;


unsigned char i2c_address_3105=0x18;
unsigned char i2c_address_ep=0x64;
unsigned char i2c_address_rfid=0x38;
unsigned char i2c_address_tca=0x20;
unsigned char i2c_address_wm=0x1a;
unsigned char i2c_address_tas=0x1d;
unsigned char i2c_address_mcu=0x6e;

//加入WM8776-TAS5707-芯控制全局变量
unsigned char wm8776_volume_table[9]={0x00,0x9f,0xab,0xb7,0xc3,0xcf,0xdb,0xe7,0xf3};

//////3104全局变量
const unsigned char AIC3105Bass[] = 
{

  0x7D,0x86,0x84,0xEF,0x78,0xB4,0x7A,0xFF,0x89,0xA0,
  0x7F,0x28,0x82,0x0B,0x7C,0xC9,0x7D,0xF3,0x84,0x09,
  0x7F,0x98,0x81,0x01,0x7E,0x68,0x7E,0xFF,0x81,0xFD,
  0x7F,0xC0,0x81,0x6F,0x7D,0x68,0x7E,0x90,0x82,0xD6,
  0x7F,0x92,0x82,0xCF,0x7A,0xE6,0x7D,0x2E,0x85,0x83,      // bass 5
  0x7F,0xFF,0x82,0xA4,0x7A,0xD5,0x7D,0x5E,0x85,0x27,
  0x7F,0xFF,0x80,0xB6,0x7E,0x98,0x7F,0x67,0x81,0x2F,
  0x7F,0xFF,0x81,0x6E,0x7D,0x2E,0x7E,0xCD,0x82,0x5E,
  0x7F,0xFF,0x82,0xD4,0x7A,0x78,0x7D,0x9D,0x84,0xAD,
  0x7F,0xFF,0x80,0xCA,0x7E,0x71,0x7F,0x71,0x81,0x1C,      // bass 10
  0x7F,0xFF,0x81,0x89,0x7C,0xF9,0x7E,0xE9,0x82,0x27,
  0x7F,0xFF,0x83,0x15,0x79,0xFD,0x7D,0xD1,0x84,0x49,
  0x7F,0xFF,0x81,0x04,0x7D,0xFE,0x7F,0x65,0x81,0x33,
  0x7F,0xFF,0x83,0x1F,0x79,0xE9,0x7E,0x27,0x83,0xA2,
  0x7F,0xFF,0x82,0x7E,0x7B,0x1D,0x7E,0xBF,0x82,0x7A
  
};

const unsigned char AIC3105Treble[] = 
{
  0x21,0xDB,0xE1,0xA6,0x1B,0x83,0x79,0x55,0x8C,0xAB,
  0x25,0x88,0xE5,0xF2,0x14,0x1C,0x6B,0xDA,0xA2,0xC4,
  0x32,0x95,0xD7,0xFB,0x21,0x29,0x6F,0xB2,0x9C,0xE6,
  0x4A,0x21,0xC7,0x06,0x2E,0x64,0x69,0xED,0xA5,0x95,
  0x38,0x83,0xE4,0xC3,0x13,0x44,0x56,0x22,0xBE,0x6D,    //Treble 5
  0x3C,0xE2,0x06,0x64,0x0A,0xEB,0x24,0x60,0xE2,0xA8,
  0x59,0x78,0xF8,0xFA,0x0F,0xBD,0x20,0x4D,0xE4,0x3B,
  0x7F,0xFF,0x20,0x47,0x1B,0xC3,0xDE,0xFE,0xE3,0xF9,
  0x7F,0xFF,0x24,0x31,0x1D,0x45,0xCA,0x6E,0xD9,0xD2,
  0x7F,0xFF,0xE4,0x7E,0x1A,0x2A,0xFF,0x7B,0xEA,0x0A,    // Treble10
  0x7F,0xFF,0xB8,0x95,0x33,0x55,0x2E,0xF1,0xDD,0xA8,
  0x7F,0xFF,0xAC,0xEE,0x3E,0x56,0x48,0x87,0xCB,0xB6,
  0x7F,0xFF,0x95,0xFA,0x5A,0x90,0x62,0xB6,0xAF,0x79,
  0x7F,0xFF,0x9E,0xE7,0x4E,0x73,0x54,0x19,0xC0,0x99,
  0x7F,0xFF,0x90,0x95,0x62,0xAA,0x64,0x8A,0xAD,0x0F
};

const unsigned char LRB1Addr[] =
{
  1,2,3,4,5,6,13,14,15,16
};  

const unsigned char LRB2Addr[] =
{
  7,8,9,10,11,12,17,18,19,20
};  











//int gettimeofday(struct timeval *tv, struct timezone *tz);

/************************************************************************************
函数申明

	
************************************************************************************/

/**********************************************************************************

wifi物联网函数集合声明
**********************************************************************************/
static int get_internetthings_mac_from_uci(void);//从brxy中获取mac并加入链表中
static void init_wifi_internettings_header(void);//初始化header，number赋值为零
static void add_to_header(void *data,int type);//加结构体到header
static void *add_internetthings_from_broadcast(void);//用广播加设备线程
static int broadcast_to_add(char *mac,char *lanip,struct wifi_internetthings_mac *mac_struct);//发出广播函数
static int get_lan_ip_to_broadcast(char *ip);//从network中获取lan口ip
static void deal_with_recive_from_broadcast(char * rebuf,char *oldmac,struct wifi_internetthings_mac *mac_struct);//处理广播后返回的数据

static void libcurl_global_init(void);//libcurl库初始化
static void *select_wifi_internetthings_status(void);//查询线程
static void select_wifi_internetthings_fuction(void);//查询总体功能函数
static void libcurl_select_rfid(struct wifi_rfid *sw);//用curl查询rfid
static void libcurl_select_screen(struct wifi_screen *sw);//用curl查询screen
static void libcurl_select_sensor(struct wifi_sensor *sw);//用curl查询sensor
static void libcurl_select_switch(struct wifi_switch *sw);//用curl查询switch
static size_t libcurl_write_callback(void *ptr, size_t size, size_t nmemb, void *stream);//curl存储回调函数


static void turn_on_off_wifi_switch(int st);//开关灯
static void delet_wifi_things(char *thing);//删除wifi物联网设备
static int add_wifi_things(char *thing);//添加wifi物联网设备
static void init_mutex(void);
static void turn_on_off_screen(int st);

//static int get_sqlite3_RFID_ADMIN(char *cardId);
static void wifi_rfid_data_turn_on(char *data);
static void *recive_wifi_rfid(void);
static void init_broadcast_socket(void);


/**********************************************************************************

qq调用函数申明
**********************************************************************************/

//ota升级相关
int cb_on_new_pkg_come(unsigned long long from, unsigned long long pkg_size, const char * title, const char * desc, unsigned int target_version);
void cb_on_download_progress(unsigned long long download_size, unsigned long long total_size);
void cb_on_download_complete(int ret_code);
void cb_on_update_confirm();
void on_receive_controll_dts(unsigned long long from_id, tx_data_point  datapoint);
void on_receive_query_all_status(unsigned long long from_id, tx_data_point	datapoint);

//数字教学一体机业务处理  处理设备关心的控制端指令
void my_on_receive_data_point(unsigned long long from_id, tx_data_point * data_points, int data_points_count);

//公共方法
bool readBufferFromFile(char *pPath, unsigned char *pBuffer, int nInSize, int *pSizeUsed);
/*************************************************************************************
以下是i2c函数申明

*************************************************************************************/

static int i2c_init(void);
static int i2c_write(unsigned char sla,unsigned int regi,unsigned int val);
static int i2c_read(unsigned char sla,unsigned int regi1,unsigned char *val);
//3104
static int TLV320_BASSWrite(int number);
static int TLV320_TreableWrite(int number);
void Tlv320_EnMute(void);
void Tlv320_DisMute(void);
void tlv320_reset(void);
void Init_tlv320(void);
void VOLUME_TLV320(int volume);
void MUTE_TLV320(int mute);
//ep
int ep_passage_onoff(int onoff);
int ep_passage_switch(int channel);
/*************************************************************************************
以下是RF522函数申明

*************************************************************************************/

void Delay_I_1us(uint16_t k);
void SPIWriteByte(unsigned char infor);
unsigned char SPIReadByte(void);
unsigned char ReadRawRC(unsigned char Address);
void WriteRawRC(unsigned char Address, unsigned char value);
void SetBitMask(unsigned char reg,unsigned char mask) ;
void ClearBitMask(unsigned char reg,unsigned char mask)  ;
void PcdAntennaOn(void);
void PcdAntennaOff(void);
void PcdReset(void);
void IC_CMT(unsigned char *UID,unsigned char *KEY,unsigned char RW,unsigned char *Dat);
void M500PcdConfigISOType(unsigned char type);
unsigned char PcdComMF522(unsigned char Command, 		
                 unsigned char *pInData, 		
                 unsigned char InLenByte,		
                 unsigned char *pOutData, 		
                 uint16_t  *pOutLenBit)	;
unsigned char PcdRequest(unsigned char req_code,unsigned char *pTagType);
unsigned char PcdAnticoll(unsigned char *pSnr);
void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData);
unsigned char PcdRead(unsigned char addr,unsigned char *pData);
unsigned char PcdSelect(unsigned char *pSnr);
unsigned char PcdHalt(void);
unsigned char PcdWrite(unsigned char addr,unsigned char *pData);
unsigned char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr);
/*************************************************************************************
以下是播放本地语音函数申明

*************************************************************************************/

void *local_play_thread_t_free(void *ptr);
void local_alsa_play_inaction(int arg);

int play_music(int file_seri);
void *local_alsa_play(void *arg);
int get_play_file_name(int file_serial,char *file_name);


/*************************************************************************************
以下是投影仪串口函数申明

*************************************************************************************/

static int init_serial(void);
static int set_serial(void);
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);


/*************************************************************************************
rfid数据判断函数

*************************************************************************************/

static unsigned char String2hex(char* str);
static int rfid_decision(char * data);
static char *strupr(char *str);


/*************************************************************************************
开机等处理函数，包括对tca芯片的控制

*************************************************************************************/
int tca_set_bit(int bit);
int clean_tca_bit(int bit);
int pc_close_close(void);
int pc_open_open(void);
int pc_open(void);
int pc_close(void);
int projector_open(void);
int projector_close(void);
int projector_open_open(void);
int projector_close_close(void);
int ep_change(void);
int ep_open(void);
int stand_open(void);
int stand_close(void);
int stand_open_open(void);
int stand_close_close(void);
int volume_add(void);
int volume_discrease(void);
int init_wm_tas_rfid_tca(void);
int open_machine(void);
int close_machine(void);

/*************************************************************************************
nopoll 相关函数

*************************************************************************************/


int report_fuction(void);
/*************************************************************************************
以下是与mcu传输数据函数声明

*************************************************************************************/

int read_lisence_from_mcu(unsigned char *getbuf);
int send_lisence_to_mcu(char *lisencedata);
int send_upgrade_mcu_file_to_mcu(char *path);
int get_lisence_from_mcu_to_check(void);

int report_fuction1(void);
int report_fuction2(void);


/*************************************************************************************
以下是qq license 验证用到的函数

*************************************************************************************/

static void md5_block(MD5_CTX *c, register uint32 *X, int num)
{
	register uint32 A,B,C,D;

	A=c->A;
	B=c->B;
	C=c->C;
	D=c->D;
	for (;;)
	{
		num-=64;
		if (num < 0) break;
		/* Round 0 */
		R0(A,B,C,D,X[ 0], 7,0xd76aa478L);
		R0(D,A,B,C,X[ 1],12,0xe8c7b756L);
		R0(C,D,A,B,X[ 2],17,0x242070dbL);
		R0(B,C,D,A,X[ 3],22,0xc1bdceeeL);
		R0(A,B,C,D,X[ 4], 7,0xf57c0fafL);
		R0(D,A,B,C,X[ 5],12,0x4787c62aL);
		R0(C,D,A,B,X[ 6],17,0xa8304613L);
		R0(B,C,D,A,X[ 7],22,0xfd469501L);
		R0(A,B,C,D,X[ 8], 7,0x698098d8L);
		R0(D,A,B,C,X[ 9],12,0x8b44f7afL);
		R0(C,D,A,B,X[10],17,0xffff5bb1L);
		R0(B,C,D,A,X[11],22,0x895cd7beL);
		R0(A,B,C,D,X[12], 7,0x6b901122L);
		R0(D,A,B,C,X[13],12,0xfd987193L);
		R0(C,D,A,B,X[14],17,0xa679438eL);
		R0(B,C,D,A,X[15],22,0x49b40821L);
		/* Round 1 */
		R1(A,B,C,D,X[ 1], 5,0xf61e2562L);
		R1(D,A,B,C,X[ 6], 9,0xc040b340L);
		R1(C,D,A,B,X[11],14,0x265e5a51L);
		R1(B,C,D,A,X[ 0],20,0xe9b6c7aaL);
		R1(A,B,C,D,X[ 5], 5,0xd62f105dL);
		R1(D,A,B,C,X[10], 9,0x02441453L);
		R1(C,D,A,B,X[15],14,0xd8a1e681L);
		R1(B,C,D,A,X[ 4],20,0xe7d3fbc8L);
		R1(A,B,C,D,X[ 9], 5,0x21e1cde6L);
		R1(D,A,B,C,X[14], 9,0xc33707d6L);
		R1(C,D,A,B,X[ 3],14,0xf4d50d87L);
		R1(B,C,D,A,X[ 8],20,0x455a14edL);
		R1(A,B,C,D,X[13], 5,0xa9e3e905L);
		R1(D,A,B,C,X[ 2], 9,0xfcefa3f8L);
		R1(C,D,A,B,X[ 7],14,0x676f02d9L);
		R1(B,C,D,A,X[12],20,0x8d2a4c8aL);
		/* Round 2 */
		R2(A,B,C,D,X[ 5], 4,0xfffa3942L);
		R2(D,A,B,C,X[ 8],11,0x8771f681L);
		R2(C,D,A,B,X[11],16,0x6d9d6122L);
		R2(B,C,D,A,X[14],23,0xfde5380cL);
		R2(A,B,C,D,X[ 1], 4,0xa4beea44L);
		R2(D,A,B,C,X[ 4],11,0x4bdecfa9L);
		R2(C,D,A,B,X[ 7],16,0xf6bb4b60L);
		R2(B,C,D,A,X[10],23,0xbebfbc70L);
		R2(A,B,C,D,X[13], 4,0x289b7ec6L);
		R2(D,A,B,C,X[ 0],11,0xeaa127faL);
		R2(C,D,A,B,X[ 3],16,0xd4ef3085L);
		R2(B,C,D,A,X[ 6],23,0x04881d05L);
		R2(A,B,C,D,X[ 9], 4,0xd9d4d039L);
		R2(D,A,B,C,X[12],11,0xe6db99e5L);
		R2(C,D,A,B,X[15],16,0x1fa27cf8L);
		R2(B,C,D,A,X[ 2],23,0xc4ac5665L);
		/* Round 3 */
		R3(A,B,C,D,X[ 0], 6,0xf4292244L);
		R3(D,A,B,C,X[ 7],10,0x432aff97L);
		R3(C,D,A,B,X[14],15,0xab9423a7L);
		R3(B,C,D,A,X[ 5],21,0xfc93a039L);
		R3(A,B,C,D,X[12], 6,0x655b59c3L);
		R3(D,A,B,C,X[ 3],10,0x8f0ccc92L);
		R3(C,D,A,B,X[10],15,0xffeff47dL);
		R3(B,C,D,A,X[ 1],21,0x85845dd1L);
		R3(A,B,C,D,X[ 8], 6,0x6fa87e4fL);
		R3(D,A,B,C,X[15],10,0xfe2ce6e0L);
		R3(C,D,A,B,X[ 6],15,0xa3014314L);
		R3(B,C,D,A,X[13],21,0x4e0811a1L);
		R3(A,B,C,D,X[ 4], 6,0xf7537e82L);
		R3(D,A,B,C,X[11],10,0xbd3af235L);
		R3(C,D,A,B,X[ 2],15,0x2ad7d2bbL);
		R3(B,C,D,A,X[ 9],21,0xeb86d391L);

		A+=c->A&0xffffffffL;
		B+=c->B&0xffffffffL;
		c->A=A;
		c->B=B;
		C+=c->C&0xffffffffL;
		D+=c->D&0xffffffffL;
		c->C=C;
		c->D=D;
		X+=16;
	}
}

void MD5_Init(MD5_CTX *c)
{
	memset(c,0,sizeof(MD5_CTX));
	c->A=INIT_DATA_A;
	c->B=INIT_DATA_B;
	c->C=INIT_DATA_C;
	c->D=INIT_DATA_D;
	c->Nl=0;
	c->Nh=0;
	c->num=0;
}

void MD5_Update(MD5_CTX *c,const register uint8 *data,uint32 len)
{
	register uint32 *p;
	int sw,sc;
	uint32 l;

	if (len == 0) return;

	l=(c->Nl+(len<<3))&0xffffffffL;
	/* 95-05-24 eay Fixed a bug with the overflow handling, thanks to
	* Wei Dai <weidai@eskimo.com> for pointing it out. */
	if (l < c->Nl) /* overflow */
		c->Nh++;
	c->Nh+=(len>>29);
	c->Nl=l;

	if (c->num != 0)
	{
		p=c->data;
		sw=c->num>>2;
		sc=c->num&0x03;

		if ((c->num+len) >= MD5_CBLOCK)
		{
			l= p[sw];
			p_c2l(data,l,sc);
			p[sw++]=l;
			for (; sw<MD5_LBLOCK; sw++)
			{
				c2l(data,l);
				p[sw]=l;
			}
			len-=(MD5_CBLOCK-c->num);

			md5_block(c,p,64);
			c->num=0;
			/* drop through and do the rest */
		}
		else
		{
			int ew,ec;

			c->num+=(int)len;
			if ((sc+len) < 4) /* ugly, add char's to a word */
			{
				l= p[sw];
				p_c2l_p(data,l,sc,len);
				p[sw]=l;
			}
			else
			{
				ew=(c->num>>2);
				ec=(c->num&0x03);
				l= p[sw];
				p_c2l(data,l,sc);
				p[sw++]=l;
				for (; sw < ew; sw++)
				{ c2l(data,l); p[sw]=l; }
				if (ec)
				{
					c2l_p(data,l,ec);
					p[sw]=l;
				}
			}
			return;
		}
	}

	/* we now can process the input data in blocks of MD5_CBLOCK
	* chars and save the leftovers to c->data. */
#ifdef L_ENDIAN
	//if ((uint32_PTR)data % sizeof(uint32) == 0)
	if ((uint32)(*(uint32*)&data) % sizeof(uint32) == 0)
	{
		sw=len/MD5_CBLOCK;
		if (sw > 0)
		{
			sw*=MD5_CBLOCK;
			md5_block(c,(uint32 *)data,sw);
			data+=sw;
			len-=sw;
		}
	}
#endif
	p=c->data;
	while (len >= MD5_CBLOCK)
	{
#if defined(L_ENDIAN) || defined(B_ENDIAN)
		if (p != (uint32 *)data)
			memcpy(p,data,MD5_CBLOCK);
		data+=MD5_CBLOCK;
#ifdef B_ENDIAN
		for (sw=(MD5_LBLOCK/4); sw; sw--)
		{
			Endian_Reverse32(p[0]);
			Endian_Reverse32(p[1]);
			Endian_Reverse32(p[2]);
			Endian_Reverse32(p[3]);
			p+=4;
		}
#endif
#else
		for (sw=(MD5_LBLOCK/4); sw; sw--)
		{
			c2l(data,l); *(p++)=l;
			c2l(data,l); *(p++)=l;
			c2l(data,l); *(p++)=l;
			c2l(data,l); *(p++)=l; 
		} 
#endif
		p=c->data;
		md5_block(c,p,64);
		len-=MD5_CBLOCK;
	}
	sc=(int)len;
	c->num=sc;
	if (sc)
	{
		sw=sc>>2;   /* words to copy */
#ifdef L_ENDIAN
		p[sw]=0;
		memcpy(p,data,sc);
#else
		sc&=0x03;
		for ( ; sw; sw--)
		{ c2l(data,l); *(p++)=l; }
		c2l_p(data,l,sc);
		*p=l;
#endif
	}
}

void MD5_Final(uint8 *md, MD5_CTX *c)
{
	register int i,j;
	register uint32 l;
	register uint32 *p;
	static uint8 end[4]={0x80,0x00,0x00,0x00};
	uint8 *cp=end;

	/* c->num should definitly have room for at least one more uint8. */
	p=c->data;
	j=c->num;
	i=j>>2;

	/* purify often complains about the following line as an
	* Uninitialized Memory Read.  While this can be true, the
	* following p_c2l macro will reset l when that case is true.
	* This is because j&0x03 contains the number of 'valid' uint8s
	* already in p[i].  If and only if j&0x03 == 0, the UMR will
	* occur but this is also the only time p_c2l will do
	* l= *(cp++) instead of l|= *(cp++)
	* Many thanks to Alex Tang <altitude@cic.net> for pickup this
	* 'potential bug' */
#ifdef PURIFY
	if ((j&0x03) == 0) p[i]=0;
#endif
	l=p[i];
	p_c2l(cp,l,j&0x03);
	p[i]=l;
	i++;
	/* i is the next 'undefined word' */
	if (c->num >= MD5_LAST_BLOCK)
	{
		for (; i<MD5_LBLOCK; i++)
			p[i]=0;
		md5_block(c,p,64);
		i=0;
	}
	for (; i<(MD5_LBLOCK-2); i++)
		p[i]=0;
	p[MD5_LBLOCK-2]=c->Nl;
	p[MD5_LBLOCK-1]=c->Nh;
	md5_block(c,p,64);
	cp=md;
	l=c->A; l2c(l,cp);
	l=c->B; l2c(l,cp);
	l=c->C; l2c(l,cp);
	l=c->D; l2c(l,cp);

	/* clear stuff, md5_block may be leaving some stuff on the stack
	* but I'm not worried :-) */
	c->num=0;
	/*  memset((char *)&c,0,sizeof(c));*/
}

void Md5HashBuffer( uint8 *outBuffer, const void *inBuffer, uint32 length)
{
	MD5_CTX md5InfoBuffer;

	MD5_Init( &md5InfoBuffer );
	MD5_Update( &md5InfoBuffer, (uint8*)inBuffer, length );
	MD5_Final( outBuffer, &md5InfoBuffer );
}










//用于读取指定文件，参数返回数据，内部申请内存外部释放
int ReadFileECC(const char *szFilePath, char **pDataBuffer, int *pFilelen) {
	int ret = 0;
	FILE *file = NULL;
	int nFilelen = 0;
	char *pDataFileBuffer = NULL;
	do {
		file = fopen(szFilePath, "rb");
		if (!file) {
			printf("fopen文件%s打开失败\n", szFilePath);
			ret = -1;
			break;
		}

		fseek(file, 0L, SEEK_END); /* 定位到文件末尾 */
		nFilelen = ftell(file);  // 计算文件长度
		pDataFileBuffer = (char*) malloc(nFilelen);
		if (pDataFileBuffer) {
			memset(pDataFileBuffer, 0, nFilelen);
			fseek(file, 0L, SEEK_SET); /* 定位到文件开头 */
			fread(pDataFileBuffer, nFilelen, 1, file);

			// 过滤回车换行
			int i = 0;
			int j = 0;
			for (; i < nFilelen; ++ i) {
				if (*(pDataFileBuffer + i) == '\n' || *(pDataFileBuffer + i) == '\r') {
					continue;
				}
				*(pDataFileBuffer + j) = *(pDataFileBuffer + i);
				++j;
			}
			if(j < i)
			{
				*(pDataFileBuffer + j) = '\0';
			}

			*pDataBuffer = pDataFileBuffer;
			*pFilelen = j;
			fclose(file);
			file = NULL;
			return 1;
		} else {
			printf("ReadFileECC申请内存失败\n");
			ret = -2;
			break;
		}
	} while (0);

	if (file) {
		fclose(file);
		file = NULL;
	}

	if (pDataFileBuffer) {
		free(pDataFileBuffer);
	}

	*pFilelen = 0;
	*pDataBuffer = NULL;
	return ret;
}

void ReadBufferHex(char * pcBufIn, int nLenBufIn, char * pcBufOut, int*pnLenBufOut)
{
	if (NULL == pcBufIn || NULL == pcBufOut || NULL == pnLenBufOut)
	{
		return;
	}

	int nLenBufOut = 0;
	int i = 0;
	for (i = 0; i < nLenBufIn; i += 2)
	{
		char cByte[4] = {0};
		sscanf(pcBufIn, "%2x", cByte);
#ifdef _SDK_BE_
		memcpy(pcBufOut, cByte+3, 1);
#else
		memcpy(pcBufOut, cByte, 1);
#endif

		pcBufIn  += 2;
		pcBufOut += 1;
		nLenBufOut +=1;
	}

	(*pnLenBufOut) = nLenBufOut;
}

int CreateECDSAKey(const char *pFilePath)
{
	int ret = 0;
	BIO 	*pBioKeyFile = NULL;
	EC_KEY  *ec_key = NULL;;   
	char    *pFileAllPath = NULL;
	do 
	{
		EC_GROUP *ec_group;  
		ec_key = EC_KEY_new();

		if (!ec_key)  
		{ 
			ret = -1;
			printf("Error：EC_KEY_new()\n");   
			break;  
		}   

		ec_group = EC_GROUP_new_by_curve_name(NID_secp256k1);
		if (!ec_group)
		{   
			ret = -2;
			printf("Error：CreateECDSAKey--EC_GROUP_new_by_curve_name()\n");  
			break;
		}
		
		EC_GROUP_set_asn1_flag(ec_group, OPENSSL_EC_NAMED_CURVE);
		EC_GROUP_set_point_conversion_form(ec_group, POINT_CONVERSION_UNCOMPRESSED);

		if(1 != EC_KEY_set_group(ec_key,ec_group))  
		{   
			ret = -3;
			printf("Error：CreateECDSAKey--EC_KEY_set_group()\n");  
			break;
		}
		
		if (!EC_KEY_generate_key(ec_key))
		{
			ret = -3;
			printf("Error：CreateECDSAKey--EC_KEY_generate_key()\n");
			break;
		}

		char *pPublicKey  = "public.pem";
		char *pPrivateKey = "ec_key.pem";
		//////////////////////////////////////////////////////////////////////////
		//保存公钥
		int nPathLen = strlen(pFilePath) + strlen(pPublicKey)+ strlen(pPrivateKey)+2;
		pFileAllPath = (char*)malloc(nPathLen);
		if (!pFileAllPath)
		{
			ret = -5;
			printf("CreateECDSAKey--申请内存失败\n");
			break;
		}

		//生成保存公钥的文件路径
		memset((void*)pFileAllPath,0,nPathLen);
		strcpy(pFileAllPath,pFilePath);
		strcat(pFileAllPath,"/");
		strcat(pFileAllPath,pPublicKey);

		pBioKeyFile = BIO_new_file(pFileAllPath, "wb");

		if (!pBioKeyFile)
		{
			ret = -6;
			printf("BIO创建文件%s失败\n",pFileAllPath);
			break;
		}
		
		if (1 != PEM_write_bio_EC_PUBKEY(pBioKeyFile, ec_key))
		{
			ret = -7;
			printf("PEM_write_bio_EC_PUBKEY写入key文件%s失败\n",pFileAllPath);
			break;
		}

		//////////////////////////////////////////////////////////////////////////
		//保存私钥
		//生成保存私钥的文件路径
		memset((void*)pFileAllPath,0,nPathLen);
		strcpy(pFileAllPath,pFilePath);
		strcat(pFileAllPath,"/");
		strcat(pFileAllPath,pPrivateKey);

		BIO_free(pBioKeyFile); 
		pBioKeyFile = BIO_new_file(pFileAllPath, "wb");
		if (!pBioKeyFile)
		{
			ret = -8;
			printf("BIO创建文件%s失败\n",pFileAllPath);
			break;
		}

		PEM_write_bio_ECPKParameters(pBioKeyFile, ec_group);
		if (1 != PEM_write_bio_ECPrivateKey(pBioKeyFile, ec_key,NULL,NULL,0,NULL,NULL))
		{
			ret = -9;
			printf("PEM_write_bio_EC_PUBKEY写入key文件%s失败\n",pFileAllPath);
			break;
		}

		ret = 1;
	} while (0);

	if (ec_key)
	{
		EC_KEY_free(ec_key);
		ec_key = NULL;
	}

	if (pFileAllPath)
	{
		free(pFileAllPath);
		pFileAllPath = NULL;
	}

	if (pBioKeyFile)
	{
		BIO_free(pBioKeyFile); 
		pBioKeyFile = NULL;
	}

	return ret;
}

int ECDSASignToBuffer(const char *szPrivateKeyPath,char *szBufferData,const int nBufferData,const char *szLicenceBuffer,unsigned int *pLicencelen)
{
	int ret = 0;
	EC_KEY *ec_key = NULL; 
	BIO    *pBioKeyFile = NULL;

	do 
	{
		if (!szBufferData || !szLicenceBuffer || !pLicencelen)
		{
			ret = 0;
			printf("参数错误\n");
			break;
		}

		pBioKeyFile = BIO_new_file(szPrivateKeyPath,"rb");
		ec_key = PEM_read_bio_ECPrivateKey(pBioKeyFile, NULL, NULL,NULL);

		if (!ec_key)
		{
			ret = -3;
			printf("从文件%s中读取密钥解密失败\n",szPrivateKeyPath);
			break;
		}

		unsigned char *signature = NULL;
		unsigned char digest[32] = {};
		unsigned int dgst_len = 0;

		EVP_MD_CTX md_ctx;
		EVP_MD_CTX_init(&md_ctx);
		EVP_DigestInit(&md_ctx,EVP_sha256());
		EVP_DigestUpdate(&md_ctx, (const void*)szBufferData,nBufferData);
		EVP_DigestFinal(&md_ctx, digest, &dgst_len);

		/* 数据签名 */   
		if (!ECDSA_sign(0,(const unsigned char *)digest, dgst_len,(unsigned char *)szLicenceBuffer,pLicencelen,ec_key)) 
		{
			ret = -4;
			printf("ECDSA_sign error\n");
			break;
		}

		ret = 1;
	} while (0);

	if (pBioKeyFile)
	{
		BIO_free(pBioKeyFile);
		pBioKeyFile = NULL;
	}

	if (ec_key)
	{
		EC_KEY_free(ec_key);
		ec_key = NULL;
	}
	
	return ret;
}

int ECDSAVerifyLicenceBuffer(const char *szPublicKeyPath,char *szBuffer,int nBufflen,char *szLicenceData,int nLicencelen)
{
	int ret;  
	EC_KEY *ec_key = NULL;  
	EC_GROUP *ec_group = NULL;   
	BIO *pBioKeyFile = NULL;
	do 
	{
		pBioKeyFile = BIO_new_file(szPublicKeyPath,"rb");
		ec_key = PEM_read_bio_EC_PUBKEY(pBioKeyFile, NULL, NULL,NULL);

		if (ec_key == NULL)
		{
			ret = -6;
			printf("Error：ECDSAVerifyLicenceBuffer PEM_read_bio_EC_PUBKEY\n");
			break;
		}

		unsigned char digest[32]={};  
		unsigned int  dgst_len = 0;   
		EVP_MD_CTX md_ctx;  
		EVP_MD_CTX_init(&md_ctx);  
		EVP_DigestInit(&md_ctx, EVP_sha256()); 
		// 散列算法  
		EVP_DigestUpdate(&md_ctx, (const void*)szBuffer,nBufflen);  
		EVP_DigestFinal(&md_ctx, digest, &dgst_len);

		/* 验证签名 */
		//nLicencelen++;
		ret = ECDSA_verify(0,(const unsigned char*)digest, dgst_len, (const unsigned char *)szLicenceData, nLicencelen,ec_key);
	} while (0);

	if (ec_key)
	{
		EC_KEY_free(ec_key); 
		ec_key = NULL;
	}

	if (pBioKeyFile)
	{
		BIO_free(pBioKeyFile);
		pBioKeyFile = NULL;
	}

	return ret; 
}

int ECDSASignBufferToLicenceFile(const char *szPrivateKeyPath,char *szBufferData,const int nBufferData,const char *szLicencePath)
{
	int nRet = 0;
	unsigned int nLicencelen   = 0;
	char szLicenceBuffer[1024] = {};
	nRet = ECDSASignToBuffer(szPrivateKeyPath,szBufferData,nBufferData,szLicenceBuffer,&nLicencelen);
	if (1 == nRet)
	{
		FILE *file = fopen(szLicencePath,"wb");
		if (file)
		{
			fwrite(szLicenceBuffer,nLicencelen,1,file);
			fclose(file);
		}
	}
	return nRet;

}

bool base16Encode(unsigned char* pBufferIn, int nBufferIn, unsigned char* pBufferOut, int nBufferOut, int* pBufferOutUsed)
{
	*pBufferOutUsed = 0;

	if (!pBufferIn || !pBufferOut)
		return false;

	if (nBufferOut < (nBufferIn * 2))
		return false;

	int i = 0;
	for (; i < nBufferIn; i++)
	{
		char szTmp[3] = {0};
		sprintf(szTmp, "%02X", *(pBufferIn + i));
		strcat((char*)pBufferOut, szTmp);
		*pBufferOutUsed += 2;
	}
	return true;
}

int ECDSASignBufferBase16ToLicenceFile(const char *szPrivateKeyPath,char *szBufferData,const int nBufferData,const char *szLicencePath)
{
	int nRet = 0;
	unsigned int nLicencelen   = 0;
	char szLicenceBuffer[512] = {};
	nRet = ECDSASignToBuffer(szPrivateKeyPath,szBufferData,nBufferData,szLicenceBuffer,&nLicencelen);
	if (1 == nRet)
	{
		FILE *file = fopen(szLicencePath,"wb");
		if (file)
		{
			int nLicenceOutlen      = 0;
			int nLicenceBase16len   = 1024;
			char szLicenceBase16Buffer[1024] = {};
			base16Encode((unsigned char *)szLicenceBuffer,nLicencelen,(unsigned char *)szLicenceBase16Buffer,nLicenceBase16len,&nLicenceOutlen);
			fwrite(szLicenceBase16Buffer,nLicenceOutlen,1,file);
			fclose(file);
		}
		else
		{
			nRet = 0;
		}
	}

	return nRet;
}


int ECDSASignFileToLicenceFile(const char *szPrivateKeyPath,const char *szDataFilePath,const char *szLicencePath)
{
	int nRet = 0;
	FILE *file = NULL;
	char *szDataFile = NULL;
	unsigned int nLicencelen   = 0;
	char szLicenceBuffer[1024] = {};

	do 
	{
		int nFileLen;
		if (1 != ReadFileECC(szDataFilePath,&szDataFile,&nFileLen))
		{
			printf("文件%s打开失败\n",szDataFilePath);
			nRet = -1;
			break;
		}

		nRet = ECDSASignToBuffer(szPrivateKeyPath,szDataFile,nFileLen,szLicenceBuffer,&nLicencelen);
		if (1 == nRet)
		{
			file = fopen(szLicencePath,"wb");
			if (file)
			{
				fwrite(szLicenceBuffer,nLicencelen,1,file);
			}
		}
	} while (0);
	
	if (file)
	{
		fclose(file);
		file = NULL;
	}

	if (szDataFile)
	{
		free(szDataFile);
		szDataFile = NULL;
	}
	
	return nRet;
}

int ECDSAVerifyLicenceFile(const char *szPublicKeyPath,const char *szDataFilePath,const char *szLicencePath)
{
	int ret = 0;
	FILE *file = NULL; 
	char *pDataFileBuffer  = NULL;
	char *szLicenceBuffer  = NULL;

	do 
	{
		int nDataFilelen  = 0;
		int nLicencelen   = 0;
		if (1 != ReadFileECC(szDataFilePath,&pDataFileBuffer,&nDataFilelen))
		{
			printf("文件%s打开失败\n",szDataFilePath);
			ret = -1;
			break;
		}
		
		if (1 != ReadFileECC(szLicencePath,&szLicenceBuffer,&nLicencelen))
		{
			printf("文件%s打开失败\n",szLicencePath);
			ret = -1;
			break;
		}
		ret = ECDSAVerifyLicenceBuffer(szPublicKeyPath,pDataFileBuffer,nDataFilelen,szLicenceBuffer,nLicencelen);
	} while (0);

	if (pDataFileBuffer)
	{
		free(pDataFileBuffer);
		pDataFileBuffer = NULL;
	}

	if (szLicenceBuffer)
	{
		free(szLicenceBuffer);
		szLicenceBuffer = NULL;
	}

	if (file)
	{
		fclose(file);
		file = NULL;
	}
	return ret;

}

int ECDSAVerifyBase16LicenceFile(const char *szPublicKeyPath,const char *szDataFilePath,const char *szLicencePath)
{
	int ret = 0;
	FILE *file = NULL; 
	char *pDataFileBuffer  = NULL;
	char *szLicenceBuffer  = NULL;
	char *szDecodeLicenceBuffer = NULL;

	do 
	{
		int nDataFilelen  = 0;
		int nLicencelen   = 0;
		if (1 != ReadFileECC(szDataFilePath,&pDataFileBuffer,&nDataFilelen))
		{
			printf("文件%s打开失败\n",szDataFilePath);
			ret = -1;
			break;
		}

		if (1 != ReadFileECC(szLicencePath,&szLicenceBuffer,&nLicencelen))
		{
			printf("文件%s打开失败\n",szLicencePath);
			ret = -2;
			break;
		}

		
		szDecodeLicenceBuffer = (char*)malloc(nLicencelen*2+5);

		int nOutlen = 0;
		ReadBufferHex(szLicenceBuffer,nLicencelen,szDecodeLicenceBuffer,&nOutlen);
		ret = ECDSAVerifyLicenceBuffer(szPublicKeyPath,pDataFileBuffer,nDataFilelen,szDecodeLicenceBuffer,nOutlen);
	} while (0);

	if (pDataFileBuffer)
	{
		free(pDataFileBuffer);
		pDataFileBuffer = NULL;
	}

	if (szDecodeLicenceBuffer)
	{
		free(szDecodeLicenceBuffer);
		szDecodeLicenceBuffer = NULL;
	}

	if (szLicenceBuffer)
	{
		free(szLicenceBuffer);
		szLicenceBuffer = NULL;
	}

	if (file)
	{
		fclose(file);
		file = NULL;
	}
	return ret;
}

int GetECDHShareKeyFromSrvPublicKey(const char *szSrvPublicKey,const int nSrvPublicKeylen,const char *szSharekey,const char *szBufClientPubKey,int *pClientPubKeylen)
{
	OpenSSL_add_all_ciphers();
	OpenSSL_add_all_algorithms();

	int nResult = 0;
	EC_KEY   *ecKeyClient = NULL;
	EC_POINT *pointServer = NULL;
	do
	{
		ecKeyClient = EC_KEY_new_by_curve_name(NID_secp192k1);
		if( ecKeyClient == NULL )
		{
			log_notice("ECDH:EC_KEY_new_by_curve_name failed.");
			nResult = -1;
			break;
		}

		if (!EC_KEY_generate_key(ecKeyClient))
		{
			log_notice("ECDH:EC_KEY_generate_key failed.");
			nResult = -2;
			break;
		}

		// 客户端公钥
		const EC_POINT *pointClient = EC_KEY_get0_public_key(ecKeyClient);
		if( NULL == pointClient )
		{
			log_notice("ECDH:EC_KEY_get0_public_key failed.");
			nResult = -3;
			break;
		}

		char pubkeyClient[512] = {0};
		int  pubkeyLenCli = 0;
		pubkeyLenCli = EC_POINT_point2oct(EC_KEY_get0_group(ecKeyClient), pointClient, POINT_CONVERSION_COMPRESSED, (unsigned char*)pubkeyClient, ECDH_SIZE, NULL);
		if( pubkeyLenCli <= 0 )
		{
			log_notice("ECDH:EC_POINT_point2oct failed, pubkeyLenCli:%d.", pubkeyLenCli);
			nResult = -4;
			break;
		}

		memcpy((void*)szBufClientPubKey,pubkeyClient,pubkeyLenCli);
		if (pClientPubKeylen)
		{
			*pClientPubKeylen = pubkeyLenCli;
		}

		// 通过服务端公钥计算共享私钥 
		// 服务端公钥
		char pubkeyServer[128] = {0};
		int  pubkeyLenSvr = sizeof(pubkeyServer);
		ReadBufferHex((char*)szSrvPublicKey, nSrvPublicKeylen, pubkeyServer, &pubkeyLenSvr);

		const EC_GROUP* group = EC_KEY_get0_group((ecKeyClient));
		if(NULL == group)
		{
			log_notice("ECDH:EC_KEY_get0_group failed, return NULL.");
			nResult = -5;
			break;
		}

		pointServer = EC_POINT_new(group);
		if (NULL == pointServer)
		{
			log_notice("ECDH:EC_POINT_new failed, return NULL.");
			nResult = -6;
			break;
		}

		if(!EC_POINT_oct2point(group, pointServer, (const unsigned char*)pubkeyServer, pubkeyLenSvr, NULL))
		{
			log_notice("ECDH:EC_POINT_oct2point failed, return NULL.");
			nResult = -7;
			break;
		}

		// 计算共享密钥
		char  sharekey[512] = {0};
		int   sharekey_len = 0;
		sharekey_len = ECDH_compute_key(sharekey, sizeof(sharekey), pointServer, ecKeyClient, NULL);
		if( sharekey_len <= 0)
		{
			nResult = -8;
			log_notice("ECDH:ECDH_compute_key failed: %d", sharekey_len);
		}
		else
		{
			// 计算MD5
			Md5HashBuffer((uint8 *)szSharekey, sharekey, sharekey_len);
			nResult = 1;
		}
	}while(0);

	if(NULL != ecKeyClient)
	{
		EC_KEY_free(ecKeyClient);
	}
	if(NULL != pointServer)
	{
		EC_POINT_free(pointServer);
	}

	return nResult;
}



//我们自己的验证函数
int qq_license_verification(char *lisencebuf)
{
	char lisence[1024]={0};
	char lisence_change[2055]={0};
	char sn[20]={0};
	char *lisence_point;
	char *lisence_change_point;
	char *sn_point;
	char *public_key="/usr/share/brxydata/public.pem";
	int lisence_length=0;
	int lisence_change_length=0;
	int sn_length=0;
	int ret;

	lisence_point=lisence;
	lisence_change_point=lisence_change;
	sn_point=sn;
	strcpy(lisence,lisencebuf);
	strcpy(sn,base_info.sn);
	lisence_length=strlen(lisence);
	sn_length=16;
	//ReadBufferHex(szLicenceBuffer,nLicencelen,szDecodeLicenceBuffer,&nOutlen);

	ReadBufferHex(lisence_point,lisence_length,lisence_change_point,&lisence_change_length);
	//ret = ECDSAVerifyLicenceBuffer(szPublicKeyPath,pDataFileBuffer,nDataFilelen,szDecodeLicenceBuffer,nOutlen);
	ret=ECDSAVerifyLicenceBuffer(public_key,sn_point,sn_length,lisence_change_point,lisence_change_length);
	return ret;//返回1成功
}










/*************************************************************************************
以上是qq license 验证用到的函数

*************************************************************************************/









/*************************************************************************************
以下是i2c用到的函数

*************************************************************************************/

static int i2c_init(void)
{
	int ret;
	i2c_fd=open("/dev/i2c-0",O_RDWR);
	if(i2c_fd<0)
		{
			printf("i2c adopter open false\n");
			return -1;
		}
	ioctl(i2c_fd,I2C_TIMEOUT,2);
	ioctl(i2c_fd,I2C_RETRIES,1);
	
	i2c_write_ioctl_data.nmsgs=1;
	//i2c_write_ioctl_data.msgs = (struct i2c_msg *)malloc(sizeof(struct i2c_msg));
	while((i2c_write_ioctl_data.msgs = (struct i2c_msg *)malloc(sizeof(struct i2c_msg)))==NULL)
		{
			printf("malloc false\n");
			sleep(2);
		}
	if(!i2c_write_ioctl_data.msgs)
		{
			printf("malloc i2c_write_ioctl_data false\n");
			close(i2c_fd);
			return -1;
		}
	i2c_read_ioctl_data.nmsgs=2;
	//i2c_read_ioctl_data.msgs=(struct i2c_msg *)malloc(2*sizeof(struct i2c_msg));
	while((i2c_read_ioctl_data.msgs=(struct i2c_msg *)malloc(2*sizeof(struct i2c_msg)))==NULL)
		{
			printf("malloc false\n");
			sleep(2);
		}
	if(!i2c_read_ioctl_data.msgs)
		{
			printf("malloc i2c_read_ioctl_data false\n");
			close(i2c_fd);
			return -1;
		}
	return 1;
}

static int change_i2c_device_address(unsigned char address)
{
	int err=0;
	err=ioctl(i2c_fd,I2C_SLAVE_FORCE,address);
	if(err<0)
		{
			printf("change i2c_device_address false\n");
			return -1;
		}
	return 1;
}
static int i2c_write(unsigned char sla,unsigned int regi,unsigned int val)
{
	unsigned char buff[2];
	int ret=0;
	
	buff[0]=(unsigned char)regi;
	buff[1]=(unsigned char)val;
	//printf("%d\n",buff[1]);
	i2c_write_ioctl_data.msgs[0].addr=sla;
	(i2c_write_ioctl_data.msgs[0]).len=2;
	i2c_write_ioctl_data.msgs[0].flags=0;//写命令
	i2c_write_ioctl_data.msgs[0].buf=buff;
	while(1)
		{
			ret=ioctl(i2c_fd,I2C_RDWR,(unsigned long)&i2c_write_ioctl_data);
			if(ret<0)
				{
					printf("i2c write to slave false\n");
					close(i2c_fd);
					i2c_fd=open("/dev/i2c-0",O_RDWR);
					if(i2c_fd<0)
						{
							printf("i2c adopter open false\n");
						}
					ioctl(i2c_fd,I2C_TIMEOUT,2);
					ioctl(i2c_fd,I2C_RETRIES,1);
						}
			else
				{
					break;
				}
			sleep(2);
		}
	return 1;
}
static int i2c_read(unsigned char sla,unsigned int regi1,unsigned char *val)
{
	
	int ret;
	unsigned char regi;
	unsigned char out=0;
	regi=(unsigned char)regi1;
	i2c_read_ioctl_data.msgs[0].addr=sla;
	(i2c_read_ioctl_data.msgs[0]).len=1;
	i2c_read_ioctl_data.msgs[0].flags=0;
	i2c_read_ioctl_data.msgs[0].buf=&regi;
	
	i2c_read_ioctl_data.msgs[1].addr=sla;
	(i2c_read_ioctl_data.msgs[1]).len=1;
	i2c_read_ioctl_data.msgs[1].flags=I2C_M_RD;
	i2c_read_ioctl_data.msgs[1].buf=&out;

	//ret=ioctl(i2c_fd, I2C_SLAVE_FORCE, sla); 
	//if(ret<0)
	//	{
	//		printf("ioctl als false\n");
	//	}

	while(1)
			{
				//ret=ioctl(i2c_fd,I2C_RDWR,(unsigned long)&i2c_write_ioctl_data);
				ret=ioctl(i2c_fd,I2C_RDWR,&i2c_read_ioctl_data);
				if(ret<0)
					{
						printf("i2c read to slave false\n");
						close(i2c_fd);
						i2c_fd=open("/dev/i2c-0",O_RDWR);
						if(i2c_fd<0)
							{
								printf("i2c adopter open false\n");
							}
						ioctl(i2c_fd,I2C_TIMEOUT,2);
						ioctl(i2c_fd,I2C_RETRIES,1);
							}
				else
					{
						break;
					}
				sleep(2);
			}
	//printf("out = %d\n",out);
	*val=out;
	return 1;
}
/*************************************************************************************
3104芯片初始化及控制函数

*************************************************************************************/

static int TLV320_BASSWrite(int number)
{
  int i;
  unsigned char save;
  if (number == 0)
  {
    return -1;
  }
  // level range 1 ~ 15
  number = (number - 1)* 10;
 
  i2c_write(i2c_address_3105,0, 0x00);        // page 0
  i2c_write(i2c_address_3105, 43, 0xff);      // mute dac
  if(i2c_read(i2c_address_3105,37,&save)<0)
  	{
  		printf("read i2c 3105 37 false\n");
  		return -1;
  	}
  
  i2c_write(i2c_address_3105,37, 0x20);       // DAC power down
  i2c_write(i2c_address_3105,12, 0x00);       // DAC effect Disabel
  
  for(i=0;i<100;i++);
  
  i2c_write(i2c_address_3105,0, 0x01);        // page 1
  for(i=0;i<10;i++)
  {
    // Config N0,N1,N2,D1,D2
    i2c_write(i2c_address_3105,LRB1Addr[i], AIC3105Bass[i + number]); 
  }
  i2c_write(i2c_address_3105,0, 0x00);        // page 0
  i2c_write(i2c_address_3105,37, save);       // enable dac
  i2c_write(i2c_address_3105, 43, 0x00);      //
  i2c_write(i2c_address_3105,12, 0x0a);       // DAC effect enable
  return 1;
}

static int TLV320_TreableWrite(int number)
{
  int i;
  unsigned char save;
  number = (number - 1)* 10;
  i2c_write(i2c_address_3105,0, 0x00);         // page 0
  i2c_write(i2c_address_3105,43, 0xff);        // mute dac
  if(i2c_read(i2c_address_3105,37,&save)<0)
  	{
  		printf("read i2c 3105 37 false\n");
  		return -1;
  	}
  i2c_write(i2c_address_3105,37, 0x20);        // DAC Power down
  i2c_write(i2c_address_3105,12, 0x00);        // DAC effect Disabel
  for(i=0;i<100;i++);
  
  i2c_write(i2c_address_3105,0, 0x01);         //page 1
  for(i=0;i<10;i++)
  {
    // Config N3,N4,N5,D4,D5
    i2c_write(i2c_address_3105,LRB2Addr[i], AIC3105Treble[i + number]); 
  }
  i2c_write(i2c_address_3105,0, 0x00);         // page 0
  i2c_write(i2c_address_3105,37, save);        // enable dac
  i2c_write(i2c_address_3105, 43, 0x00);    
  i2c_write(i2c_address_3105,12, 0x0a);        // DAC effect enable
  return 1;
}


void Tlv320_EnMute(void)
{
  
    i2c_write(i2c_address_3105,0, 0x00);
    i2c_write(i2c_address_3105,43,0xff);
    i2c_write(i2c_address_3105,44,0xff);
    //WriteByte_tlv320(37,0x20);
}

void Tlv320_DisMute(void)
{
  
    i2c_write(i2c_address_3105,0, 0x00);
    i2c_write(i2c_address_3105,43,0x00);
    i2c_write(i2c_address_3105,44,0x00);
    //WriteByte_tlv320(37,0xe0);

   
}
void tlv320_reset(void)
{
  i2c_write( i2c_address_3105,0, 0x00);
	i2c_write(i2c_address_3105, 1, 0x80);
}

void Init_tlv320(void)
{
 
  //read_eeprom(SOUND_PAGE);
  // TLV320 page0
  int voice_data;
  int high_data=15;
  int low_data=1;
  i2c_write(i2c_address_3105, 0, 0x00);
	//WriteByte_tlv320( 1, 0x80);
  
	i2c_write(i2c_address_3105, 4, 0x80);
	i2c_write( i2c_address_3105,102, 0x22);
	i2c_write( i2c_address_3105,3, 0x91);
	
	i2c_write( i2c_address_3105,7, 0x0a);
	i2c_write( i2c_address_3105,9, 0x3f);
  i2c_write( i2c_address_3105,12, 0x0a);      // DAC effect enalbe
	i2c_write(i2c_address_3105, 14, 0xc0);
  i2c_write(i2c_address_3105, 17, 0x00);//0f
  i2c_write(i2c_address_3105, 18, 0x00);//f0
	i2c_write(i2c_address_3105, 19, 0x04);//04
  i2c_write(i2c_address_3105, 21, 0x00);
	i2c_write(i2c_address_3105, 22, 0x04);
  i2c_write( i2c_address_3105,24, 0x00);
  i2c_write(i2c_address_3105, 37, 0xe0);      // Enable dac
  i2c_write( i2c_address_3105,38, 0x10);      // Enable dac
   
	i2c_write(i2c_address_3105, 40, 0x14);
	i2c_write(i2c_address_3105, 42, 0xa7);
  i2c_write(i2c_address_3105, 43, 0x00);     
  i2c_write( i2c_address_3105,44, 0x00);
  
	i2c_write( i2c_address_3105,46, 0x80);
	i2c_write( i2c_address_3105,47, 0x80);
	i2c_write(i2c_address_3105, 63, 0x80);
	i2c_write(i2c_address_3105, 64, 0x80);
	i2c_write( i2c_address_3105,81, 0x80);
	i2c_write(i2c_address_3105, 82, 0x80);
	i2c_write(i2c_address_3105, 91, 0x80);
	i2c_write(i2c_address_3105, 92, 0x80);
	i2c_write( i2c_address_3105,51, 0x0f);
	i2c_write(i2c_address_3105, 65, 0x0f);
	i2c_write(i2c_address_3105, 86, 0x09);
	i2c_write(i2c_address_3105, 93, 0x09);
  
  i2c_write( i2c_address_3105,53, 0x80);
  i2c_write(i2c_address_3105, 54, 0x80);
  i2c_write( i2c_address_3105,58, 0x0f);
  i2c_write( i2c_address_3105,70, 0x80);
  i2c_write( i2c_address_3105,71, 0x80);
  i2c_write(i2c_address_3105, 72, 0x0f);//?

  if((volume_3105.volume>0)&&(volume_3105.volume<9)) 
  {
    voice_data = volume_3105.volume;
    VOLUME_TLV320(voice_data);
  } 
  else
  {
    voice_data = 4;  
    VOLUME_TLV320(voice_data);
  }
  
  if ((volume_3105.high >= 1)&&(volume_3105.high<= 15))
  {
    // Config TLV320 Treable
    high_data=volume_3105.high;
    
  }
  TLV320_TreableWrite(high_data);    
  if ((volume_3105.low>= 1)&&(volume_3105.low<= 15))
  {
    // Config TLV320 Bass
    low_data=volume_3105.low;
    
  }         
  TLV320_BASSWrite(low_data);         
}

void VOLUME_TLV320(int volume)
{  
  i2c_write(i2c_address_3105,0, 0x00);
	i2c_write(i2c_address_3105, 46, 0x80|(88-((volume-1)*10)));//70
	i2c_write(i2c_address_3105, 47, 0x80|(88-((volume-1)*10)));
  i2c_write( i2c_address_3105,53, 0x80|(88-((volume-1)*10)));//70
	i2c_write(i2c_address_3105, 54, 0x80|(88-((volume-1)*10)));
	i2c_write(i2c_address_3105, 63, 0x80|(88-((volume-1)*10)));
	i2c_write( i2c_address_3105,64, 0x80|(88-((volume-1)*10)));
  i2c_write( i2c_address_3105,70, 0x80|(88-((volume-1)*10)));//70
	i2c_write(i2c_address_3105, 71, 0x80|(88-((volume-1)*10)));
	i2c_write( i2c_address_3105,81, 0x80|(73-((volume-1)*10)));
	i2c_write(i2c_address_3105, 82, 0x80|(73-((volume-1)*10)));
	i2c_write( i2c_address_3105,91, 0x80|(73-((volume-1)*10)));
	i2c_write(i2c_address_3105, 92, 0x80|(73-((volume-1)*10)));  
   
}

void MUTE_TLV320(int mute)  // Mute 
{
  i2c_write(i2c_address_3105,0, 0x00);
	if(mute)
	{
		i2c_write(i2c_address_3105, 86, 0x03);//86
		i2c_write(i2c_address_3105, 93, 0x03);//93	
		i2c_write(i2c_address_3105, 51, 0x07);//51
		i2c_write(i2c_address_3105, 65, 0x07);//65
    i2c_write( i2c_address_3105,58, 0x07);//
    i2c_write(i2c_address_3105, 72, 0x07);//    
    //////
    i2c_write(i2c_address_3105, 46, 0);//70
    i2c_write(i2c_address_3105, 47, 0);
    i2c_write(i2c_address_3105, 53, 0);//70
    i2c_write( i2c_address_3105,54, 0);
    i2c_write(i2c_address_3105, 63, 0);//70
    i2c_write(i2c_address_3105, 64, 0);
    i2c_write( i2c_address_3105,70, 0);//70
    i2c_write(i2c_address_3105, 71, 0);
    i2c_write( i2c_address_3105,81, 0);//70
    i2c_write( i2c_address_3105,82, 0);
    i2c_write(i2c_address_3105, 91, 0);//70
    i2c_write(i2c_address_3105, 92, 0);
    i2c_write(i2c_address_3105, 37, 0);//70
    //WriteByte_tlv320( 43, 0xf);
    
	}
	else
	{
		i2c_write(i2c_address_3105, 86, 0x99);
		i2c_write(i2c_address_3105, 93, 0x99);	
		i2c_write( i2c_address_3105,51, 0x0f);
		i2c_write(i2c_address_3105, 65, 0x0f);
    i2c_write( i2c_address_3105,58, 0x0f);
    i2c_write( i2c_address_3105,72, 0x0f);     
	}
}
/*************************************************************************************
ep芯片初始化函数

*************************************************************************************/

int ep_passage_switch(int channel)/*HDMI?D??*/
{
	unsigned char temp=0;  
  //i2c_write(i2c_address_ep,0x12,0x20);
  pthread_mutex_lock(&i2c_mutex);
  i2c_write(i2c_address_ep,0x13,0x30);
  i2c_write(i2c_address_ep,0x10,0x80);

  i2c_write(i2c_address_ep,0x10,0xa0);
  
  //temp = ReadByte_ep92a3e(0x11);
  if(i2c_read(i2c_address_ep,0x11,&temp)<0)
  	{
  		printf("read i2c ep 0x11 input port(0/1/2) false\n");
  		return -1;
  	}
 // pthread_mutex_unlock(&i2c_mutex);
  temp &= 0x0F;
  // mute audio tx
 // i2c_write(i2c_address_ep,0x11,temp);
 // usleep(3000000);
 //sleep(3);
  // select video tx channel
  if (channel == 1)
  {
    temp = 0x00;
  }
  else if (channel == 0)
  {
    temp = 0x01;
  }
  else if (channel == 3)
  {
    temp = 0x02;
  }
  i2c_write(i2c_address_ep,0x11,temp);
  pthread_mutex_unlock(&i2c_mutex);
 // usleep(3000000);
 sleep(3);
  // enable audio tx freq:48k
  temp |= 0x30; 
  pthread_mutex_lock(&i2c_mutex);
  i2c_write(i2c_address_ep,0x11,temp); 
  pthread_mutex_unlock(&i2c_mutex);
  //usleep(3000000);
  sleep(3);
  return 1;
}


int ep_passage_switch1(int channel)/*HDMI?D??*/
{
	unsigned char temp=0;  
  //i2c_write(i2c_address_ep,0x12,0x20);
  //i2c_write(i2c_address_ep,0x10,0x20);
  
  i2c_write(i2c_address_ep,0x13,0x30);
  //i2c_write(i2c_address_ep,0x10,0x80);

 // i2c_write(i2c_address_ep,0x10,0xa0);
  
  //temp = ReadByte_ep92a3e(0x11);
 // if(i2c_read(i2c_address_ep,0x11,&temp)<0)
  	//{
  	//	printf("read i2c ep 0x11 input port(0/1/2) false\n");
  	//	return -1;
  //	}
 // temp &= 0x0F;
  // mute audio tx
 // i2c_write(i2c_address_ep,0x11,temp);
 // usleep(3000000);
  // select video tx channel
  if (channel == 1)
  {
    temp = 0x00;
  }
  else if (channel == 0)
  {
    temp = 0x01;
  }
  else if (channel == 3)
  {
    temp = 0x02;
  }
  // enable audio tx freq:48k
  temp |= 0x30; 
  i2c_write(i2c_address_ep,0x11,temp); 
  //i2c_write(i2c_address_ep,0x10,0xa0);
 // usleep(6000000);
  return 1;
}
int ep_passage_onoff(int onoff)/*HDMI?D??*/
{
	unsigned char temp =0;  
   
  //temp = ReadByte_ep92a3e(0x10);
  if(i2c_read(i2c_address_ep,0x10,&temp)<0)
  	{
  		printf("read i2c ep 0x10 power on/off false\n");
  		return -1;
  	}
  temp &= 0x7F;
  if (onoff == 1)
  {
    temp |= 0x80;
  }
  i2c_write(i2c_address_ep,0x10,temp); 
 // i2c_write(i2c_address_ep,0x11,0x01);
  //i2c_write(i2c_address_ep,0x10,0x31);
 // usleep(30000000);
  sleep(12);
  return 1;
}
/*************************************************************************************
rfid芯片处理函数

*************************************************************************************/

void Delay_I_1us(uint16_t k)
{
 // uint16_t i,j;
 // for(i=0;i<k;i++)
  //  for(j=0;j<48;j++);
  usleep(k*20);
}

/////////////////////////////////////////////////////////////////////
//功    能：读RC522寄存器
//参数说明：Address[IN]:寄存器地址
//返    回：读出的值
/////////////////////////////////////////////////////////////////////
unsigned char ReadRawRC(unsigned char Address)
{   

	unsigned char buff;
	i2c_read(i2c_address_rfid,Address,&buff);
  //I2C1Buffer[0] = Address; 
 // I2C1_WriteRead(RC522_ADDR,2,READ_CMD);
 // return I2C1Buffer[1];
 return buff;
}
/////////////////////////////////////////////////////////////////////
//功    能：写RC632寄存器
//参数说明：Address[IN]:寄存器地址
//          value[IN]:写入的值
/////////////////////////////////////////////////////////////////////
void WriteRawRC(unsigned char Address, unsigned char value)
{  
	i2c_write(i2c_address_rfid,(unsigned int)Address,(unsigned int)value);
  //I2C1Buffer[0] = Address;
 // I2C1Buffer[1] = value;  
 // I2C1_WriteRead(RC522_ADDR,2,WRITE_CMD);
}


/////////////////////////////////////////////////////////////////////
//功    能：置RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//          mask[IN]:置位值
/////////////////////////////////////////////////////////////////////
void  SetBitMask(unsigned char reg,unsigned char mask)  
{
    unsigned char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg,tmp | mask);  // set bit mask
}

/////////////////////////////////////////////////////////////////////
//功    能：清RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//          mask[IN]:清位值
/////////////////////////////////////////////////////////////////////
void  ClearBitMask(unsigned char reg,unsigned char mask)  
{
    unsigned char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg, tmp & ~mask);  // clear bit mask
} 

/////////////////////////////////////////////////////////////////////
//开启天线  
//每次启动或关闭天险发射之间应至少有1ms的间隔
/////////////////////////////////////////////////////////////////////
void PcdAntennaOn(void)
{
    unsigned char i;
    i = ReadRawRC(TxControlReg);
    if (!(i & 0x03))
    {
        SetBitMask(TxControlReg, 0x03);
    }
}

/////////////////////////////////////////////////////////////////////
//关闭天线
/////////////////////////////////////////////////////////////////////
void PcdAntennaOff(void)
{
  ClearBitMask(TxControlReg, 0x03);
}


/////////////////////////////////////////////////////////////////////
//功    能：复位RC522
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
void PcdReset(void)
{
	//PORTD|=(1<<RC522RST);
	//RC522_REST = 1;
   // RC522_RESET_H();
    Delay_I_1us(1);
	//PORTD&=~(1<<RC522RST);
	//RC522_REST = 0;
   // RC522_RESET_L();
    Delay_I_1us(3);
	//PORTD|=(1<<RC522RST);
	//RC522_REST = 1;
   // RC522_RESET_H();
    Delay_I_1us(2);
    WriteRawRC(0x01,0x0f);
    while(ReadRawRC(0x01)&0x10);   //检查卡片
    Delay_I_1us(10);
    
    WriteRawRC(ModeReg,0x3D);      //定义发送和接收常用模式 和Mifare卡通讯，CRC初始值0x6363
    WriteRawRC(TReloadRegL,30);    //16位定时器低位
    WriteRawRC(TReloadRegH,0);		 //16位定时器高位
    WriteRawRC(TModeReg,0x8D);		 //定义内部定时器的设置
    WriteRawRC(TPrescalerReg,0x3E);//设置定时器分频系数
    WriteRawRC(TxAutoReg,0x40);		 //	调制发送信号为100%ASK
   
        
    //return MI_OK;
}


//////////////////////////////////////////////////////////////////////
//设置RC632的工作方式 
//////////////////////////////////////////////////////////////////////

void M500PcdConfigISOType(unsigned char type)
{
   if (type == 'A')                     //ISO14443_A
   { 
       ClearBitMask(Status2Reg,0x08);
       WriteRawRC(ModeReg,0x3D);//3F
       WriteRawRC(RxSelReg,0x86);//84
       WriteRawRC(RFCfgReg,0x7F);   //4F
   	   WriteRawRC(TReloadRegL,30);//tmoLength);// TReloadVal = 'h6a =tmoLength(dec) 
	   WriteRawRC(TReloadRegH,0);
       WriteRawRC(TModeReg,0x8D);
	   WriteRawRC(TPrescalerReg,0x3E);
	   Delay_I_1us(2);
       PcdAntennaOn();//开天线
   }
 //  else return (-1); 
   
   //return MI_OK;
}

/////////////////////////////////////////////////////////////////////
//功    能：通过RC522和ISO14443卡通讯
//参数说明：Command[IN]:RC522命令字
//          pInData[IN]:通过RC522发送到卡片的数据
//          InLenByte[IN]:发送数据的字节长度
//          pOutData[OUT]:接收到的卡片返回数据
//          *pOutLenBit[OUT]:返回数据的位长度
/////////////////////////////////////////////////////////////////////

unsigned char PcdComMF522(unsigned char Command, 		//RC522命令字
                 unsigned char *pInData, 		//通过RC522发送到卡片的数据
                 unsigned char InLenByte,		//发送数据的字节长度
                 unsigned char *pOutData, 		//接收到的卡片返回数据
                 uint16_t  *pOutLenBit)		//返回数据的位长度
{
    unsigned char status = MI_ERR;
    unsigned char irqEn   = 0x00;
    unsigned char waitFor = 0x00;
    unsigned char lastBits;
    unsigned char n;
    uint16_t i;
    switch (Command)
    {
       case PCD_AUTHENT:		//Mifare认证
          irqEn   = 0x12;		//允许错误中断请求ErrIEn  允许空闲中断IdleIEn
          waitFor = 0x10;		//认证寻卡等待时候 查询空闲中断标志位
          break;
       case PCD_TRANSCEIVE:		//接收发送 发送接收
          irqEn   = 0x77;		//允许TxIEn RxIEn IdleIEn LoAlertIEn ErrIEn TimerIEn
          waitFor = 0x30;		//寻卡等待时候 查询接收中断标志位与 空闲中断标志位
          break;
       default:
         break;
    }
   
    WriteRawRC(ComIEnReg,irqEn|0x80);		//IRqInv置位管脚IRQ与Status1Reg的IRq位的值相反 
    ClearBitMask(ComIrqReg,0x80);			//Set1该位清零时，CommIRqReg的屏蔽位清零
    WriteRawRC(CommandReg,PCD_IDLE);		//写空闲命令
    SetBitMask(FIFOLevelReg,0x80);			//置位FlushBuffer清除内部FIFO的读和写指针以及ErrReg的BufferOvfl标志位被清除
    
    for (i=0; i<InLenByte; i++)
    {   WriteRawRC(FIFODataReg, pInData[i]);    }		//写数据进FIFOdata
    WriteRawRC(CommandReg, Command);					//写命令
   
    
    if (Command == PCD_TRANSCEIVE)
    {    SetBitMask(BitFramingReg,0x80);  }				//StartSend置位启动数据发送 该位与收发命令使用时才有效
    
    i = 1000;//根据时钟频率调整，操作M1卡最大等待时间25ms
    do 														//认证 与寻卡等待时间	
    {
         n = ReadRawRC(ComIrqReg);							//查询事件中断
         i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitFor));		//退出条件i=0,定时器中断，与写空闲命令
    ClearBitMask(BitFramingReg,0x80);					//清理允许StartSend位
    if (i!=0)
    {    
         if(!(ReadRawRC(ErrorReg)&0x1B))			//读错误标志寄存器BufferOfI CollErr ParityErr ProtocolErr
         {
             status = MI_OK;
             if (n & irqEn & 0x01)					//是否发生定时器中断
             {   status = MI_NOTAGERR;   }
             if (Command == PCD_TRANSCEIVE)
             {
               	n = ReadRawRC(FIFOLevelReg);			//读FIFO中保存的字节数
              	lastBits = ReadRawRC(ControlReg) & 0x07;	//最后接收到得字节的有效位数
                if (lastBits)
                {   *pOutLenBit = (n-1)*8 + lastBits;   }	//N个字节数减去1（最后一个字节）+最后一位的位数 读取到的数据总位数
                else
                {   *pOutLenBit = n*8;   }					//最后接收到的字节整个字节有效
                if (n == 0)									
                {   n = 1;    }
                if (n > MAXRLEN)
                {   n = MAXRLEN;   }
                for (i=0; i<n; i++)
                {   pOutData[i] = ReadRawRC(FIFODataReg);    }
            }
         }
         else
         {   status = MI_ERR;   }
   }
   
   SetBitMask(ControlReg,0x80);           // stop timer now
   WriteRawRC(CommandReg,PCD_IDLE); 
   return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：寻卡
//参数说明: req_code[IN]:寻卡方式
//                0x52 = 寻感应区内所有符合14443A标准的卡
//                0x26 = 寻未进入休眠状态的卡
//          pTagType[OUT]：卡片类型代码
//                0x4400 = Mifare_UltraLight
//                0x0400 = Mifare_One(S50)
//                0x0200 = Mifare_One(S70)
//                0x0800 = Mifare_Pro(X)
//                0x4403 = Mifare_DESFire
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////

unsigned char PcdRequest(unsigned char req_code,unsigned char *pTagType)
{
   unsigned char status;  
   //uint i;
   uint16_t  unLen;
   unsigned char ucComMF522Buf[MAXRLEN]; 

   ClearBitMask(Status2Reg,0x08);	//清理指示MIFARECyptol单元接通以及所有卡的数据通信被加密的情况
   WriteRawRC(BitFramingReg,0x07);	//	发送的最后一个字节的 七位
   SetBitMask(TxControlReg,0x03);	//TX1,TX2管脚的输出信号传递经发送调制的13.56的能量载波信号

   ucComMF522Buf[0] = req_code;		//存入 卡片命令字

   status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);	//寻卡    
   if ((status == MI_OK) && (unLen == 0x10))	//寻卡成功返回卡类型 
   {    
       *pTagType     = ucComMF522Buf[0];
       *(pTagType+1) = ucComMF522Buf[1];
   }
   else
   {   
		status = MI_ERR;
	}
   
   return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：防冲撞
//参数说明: pSnr[OUT]:卡片序列号，4字节
//返    回: 成功返回MI_OK
///////////////////////////////////////////////////////////////////// 
unsigned char PcdAnticoll(unsigned char *pSnr)
{
    unsigned char status;
    unsigned char i,snr_check=0;
    uint16_t  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    

    ClearBitMask(Status2Reg,0x08);		//清MFCryptol On位 只有成功执行MFAuthent命令后，该位才能置位
    WriteRawRC(BitFramingReg,0x00);		//清理寄存器 停止收发
    ClearBitMask(CollReg,0x80);			//清ValuesAfterColl所有接收的位在冲突后被清除
    
  // WriteRawRC(BitFramingReg,0x07);	//	发送的最后一个字节的 七位
  // SetBitMask(TxControlReg,0x03);	//TX1,TX2管脚的输出信号传递经发送调制的13.56的能量载波信号
   
    ucComMF522Buf[0] = 0x93;	//卡片防冲突命令
    ucComMF522Buf[1] = 0x20;
   
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);//与卡片通信
    if (status == MI_OK)		//通信成功
    {
    	for (i=0; i<4; i++)
         {   
             *(pSnr+i)  = ucComMF522Buf[i];			//读出UID
             snr_check ^= ucComMF522Buf[i];

         }
         if (snr_check != ucComMF522Buf[i])
         {   status = MI_ERR;    }
    }
    
    SetBitMask(CollReg,0x80);
    return status;
}
/////////////////////////////////////////////////////////////////////
//用MF522计算CRC16函数
/////////////////////////////////////////////////////////////////////
void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData)
{
    unsigned char i,n;
    ClearBitMask(DivIrqReg,0x04);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<len; i++)
    {   WriteRawRC(FIFODataReg, *(pIndata+i));   }
    WriteRawRC(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do 
    {
        n = ReadRawRC(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));
    pOutData[0] = ReadRawRC(CRCResultRegL);
    pOutData[1] = ReadRawRC(CRCResultRegM);
}
/////////////////////////////////////////////////////////////////////
//功    能：选定卡片
//参数说明: pSnr[IN]:卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
unsigned char PcdSelect(unsigned char *pSnr)
{
    unsigned char status;
    unsigned char i;
    uint16_t  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i=0; i<4; i++)
    {
    	ucComMF522Buf[i+2] = *(pSnr+i);
    	ucComMF522Buf[6]  ^= *(pSnr+i);
    }
    CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
  
    ClearBitMask(Status2Reg,0x08);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
    
    if ((status == MI_OK) && (unLen == 0x18))
    {   status = MI_OK;  }
    else
    {   status = MI_ERR;    }

    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：验证卡片密码
//参数说明: auth_mode[IN]: 密码验证模式
//                 0x60 = 验证A密钥
//                 0x61 = 验证B密钥 
//          addr[IN]：块地址
//          pKey[IN]：密码
//          pSnr[IN]：卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////               
unsigned char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr)
{
    unsigned char status;
    uint16_t  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+2] = *(pKey+i);   }
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+8] = *(pSnr+i);   }
 //   memcpy(&ucComMF522Buf[2], pKey, 6); 
 //   memcpy(&ucComMF522Buf[8], pSnr, 4); 
    
    status = PcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
    if ((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
    {   status = MI_ERR;   }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：写数据到M1卡一块
//参数说明: addr[IN]：块地址
//          pData[IN]：写入的数据，16字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////   
unsigned char PcdWrite(unsigned char addr,unsigned char *pData)
{
    unsigned char status;
    uint16_t  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
        //memcpy(ucComMF522Buf, pData, 16);
        for (i=0; i<16; i++)
        {    ucComMF522Buf[i] = *(pData+i);   }
        CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);

        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    } 
    return status;
}
/////////////////////////////////////////////////////////////////////
//功    能：读取M1卡一块数据
//参数说明: addr[IN]：块地址
//          pData[OUT]：读出的数据，16字节
//返    回: 成功返回MI_OK
///////////////////////////////////////////////////////////////////// 
unsigned char PcdRead(unsigned char addr,unsigned char *pData)
{
    unsigned char status;
    uint16_t  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
   
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    if ((status == MI_OK) && (unLen == 0x90))
 //   {   memcpy(pData, ucComMF522Buf, 16);   }
    {
        for (i=0; i<16; i++)
        {    *(pData+i) = ucComMF522Buf[i];   }
    }
    else
    {   status = MI_ERR;   }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：命令卡片进入休眠状态
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
unsigned char PcdHalt(void)
{
//    char status;
    uint16_t  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 	PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
   // status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    return MI_OK;
}

unsigned char IC_Test(void)
{

    unsigned char find=0xaa;
    unsigned char ret;   
    
    ret = PcdRequest(0x52,aIDType);//寻卡
    if(ret != 0x26)
        ret = PcdRequest(0x52,aIDType);
    if(ret != 0x26)
        find = 0xaa;
    if((ret == 0x26)&&(find == 0xaa))
    {
        //if(PcdAnticoll(ucTagType) == 0x26)//防冲撞
        {
            // HalUARTWrite(0,ucTagType,4);
            find = 0x00;
            return 1;
        }
    }
    
    return 0;

}

void rc522_init(void)
{
    unsigned char i;
    PcdReset();
    M500PcdConfigISOType('A');
    for(i=0;i<6;i++)
    {
      rfbuf[i] = 0;
    }
}

void iccardcode()
{	     
  unsigned char cmd;
	unsigned char status;
	
	cmd = RevBuffer[0];
	switch(cmd)
 	{
 		// Halt the card     //??????
		case 1:     
			status= PcdHalt();;			
			RevBuffer[0]=1;
			RevBuffer[1]=status;
			break;			
			// Request,Anticoll,Select,return CardType(2 bytes)+CardSerialNo(4 bytes)
		case 2:     
                // find select card,   return card type(2 bytes)
			status= PcdRequest(RevBuffer[1],&RevBuffer[2]);
			if(status!=0)
			{
				status= PcdRequest(RevBuffer[1],&RevBuffer[2]);
				if(status!=0)				
				{
					RevBuffer[0]=1;	
					RevBuffer[1]=status;
					break;
				}
			}  
      aIDType[0] = RevBuffer[2];
      aIDType[1] = RevBuffer[3];
			RevBuffer[0]=3;	
			RevBuffer[1]=status;
			break;
			// avoid collision ,return card id(4 bytes)MLastSelectedSnr
		case 3:      
			status = PcdAnticoll(&RevBuffer[2]);
			if(status!=0)
			{
				RevBuffer[0]=1;	
				RevBuffer[1]=status;
				break;
			}
			//mymemcpy(MLastSelectedSnr,&RevBuffer[2],4);
      memcpy(MLastSelectedSnr,&RevBuffer[2],4);
			RevBuffer[0]=5;
			RevBuffer[1]=status;
			break;	
		case 4:		                    // Select Card
			status=PcdSelect(MLastSelectedSnr);
			if(status!=MI_OK)
			{
				RevBuffer[0]=1;	
				RevBuffer[1]=status;
				break;
			}
			RevBuffer[0]=3;
			RevBuffer[1]=status;			
			break;
		case 5:	    // Key loading into the MF RC500's EEPROM
      status = PcdAuthState(RevBuffer[1], RevBuffer[3], DefaultKey, MLastSelectedSnr);//Check password
			RevBuffer[0]=1;
			RevBuffer[1]=status;			
			break;							
		case 6: 
			RevBuffer[0]=1;
			RevBuffer[1]=status;			
			break;				
		case 7:     
    	RevBuffer[0]=1;
			RevBuffer[1]=status;			
			break;
		case 8:     // Read the mifare card
		            // ??
			status=PcdRead(RevBuffer[1],&RevBuffer[2]);
			if(status==0)
			{RevBuffer[0]=17;}
			else
			{RevBuffer[0]=1;}
			RevBuffer[1]=status;			
			break;
		case 9:     // Write the mifare card
		            // ??  ????
			status=PcdWrite(RevBuffer[1],&RevBuffer[2]);
			RevBuffer[0]=1;
			RevBuffer[1]=status;			
			break;
/*
		case 10:
      PcdValue(RevBuffer[1],RevBuffer[2],&RevBuffer[3]);
			RevBuffer[0]=1;	
			RevBuffer[1]=status;
			break;
		case 12:    // Parameter setting
		  PcdBakValue(RevBuffer[1], RevBuffer[2]);
			RevBuffer[0]=1;	//contact
			RevBuffer[1]=0;
			break;	
*/    
	}
}

/////////////////////////////////////////
// input MLastSelectedSnr[]
// output DefaultKey[]
////////////////////////////////////////
void RFID_passwords(void)
{
  DefaultKey[0] = MLastSelectedSnr[3]%2+MLastSelectedSnr[0]/2;   //密码算法
	DefaultKey[1] = MLastSelectedSnr[2]/2+MLastSelectedSnr[0]%2;
	DefaultKey[2] = MLastSelectedSnr[1]%2+MLastSelectedSnr[3]/2;
	DefaultKey[3] = MLastSelectedSnr[2]/2+MLastSelectedSnr[3]/2;
	DefaultKey[4] = MLastSelectedSnr[0]/2+MLastSelectedSnr[1]/2;
	DefaultKey[5] = MLastSelectedSnr[2]%2+MLastSelectedSnr[1]/2;
            
	RevBuffer[17] = MLastSelectedSnr[0]+MLastSelectedSnr[1]+MLastSelectedSnr[2]+MLastSelectedSnr[3];
  // A key          
	RevBuffer[2] = DefaultKey[RevBuffer[17]%6];			   
	RevBuffer[3] = DefaultKey[(RevBuffer[17]+1)%6];
	RevBuffer[4] = DefaultKey[(RevBuffer[17]+2)%6];
	RevBuffer[5] = DefaultKey[(RevBuffer[17]+3)%6];
	RevBuffer[6] = DefaultKey[(RevBuffer[17]+4)%6];
	RevBuffer[7] = DefaultKey[(RevBuffer[17]+5)%6];
	
	DefaultKey[0] = RevBuffer[2];  
	DefaultKey[1] = RevBuffer[3];
	DefaultKey[2] = RevBuffer[4];
	DefaultKey[3] = RevBuffer[5];
	DefaultKey[4] = RevBuffer[6];
	DefaultKey[5] = RevBuffer[7];
}
//rfid_read
unsigned int rfid_read(void)
{
	unsigned int status=0;
 // IWDG_ReloadCounter();
  
  RevBuffer[0] = 0x02; 
  RevBuffer[1] = 0x26;
  iccardcode();
  
  if((RevBuffer[0]==3)&&(RevBuffer[2]==0x04)) 
  {
    RevBuffer[0] = 0x03;
    iccardcode();
    if(RevBuffer[0]==5) 
    {
      RevBuffer[0] = 0x04;
      iccardcode();
      if(RevBuffer[0]==3)
			{	
          RevBuffer[0] = 0x01;
          iccardcode();
		  memmove(rfid_info.rfid_card_id,MLastSelectedSnr,4);
		  printf("get card id =%02x%02x%02x%02x\n",rfid_info.rfid_card_id[0],rfid_info.rfid_card_id[1],rfid_info.rfid_card_id[2],rfid_info.rfid_card_id[3]);
		  status=1;
         }	
      
    }
  }
  
  PcdAntennaOff();	  
  return status;
}


//////////////////////////////////////////////////////////////////
// read RFID infiormation 
// in: 
// mode : 1: school
//        2: engineer
// out:
// status 0: error
//        1: school card
//        2: engineer card
/////////////////////////////////////////////////////////////////
unsigned int RFID_user_read()
{
  unsigned int status=0;
 // IWDG_ReloadCounter();
  
  RevBuffer[0] = 0x02; 
  RevBuffer[1] = 0x26;
  iccardcode();
  
  if((RevBuffer[0]==3)&&(RevBuffer[2]==0x04)) 
  {
    RevBuffer[0] = 0x03;
    iccardcode();
    if(RevBuffer[0]==5) 
    {
      RevBuffer[0] = 0x04;
      iccardcode();
      if(RevBuffer[0]==3)
			{	
        RFID_passwords();
        // check passwords
        RevBuffer[0] = 0x05;
        RevBuffer[1] = 0x60;
        RevBuffer[2] = 0x01;
        RevBuffer[3] = rfid_info.sector<<2;//4;//60;//0x04;// data block addr
        iccardcode();
        if(RevBuffer[1]==0) 
        {	
          // read card id
          // RevBuffer[2]~RevBuffer[17]
					RevBuffer[0] = 0x08;
					RevBuffer[1] = rfid_info.sector<<2;//4;//60;//0x04;// data block addr
					iccardcode();       
          //IWDG_ReloadCounter();
					if(RevBuffer[0]==17)
					{            
            // read school and engineer number to readbuff[]            
            // School card
           // memset(get_from_rfid,0,10);
            status = 1;     
			//memmove(get_from_rfid,&RevBuffer[8],10);
			memmove(rfid_info.rfid_card_projector_id_read,&RevBuffer[8],10);
			printf("get card id 2 = %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",rfid_info.rfid_card_projector_id_read[0],rfid_info.rfid_card_projector_id_read[1],rfid_info.rfid_card_projector_id_read[2],rfid_info.rfid_card_projector_id_read[3],rfid_info.rfid_card_projector_id_read[4],rfid_info.rfid_card_projector_id_read[5],rfid_info.rfid_card_projector_id_read[6],rfid_info.rfid_card_projector_id_read[7],rfid_info.rfid_card_projector_id_read[8],rfid_info.rfid_card_projector_id_read[9]);
					}
          RevBuffer[0] = 0x01;
          iccardcode();
         }	
      }
    }
  }
  //Delay10ms = 0;
  //while(Delay10ms<=1) 
  //{
  //  IWDG_ReloadCounter();
  //}
  PcdAntennaOff();	  
  return status;
}

void RFID_data_write(unsigned char mode)
{
  // Write passwords and school codes to
  // section1 , data block0
	RevBuffer[0] = 0x09;
	RevBuffer[1] = RFIDSection<<2;//4;//60;//0x04;  // data block addr
            
  RevBuffer[2] = DefaultKey[0];			   
	RevBuffer[3] = DefaultKey[1];
	RevBuffer[4] = DefaultKey[2];
	RevBuffer[5] = DefaultKey[3];
	RevBuffer[6] = DefaultKey[4];
	RevBuffer[7] = DefaultKey[5];  
  
  // read school and engineer number to readbuff[]

  memcpy(RevBuffer+8,readbuff+1,readbuff[0]);  
  iccardcode();	
}


//////////////////////////////////////////////////////////////////
// write RFID infiormation 
// in: 
// mode : 1: school
//        2: engineer
// out:
// status 0: fail
//        1: success
/////////////////////////////////////////////////////////////////
unsigned char RFID_user_write(unsigned char mode)
{
	unsigned char status=0,i;
	// Enable antenna
	PcdAntennaOn();  
	//delay_10ms(5);
  usleep(1000*5);
 
  // Find card type 2 byte
	RevBuffer[0] = 0x02; 
	RevBuffer[1] = 0x26;
	iccardcode();
	if((RevBuffer[0]!=3)||(RevBuffer[2]!=0x04)) ;
	else
	{
    // Read card id 4 bytes MLastSelectedSnr   
		RevBuffer[0] = 0x03;
		iccardcode();
		if(RevBuffer[0]==5)
		{	
      // select card
			RevBuffer[0] = 0x04;
			iccardcode();
			if(RevBuffer[0]==3)
			{										
				for(i=0;i<6;i++) 
        {
          // default passwords
          DefaultKey[i]=0xff;
        }
        
				RevBuffer[0] = 0x05;  // check passwords cmd
				RevBuffer[1] = 0x60;  // check A KEY mode
				RevBuffer[2] = 0x01;
				RevBuffer[3] = (RFIDSection<<2) + 3;//;//63;//0x07;  // addr : secton 1 ,control block3
				iccardcode();					
       
				if(RevBuffer[1]!=0x00) 
				{ 
          // check error 
          // to indicate that the card has been written passwords.
          RevBuffer[0] = 0x02; 
					RevBuffer[1] = 0x26;
					iccardcode();
					RevBuffer[0] = 0x03;
					iccardcode();
					RevBuffer[0] = 0x04;
					iccardcode();
          
          RFID_passwords();        
            
          // check passwords secton1 data block0
					RevBuffer[0] = 0x05;
					RevBuffer[1] = 0x60;
					RevBuffer[2] = 0x01;
					RevBuffer[3] = RFIDSection<<2;//4;//60;//  0x04; //data block addr
					iccardcode();
            
          // Write passwords and school codes to
          // section1 , data block0
          RFID_data_write(mode);
			  }
			  else
				{          
          // new card check ok , write brxy passwords and school codes
          RFID_passwords();
          // control code
          RevBuffer[8] = 0xff;
					RevBuffer[9] = 0x07;
					RevBuffer[10] = 0x80;
					RevBuffer[11] = 0x69;
            
          // B key
					RevBuffer[12] = RevBuffer[2];			   
					RevBuffer[13] = RevBuffer[3];
					RevBuffer[14] = RevBuffer[4];
					RevBuffer[15] = RevBuffer[5];
					RevBuffer[16] = RevBuffer[6];
					RevBuffer[17] = RevBuffer[7];
          // wreite section1 control block3
					RevBuffer[0] = 0x09;
					RevBuffer[1] = (RFIDSection<<2)+3;//7;//63;//0x07;// control block addr
					iccardcode();
            
          // check passwords secton1 data block0
					RevBuffer[0] = 0x05;
					RevBuffer[1] = 0x60;
					RevBuffer[2] = 0x01;
					RevBuffer[3] = RFIDSection<<2;//4;//60;//  0x04; //data block addr
					iccardcode();
            
          // Write passwords and school codes to
          // section1 , data block0
					RFID_data_write(mode);
				}
				
				for(i=0;i<3;i++)
        {
          if(RFID_user_read() > 0)
          {
            status=1; 
            break;
          }
        }
			}
		}
	}
	PcdAntennaOff();
	return status;
}

unsigned char RFID_check(void)
{
	unsigned char status = 0;
	PcdAntennaOn();
	//delay_10ms(5);
  usleep(5000);
	RevBuffer[0] = 0x02; 
	RevBuffer[1] = 0x26;
	iccardcode();
  if((RevBuffer[0]!=3)||(RevBuffer[2]!=0x04))
  {
    RevBuffer[0] = 0x02; 
    RevBuffer[1] = 0x26;
    iccardcode();
  }
  
	if((RevBuffer[0]==3)&&(RevBuffer[2]==0x04))
  {	
		RevBuffer[0] = 0x03;
		iccardcode();
		if(RevBuffer[0]==5)
		{	
      if(RFID_user_read())
      {
        // Router mode
        status = 1;
      }
    }
  }		
	return status;
}

/*************************************************************************************
i2c转gpio--TCA9555PWR芯片控制

*************************************************************************************/

int tca_set_bit_old(int bit)
{
	unsigned char to=0;
	if(i2c_read(i2c_address_tca,tca_info.port0_out,&to)<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	switch(bit)
		{
			case 0:
				to|=(1<<tca_info.mation_out);//系统电源 低有效
				break;
			case 1:
				to|=(1<<tca_info.pc_out);
				break;
			case 2:
				to|=(1<<tca_info.mation_status);
				break;
			case 3:
				to|=(1<<tca_info.ep_status);
				break;
			case 4:
				to|=(1<<tca_info.projector_out);//x86电源 低有效
				break;
			case 5:
				to|=(1<<tca_info.lock_enable);
				break;
			case 6:
				to|=(1<<tca_info.lock_out);
				break;
			case 7:
				to|=(1<<tca_info.projector_status);
				break;
			default:
				return -1;
				break;
		}
	if(i2c_write(i2c_address_tca,tca_info.port0_out,to)<0)
		{
			printf("can not write to tca\n");
			return -1;
		}
	return 1;
}

int tca_set_bit(int bit)
{
	unsigned int io_register;
	switch(bit)
		{
			case 0:
				//to|=(1<<tca_info.mation_out);//系统电源 低有效
				io_register=mcu_i2c_info.x84_power;//低电平有效
				break;
			case 1:
				//to|=(1<<tca_info.pc_out);
				io_register=mcu_i2c_info.x86open_off;
				break;
			case 2:
				//to|=(1<<tca_info.mation_status);
				io_register=mcu_i2c_info.system_status;
				break;
			case 3:
				//to|=(1<<tca_info.ep_status);
				io_register=mcu_i2c_info.chanel_status;
				break;
			case 4:
				//to|=(1<<tca_info.projector_out);//x86电源 低有效
				io_register=mcu_i2c_info.projector_power;//低电平有效
				break;
			case 5:
				//to|=(1<<tca_info.lock_enable);
				io_register=mcu_i2c_info.lock_enable;
				break;
			case 6:
				//to|=(1<<tca_info.lock_out);
				io_register=mcu_i2c_info.lock_out;
				break;
			case 7:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.projector_status;
				break;
			case 8:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.ope_power;//低电平有效
				break;
			case 9:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.ep_power;//高电平有效
				break;
			case 10:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.buzzer;
				break;
			case 11:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.reset_ep92;
				break;
			case 12:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.lock_status;
				break;
			case 13:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.x86open_off_config;
				break;
			case 14:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.lock_out_config;
				break;
			case 15:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.lock_enable_config;
				break;
			case 16:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.ope_power_config;
				break;
			case 17:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.x84_power_config;
				break;
			case 18:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.projector_power_config;
				break;
			case 19:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.ep_power_config;
				break;
			case 20:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.buzzer_config;
				break;
			case 21:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.reset_ep92_config;
				break;
			case 22:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.chanel_status_config;
				break;
			case 23:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.projector_status_config;
				break;
			case 24:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.system_status_config;
				break;
			case 25:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.lock_status_config;
				break;
			case 26:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.system_status_init;//不会再没启动系统的时候闪烁，以此判断是否需要初始化系统设备
				break;
			case 27:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.system_status_init_config;
				break;
			default:
				return -1;
				break;
		}
	if(i2c_write(i2c_address_mcu,io_register,1)<0)
		{
			printf("can not tca_set_bit \n");
			return -1;
		}
	return 1;
}




int clean_tca_bit(int bit)
{
	unsigned int io_register;
	switch(bit)
		{
			case 0:
				//to|=(1<<tca_info.mation_out);//系统电源 低有效
				io_register=mcu_i2c_info.x84_power;//低电平有效
				break;
			case 1:
				//to|=(1<<tca_info.pc_out);
				io_register=mcu_i2c_info.x86open_off;
				break;
			case 2:
				//to|=(1<<tca_info.mation_status);
				io_register=mcu_i2c_info.system_status;
				break;
			case 3:
				//to|=(1<<tca_info.ep_status);
				io_register=mcu_i2c_info.chanel_status;
				break;
			case 4:
				//to|=(1<<tca_info.projector_out);//x86电源 低有效
				io_register=mcu_i2c_info.projector_power;//低电平有效
				break;
			case 5:
				//to|=(1<<tca_info.lock_enable);
				io_register=mcu_i2c_info.lock_enable;
				break;
			case 6:
				//to|=(1<<tca_info.lock_out);
				io_register=mcu_i2c_info.lock_out;
				break;
			case 7:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.projector_status;
				break;
			case 8:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.ope_power;//低电平有效
				break;
			case 9:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.ep_power;//高电平有效
				break;
			case 10:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.buzzer;
				break;
			case 11:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.reset_ep92;
				break;
			case 12:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.lock_status;
				break;
			case 13:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.x86open_off_config;
				break;
			case 14:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.lock_out_config;
				break;
			case 15:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.lock_enable_config;
				break;
			case 16:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.ope_power_config;
				break;
			case 17:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.x84_power_config;
				break;
			case 18:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.projector_power_config;
				break;
			case 19:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.ep_power_config;
				break;
			case 20:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.buzzer_config;
				break;
			case 21:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.reset_ep92_config;
				break;
			case 22:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.chanel_status_config;
				break;
			case 23:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.projector_status_config;
				break;
			case 24:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.system_status_config;
				break;
			case 25:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.lock_status_config;
				break;
			case 26:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.system_status_init;
				break;
			case 27:
				//to|=(1<<tca_info.projector_status);
				io_register=mcu_i2c_info.system_status_init_config;
				break;
			default:
				return -1;
				break;
		}
	if(i2c_write(i2c_address_mcu,io_register,0)<0)
		{
			printf("can not clean_tca_bit \n");
			return -1;
		}
	return 1;
}

int clean_tca_bit_old(int bit)
{
	unsigned char to=0;
	if(i2c_read(i2c_address_tca,tca_info.port0_out,&to)<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	switch(bit)
		{
			case 0:
				to&=(~(1<<tca_info.mation_out));
				break;
			case 1:
				to&=(~(1<<tca_info.pc_out));
				break;
			case 2:
				to&=(~(1<<tca_info.mation_status));
				break;
			case 3:
				to&=(~(1<<tca_info.ep_status));
				break;
			case 4:
				to&=(~(1<<tca_info.projector_out));
				break;
			case 5:
				to&=(~(1<<tca_info.lock_enable));
				break;
			case 6:
				to&=(~(1<<tca_info.lock_out));
				break;
			case 7:
				to&=(~(1<<tca_info.projector_status));
				break;
			default:
				return -1;
				break;
		}
	if(i2c_write(i2c_address_tca,tca_info.port0_out,to)<0)
		{
			printf("can not write to tca\n");
			return -1;
		}
	return 1;
}


int pc_open_open(void)
{
	if(device_status.pc==0)
		{
			pthread_mutex_lock(&i2c_mutex);
			tca_set_bit(1);
			pthread_mutex_unlock(&i2c_mutex);
			usleep(3000000);
			pthread_mutex_lock(&i2c_mutex);
			clean_tca_bit(1);
			usleep(10000);
			pthread_mutex_unlock(&i2c_mutex);
			device_status.pc=1;
		}
	
}
int pc_close_close(void)
{
	if(device_status.pc==1)
		{
			tca_set_bit(1);
			usleep(3000000);
			clean_tca_bit(1);
			usleep(10000);
			device_status.pc=0;
		}
	
}
int pc_open(void)
{
	int flag=0;
	pthread_mutex_lock(&action_mutex);
	if((device_status.k4==1)&&(device_status.machine_flag==0)&&(device_status.pc_flag==0)&&(device_status.pc==0))
		{
			flag=1;
			device_status.pc_flag=1;
		}
	pthread_mutex_unlock(&action_mutex);
	if(flag==1)
		{
			printf("open pc\n");
			local_alsa_play_inaction(2);
			pthread_mutex_lock(&i2c_mutex);
			
			tca_set_bit(1);
			usleep(3000000);
			clean_tca_bit(1);
			usleep(10000);
			pthread_mutex_unlock(&i2c_mutex);
			ep_passage_switch(device_status.ep);
			device_status.pc=1;
			device_status.pc_flag=0;
		}
	
}
int pc_close(void)
{
	int flag=0;
	pthread_mutex_lock(&action_mutex);
	if((device_status.k4==1)&&(device_status.machine_flag==0)&&(device_status.pc_flag==0)&&(device_status.pc==1))
		{
			flag=1;
			device_status.pc_flag=1;
		}
	pthread_mutex_unlock(&action_mutex);
	if(flag==1)
		{
			printf("close pc\n");
			local_alsa_play_inaction(14);
			pthread_mutex_lock(&i2c_mutex);
			tca_set_bit(1);
			usleep(3000000);
			clean_tca_bit(1);
			usleep(10000);
			pthread_mutex_unlock(&i2c_mutex);
			device_status.pc=0;
			device_status.pc_flag=0;
		}
	
}






int projector_open(void)
{
	unsigned char op[18];
	char ope[3]={0};
	int i=0;
	int length=0;
	int flag=0;
	pthread_mutex_lock(&action_mutex);
	if((device_status.k4==1)&&(device_status.machine_flag==0)&&(device_status.projector_flag==0)&&(device_status.projector==0))
		{
			flag=1;
			device_status.projector_flag=1;
		}
	pthread_mutex_unlock(&action_mutex);

	
	memset(op,0,18);
	length=strlen(projector_info.start_code);
	if((length==0)||((length%2)>0))
		{
			return -1;
		}
	if(flag==1)
		{
			//device_status.projector_flag=1;
			local_alsa_play_inaction(5);
			printf("open projector\n");
			pthread_mutex_lock(&i2c_mutex);
			//tca_set_bit(4);
			tca_set_bit(7);
			usleep(10000);
			pthread_mutex_unlock(&i2c_mutex);
			sleep(projector_info.start_delay);
			for(i=0;i<(length/2);i++)
				{
					ope[0]=projector_info.start_code[i*2];
					ope[1]=projector_info.start_code[i*2+1];
					op[i]=String2hex(ope);
					printf("send projector code  = %02x\n",op[i]);
				}
			if(write(projector_info.serial_fd,op,(length/2))!=(length/2))
				{
					printf("send projector open false\n");
					tcflush(projector_info.serial_fd, TCOFLUSH);
				}
			else
				{
					printf("send projector open success\n");
					sleep(projector_info.interval);
					device_status.projector=1;
				}
			
			device_status.projector_flag=0;
		}
	return 1;
}
int projector_close(void)
{
	
	unsigned char op[18];
	char ope[3]={0};
	int i=0;
	int length=0;
	int flag=0;
	pthread_mutex_lock(&action_mutex);
	if((device_status.k4==1)&&(device_status.machine_flag==0)&&(device_status.projector_flag==0)&&(device_status.projector==1))
		{
			flag=1;
			device_status.projector_flag=1;
		}
	pthread_mutex_unlock(&action_mutex);
	memset(op,0,18);
	length=strlen(projector_info.shutdown_code);
	if((length==0)||((length%2)>0))
		{
			return -1;
		}
	if(flag==1)
		{

			printf("close projector\n");
		
			local_alsa_play_inaction(6);
			for(i=0;i<(length/2);i++)
				{
					ope[0]=projector_info.shutdown_code[i*2];
					ope[1]=projector_info.shutdown_code[i*2+1];
					op[i]=String2hex(ope);
					printf("send projector code  = %02x\n",op[i]);
				}
			if(write(projector_info.serial_fd,op,(length/2))!=(length/2))
				{
					printf("send projector close false\n");
					tcflush(projector_info.serial_fd, TCOFLUSH);
				}
			else
				{
					printf("send projector close success\n");
					sleep(projector_info.shutdown_delay);
					device_status.projector=0;
					pthread_mutex_lock(&i2c_mutex);
					//clean_tca_bit(4);
					clean_tca_bit(7);
					usleep(10000);
					pthread_mutex_unlock(&i2c_mutex);
				}
			
			device_status.projector_flag=0;
		}
	return 1;
}
int projector_open_open(void)
{
	unsigned char op[18];
	char ope[3]={0};
	int i=0;
	int length=0;
	memset(op,0,18);
	length=strlen(projector_info.start_code);
	if((length==0)||((length%2)>0))
		{
			return -1;
		}
	//tca_set_bit(4);
	sleep(projector_info.start_delay);
	if(device_status.projector==0)
		{
			for(i=0;i<(length/2);i++)
				{
					ope[0]=projector_info.start_code[i*2];
					ope[1]=projector_info.start_code[i*2+1];
					op[i]=String2hex(ope);
				}
			if(write(projector_info.serial_fd,op,(length/2))!=(length/2))
				{
					tcflush(projector_info.serial_fd, TCOFLUSH);
				}
			else
				{
					//sleep(projector_info.interval);
					device_status.projector=1;
					pthread_mutex_lock(&i2c_mutex);
					tca_set_bit(7);
					usleep(10000);
					pthread_mutex_unlock(&i2c_mutex);
				}
			
			//device_status.projector_flag=0;
		}
		
	return 1;
}
int projector_close_close(void)
{
	unsigned char op[18];
	char ope[3]={0};
	int i=0;
	int length=0;
	memset(op,0,18);
	length=strlen(projector_info.shutdown_code);
	if((length==0)||((length%2)>0))
		{
			return -1;
		}
	if(device_status.projector==1)
		{
			for(i=0;i<(length/2);i++)
				{
					ope[0]=projector_info.shutdown_code[i*2];
					ope[1]=projector_info.shutdown_code[i*2+1];
					op[i]=String2hex(ope);
				}
			if(write(projector_info.serial_fd,op,(length/2))!=(length/2))
				{
					printf("write to projecter from seral false\n");
					tcflush(projector_info.serial_fd, TCOFLUSH);
				}
			else
				{
					sleep(projector_info.shutdown_delay);
					device_status.projector=0;
					
				}
			
		}
		
	return 1;
}


int ep_change(void)
{
	int flag=0;
	pthread_mutex_lock(&action_mutex);
	if((device_status.k4==1)&&(device_status.machine_flag==0))
		{
			flag=1;
			
		}
	pthread_mutex_unlock(&action_mutex);
	if(flag==1)
		{
			printf("switch ep HDMI\n");
			if(device_status.ep==1)
				{
					device_status.ep=0;
					printf("switch ep HDMI to %d\n",device_status.ep);
					local_alsa_play_inaction(10);
					
					pthread_mutex_lock(&i2c_mutex);
					tca_set_bit(3);
					usleep(10000);
					pthread_mutex_unlock(&i2c_mutex);
				}
			else
				{
					device_status.ep=1;
					printf("switch ep HDMI to %d\n",device_status.ep);
					local_alsa_play_inaction(9);
					
					pthread_mutex_lock(&i2c_mutex);
					
					clean_tca_bit(3);
					usleep(10000);
					pthread_mutex_unlock(&i2c_mutex);
				}
			ep_passage_switch(device_status.ep);
		}
}

int ep_switch_to_in(void)
{
	if(device_status.ep==1)
		{
			device_status.ep=0;
			printf("switch ep HDMI to %d\n",device_status.ep);
			local_alsa_play_inaction(10);
					
			pthread_mutex_lock(&i2c_mutex);
			tca_set_bit(3);
			usleep(10000);
			pthread_mutex_unlock(&i2c_mutex);
			ep_passage_switch(device_status.ep);
		}
	
}

int ep_switch_to_out(void)
{
	if(device_status.ep==0)
		{
			device_status.ep=1;
			printf("switch ep HDMI to %d\n",device_status.ep);
			local_alsa_play_inaction(9);
					
			pthread_mutex_lock(&i2c_mutex);
			//tca_set_bit(3);
			clean_tca_bit(3);
			usleep(10000);
			pthread_mutex_unlock(&i2c_mutex);
			ep_passage_switch(device_status.ep);
		}
	
}


int ep_open(void)
{
	//pthread_mutex_lock(&i2c_mutex);
	ep_passage_onoff(1);
	ep_passage_switch(device_status.ep);
	//ep_passage_switch(1);
	//sleep(15);
	//ep_passage_switch(0);
	//sleep(3);
	//ep_passage_switch(1);
	//sleep(3);
	//ep_passage_switch(0);
	
	
	tca_set_bit(3);
	usleep(10000);
	//pthread_mutex_unlock(&i2c_mutex);
}

int stand_open(void)
{
	int flag=0;
	pthread_mutex_lock(&action_mutex);
	if((device_status.k4==1)&&(device_status.machine_flag==0)&&(device_status.stand_lock==0))
		{
			flag=1;
			
		}
	pthread_mutex_unlock(&action_mutex);
	if(flag==1)
		{
			printf("stand open\n");
			local_alsa_play_inaction(11);
			pthread_mutex_lock(&i2c_mutex);
			tca_set_bit(6);
			usleep(10000);
			tca_set_bit(5);
			usleep(2000);
	
			clean_tca_bit(5);
			usleep(10000);
			tca_set_bit(12);
			usleep(10000);
			device_status.stand_lock=1;
			pthread_mutex_unlock(&i2c_mutex);
		}
}

int stand_close(void)
{
	int flag=0;
	pthread_mutex_lock(&action_mutex);
	if((device_status.k4==1)&&(device_status.machine_flag==0)&&(device_status.stand_lock==1))
		{
			flag=1;
			
		}
	pthread_mutex_unlock(&action_mutex);
	if(flag==1)
		{
			printf("stand close\n");
			local_alsa_play_inaction(12);
			pthread_mutex_lock(&i2c_mutex);
			clean_tca_bit(6);
			usleep(10000);
			tca_set_bit(5);
			usleep(2000);
	
			clean_tca_bit(5);
			usleep(10000);
			clean_tca_bit(12);
			usleep(10000);
			device_status.stand_lock=0;
			pthread_mutex_unlock(&i2c_mutex);
		}
}
int stand_open_open(void)
{
	pthread_mutex_lock(&i2c_mutex);
	tca_set_bit(6);
	usleep(10000);
	tca_set_bit(5);
	pthread_mutex_unlock(&i2c_mutex);
	usleep(2000000);
	pthread_mutex_lock(&i2c_mutex);
	clean_tca_bit(5);
	usleep(10000);
	tca_set_bit(12);
	usleep(10000);
	pthread_mutex_unlock(&i2c_mutex);
	device_status.stand_lock=1;
}
int stand_close_close(void)
{
	clean_tca_bit(6);
	usleep(10000);
	tca_set_bit(5);
	usleep(2000);
	
	clean_tca_bit(5);
	usleep(10000);
	clean_tca_bit(12);
	usleep(10000);
	device_status.stand_lock=0;
}
int volume_mute(void)
{
	printf("volume mute\n");
	pthread_mutex_lock(&i2c_mutex);
	i2c_write(i2c_address_wm,0x2b,0xdf);//打开通道 不静音
	pthread_mutex_unlock(&i2c_mutex);
	volume_3105.mute=1;
}

int volume_mute_disable(void)
{
	printf("volume mute\n");
	pthread_mutex_lock(&i2c_mutex);
	i2c_write(i2c_address_wm,0x2b,0x1f);//打开通道 不静音
	pthread_mutex_unlock(&i2c_mutex);
	volume_3105.mute=0;
}

int volume_add(void)
{
	if(volume_3105.mute==1)
		{
			printf("mute=8");
			pthread_mutex_lock(&i2c_mutex);
			i2c_write(i2c_address_wm,0x2b,0x1f);//打开通道 不静音
			//i2c_write(i2c_address_wm,0x07,wm8776_volume_table[volume_3105.volume]);
			pthread_mutex_unlock(&i2c_mutex);
			volume_3105.mute=0;
		}
	if(volume_3105.volume==8)
		{
			printf("volume = 8\n");
		}
	else if((volume_3105.volume<8)&&(volume_3105.volume>=0))
		{
			printf("volume add\n");
			
			volume_3105.volume++;
			pthread_mutex_lock(&i2c_mutex);
			//VOLUME_TLV320(volume_3105.volume);
			i2c_write(i2c_address_wm,0x1c,wm8776_volume_table[volume_3105.volume]);
			pthread_mutex_unlock(&i2c_mutex);
			local_alsa_play_inaction(3);
		}
	return 1;
}
int volume_discrease(void)
{
	if(volume_3105.volume==0)
		{
			if(volume_3105.mute==1)
				{
					pthread_mutex_lock(&i2c_mutex);
					i2c_write(i2c_address_wm,0x2b,0x1f);//打开通道 不静音
					pthread_mutex_unlock(&i2c_mutex);
					volume_3105.mute=0;
				}
			printf("volume = 0\n");
		}
	else if((volume_3105.volume<9)&&(volume_3105.volume>0))
		{
			
			
			if(volume_3105.mute==1)
				{
					pthread_mutex_lock(&i2c_mutex);
					i2c_write(i2c_address_wm,0x2b,0x1f);//打开通道 不静音
					pthread_mutex_unlock(&i2c_mutex);
					volume_3105.mute=0;
				}
			printf("volume discrease\n");
			
			volume_3105.volume--;
			pthread_mutex_lock(&i2c_mutex);
			//VOLUME_TLV320(volume_3105.volume);
			i2c_write(i2c_address_wm,0x1c,wm8776_volume_table[volume_3105.volume]);
			pthread_mutex_unlock(&i2c_mutex);
			local_alsa_play_inaction(4);
		}
	return 1;
}
int init_wm(void)
{
	i2c_write(i2c_address_wm,0x1a,0x00);//开耳机
	i2c_write(i2c_address_wm,0x19,0x22);//设为主模式--i2c
	i2c_write(i2c_address_wm,0x2b,0x1f);//打开通道 不静音
	i2c_write(i2c_address_wm,0x2c,0x04);//接通耳机线路
	i2c_write(i2c_address_wm,0x26,0x01);//噪音门控制
	i2c_write(i2c_address_wm,0x1c,wm8776_volume_table[volume_3105.volume]);//初始化音量
	return 1;
}
int init_tas(void)
{
	//i2c_write(i2c_address_tas,0x05,0x00);//打开扬声器
	i2c_write(i2c_address_tas,0x1b,0x00);//芯片初始化
	usleep(400000);
	i2c_write(i2c_address_tas,0x50,0x04);//设置频率
	usleep(400000);
	i2c_write(i2c_address_tas,0x05,0x00);//开功放
	usleep(400000);
	i2c_write(i2c_address_tas,0x07,0x48);//设置声音
	//i2c_write(i2c_address_tas,0x07,0x35);
	//i2c_write(i2c_address_tas,0x07,0x35);
	//i2c_write(i2c_address_tas,0x05,0x00);

	
	return 1;
}
int init_wm_tas_rfid_tca_old(void)
{
	unsigned char to=0x06;
	unsigned char too=0;
	unsigned int k=0;

	unsigned char to0=0;
	unsigned char to1=0;
	unsigned char chanel;
	//printf("1\n");
	if(i2c_read(i2c_address_tca,tca_info.port0_out,&to0)<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	//printf("2\n");
	if(i2c_read(i2c_address_tca,tca_info.port0_configuration,&to1)<0)
			{
				printf("can not read tca\n");
				return -1;
			}
	//printf("3\n");

	if((to1==0)&&((to0&2)==0)&&((to0&4)==0))
		{
			if(i2c_read(i2c_address_ep,0x11,&chanel)<0)
			{
				printf("can not read tca\n");
				return -1;
			}
			chanel&=0x0f;
			if(chanel==1)
				{
					device_status.ep=0;
				}
			else
				{
					device_status.ep=1;
				}

			
			led_glint.led_standy_flag=2;

			
			if((to0&16)>0)
				{
					device_status.projector=1;
				}
			else
				{
					device_status.projector=0;
				}
			device_status.k4=1;
			i2c_write(i2c_address_wm,0x2b,0x1f);//打开通道 不静音
			i2c_write(i2c_address_wm,0x1c,wm8776_volume_table[volume_3105.volume]);//初始化音量
			printf("it is restart,no need to init outer device\n");
			
		}
	else
		{
			//printf("4\n");
			init_tas();
			//printf("5\n");
			init_wm();
			//printf("6\n");
			//ep_open();
			//printf("7\n");
			rc522_init();
			//printf("8\n");

			//初始化tca芯片
			i2c_write(i2c_address_tca,tca_info.port0_out,to);
			i2c_write(i2c_address_tca,tca_info.port0_configuration,too);
			ep_open();
		}





	
	//Init_tlv320();
	/*init_tas();
	init_wm();
	ep_open();
	rc522_init();

	//初始化tca芯片
	i2c_write(i2c_address_tca,tca_info.port0_out,to);
	i2c_write(i2c_address_tca,tca_info.port0_configuration,too);*/
	/*for(k=0;k<4;k++)
		{
			usleep(300);
			tca_set_bit(2);
			usleep(300);
			clean_tca_bit(2);
		}*/
	return 1;
}

int init_wm_tas_rfid_tca(void)
{
	unsigned char to=0x06;
	unsigned char too=0;
	unsigned int k=0;
	int flag=0;
	int i;

	unsigned char to0[15]={0};
	unsigned char to1[10]={0};

	//printf("1\n");
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.x86open_off_config,&to0[0])<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.lock_out_config,&to0[1])<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.lock_enable_config,&to0[2])<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.ope_power_config,&to0[3])<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.x84_power_config,&to0[4])<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.projector_power_config,&to0[5])<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.ep_power_config,&to0[6])<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.buzzer_config,&to0[7])<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.reset_ep92_config,&to0[8])<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.chanel_status_config,&to0[9])<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.projector_status_config,&to0[10])<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.system_status_config,&to0[11])<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.lock_status_config,&to0[12])<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.system_status_init_config,&to0[13])<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	for(i=0;i<14;i++)
		{
			if(to0[i]==1)
				{
					flag++;
				}
		}
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.system_status_init,&to0[14])<0)
		{
			printf("can not read tca\n");
			return -1;
		}
	if(to0[14]==1)
		{
			flag++;
		}

	if(flag==15)
		{
			printf("not first init devices  no need to init \n");
			if(i2c_read(i2c_address_mcu,mcu_i2c_info.lock_status,&to1[0])<0)
				{
					printf("can not read tca\n");
					return -1;
				}
			if(i2c_read(i2c_address_mcu,mcu_i2c_info.projector_status,&to1[1])<0)
				{
					printf("can not read tca\n");
					return -1;
				}
			if(i2c_read(i2c_address_mcu,mcu_i2c_info.chanel_status,&to1[2])<0)
				{
					printf("can not read tca\n");
					return -1;
				}
			if(to1[0]==1)
				{
					device_status.stand_lock=1;
				}
			if(to1[1]==1)
				{
					device_status.projector=1;
				}
			if(to1[2]==1)
				{
					device_status.ep=0;
				}
			
			led_glint.led_standy_flag=2;

			device_status.k4=1;
			i2c_write(i2c_address_wm,0x2b,0x1f);//打开通道 不静音
			i2c_write(i2c_address_wm,0x1c,wm8776_volume_table[volume_3105.volume]);//初始化音量
			printf("it is restart,no need to init outer device\n");
			
		}
	else
		{
			//printf("4\n");
			printf("first to init devices\n");
			//初始化tca芯片
			tca_set_bit(13);//x86开关
			usleep(10000);
			clean_tca_bit(1);
			usleep(10000);
			tca_set_bit(14);//lock_out
			usleep(10000);
			clean_tca_bit(6);
			usleep(10000);
			tca_set_bit(15);//lock_enable
			usleep(10000);
			clean_tca_bit(5);
			usleep(10000);
			tca_set_bit(16);//ope_power
			usleep(10000);
			clean_tca_bit(8);
			usleep(10000);
			tca_set_bit(17);//x84_power
			usleep(10000);
			clean_tca_bit(0);
			usleep(10000);
			tca_set_bit(18);//projector_power
			usleep(10000);
			clean_tca_bit(4);
			usleep(10000);
			tca_set_bit(19);//ep_power
			usleep(10000);
			tca_set_bit(9);
			usleep(10000);
			tca_set_bit(20);//buzzer
			usleep(10000);
			clean_tca_bit(10);
			usleep(10000);
			tca_set_bit(21);//reset_ep92
			usleep(10000);
			tca_set_bit(11);
			usleep(10000);
			tca_set_bit(22);//chanel_status
			usleep(10000);
			clean_tca_bit(3);
			usleep(10000);
			tca_set_bit(23);//projector_status
			usleep(10000);
			clean_tca_bit(7);
			usleep(10000);
			tca_set_bit(24);//system_status
			usleep(10000);
			clean_tca_bit(2);
			usleep(10000);
			tca_set_bit(25);//lock_status
			usleep(10000);
			clean_tca_bit(12);
			usleep(10000);
			tca_set_bit(27);//system_status_init
			usleep(10000);
			clean_tca_bit(26);
			usleep(10000);

			init_tas();
			//printf("5\n");
			init_wm();
			//printf("6\n");
			//ep_open();
			//printf("7\n");
			rc522_init();
			//printf("8\n");
			ep_open();





			
			//tca_set_bit(3);
		}





	
	//Init_tlv320();
	/*init_tas();
	init_wm();
	ep_open();
	rc522_init();

	//初始化tca芯片
	i2c_write(i2c_address_tca,tca_info.port0_out,to);
	i2c_write(i2c_address_tca,tca_info.port0_configuration,too);*/
	/*for(k=0;k<4;k++)
		{
			usleep(300);
			tca_set_bit(2);
			usleep(300);
			clean_tca_bit(2);
		}*/
	return 1;
}




int open_machine(void)
{
	int flag=0;
	pthread_mutex_lock(&action_mutex);
	if((device_status.machine_flag==0)&&(device_status.k4==0)&&(device_status.pc_flag==0)&&(device_status.projector_flag==0))
		{
			flag=1;
			device_status.machine_flag=1;
		}
	pthread_mutex_unlock(&action_mutex);
	if(flag==1)
		{
			printf("open machine\n");
			
			pthread_mutex_lock(&i2c_mutex);
			//tca_set_bit(9);//ep 3.3
			//clean_tca_bit(0);//开ac
			//clean_tca_bit(4);//开x86
			//clean_tca_bit(8);//poe
			
			pthread_mutex_unlock(&i2c_mutex);
			//led灯控制
			led_glint.led_standy_flag=2;
			if(led_glint.led_standy_flag!=3)
				{
					sleep(1);
				}
			pthread_mutex_lock(&i2c_mutex);
			clean_tca_bit(2);
			usleep(10000);
			pthread_mutex_unlock(&i2c_mutex);
			local_alsa_play_inaction(1);
			stand_open_open();
			
			if(base_info.pc_automatic==1)
				{
					pc_open_open();
				}
			projector_open_open();
			
			pthread_mutex_lock(&i2c_mutex);
			printf("open system to set bit\n");
			tca_set_bit(2);
			usleep(10000);
			tca_set_bit(7);
			usleep(10000);
			tca_set_bit(26);
			usleep(10000);
			ep_passage_switch(device_status.ep);
			pthread_mutex_unlock(&i2c_mutex);
			device_status.k4=1;
			device_status.machine_flag=0;
			//rfid_info.flag=0;
		}
	return 1;
}


int close_machine(void)
{
	struct timeval tv;
	unsigned int before_close_pc;
	unsigned int after_close_pc;
	unsigned int sleep_time;
	int flag=0;
	pthread_mutex_lock(&action_mutex);
	if((device_status.machine_flag==0)&&(device_status.k4==1)&&(device_status.pc_flag==0)&&(device_status.projector_flag==0))
		{
			flag=1;
			device_status.machine_flag=1;
		}
	pthread_mutex_unlock(&action_mutex);
	if(flag==1)
		{
			printf("close machine\n");
			pthread_mutex_lock(&i2c_mutex);
			local_alsa_play_inaction(13);
			gettimeofday(&tv, NULL);
			before_close_pc=(tv.tv_sec * 1000) + (tv.tv_usec / 1000);

			pc_close_close();
			
			
			projector_close_close();
			stand_close_close();
			gettimeofday(&tv, NULL);
			after_close_pc=(tv.tv_sec * 1000) + (tv.tv_usec / 1000);
			sleep_time=(50*1000-(after_close_pc-before_close_pc))/1000;
			if(sleep_time<1)
				{
					printf("close pc time reached\n");
				}
			else
				{
					sleep(sleep_time);
				}
			//tca_set_bit(0);//关ac
			//tca_set_bit(4);//关x86
			//tca_set_bit(8);//poe
			//clean_tca_bit(9);//ep3.3
			
			clean_tca_bit(2);
			usleep(10000);
			clean_tca_bit(7);
			usleep(10000);
			clean_tca_bit(26);
			usleep(10000);
			pthread_mutex_unlock(&i2c_mutex);
			led_glint.led_standy_flag=0;
			sem_post(&led_glint.led_standy_sem_t);
			device_status.k4=0;
			device_status.machine_flag=0;
			rfid_info.flag=2;

		}
	return 1;
}


/*************************************************************************************
本地alsa播放函数申明

*************************************************************************************/
int get_play_file_name(int file_serial,char *file_name)
{
	int re=1;
	switch(file_serial)
		{
			case 1:
				strcpy(file_name,"/usr/share/brxydata/music/openmachine.wav");
				printf("play music      /usr/share/brxydata/music/openmachine.wav\n");
				break;
			case 2:
				strcpy(file_name,"/usr/share/brxydata/music/openpc.wav");
				printf("play music      /usr/share/brxydata/music/openpc.wav\n");
				break;
			case 3:
				strcpy(file_name,"/usr/share/brxydata/music/volumeincrease.wav");
				printf("play music      /usr/share/brxydata/music/volumeincrease.wav\n");
				break;
			case 4:
				strcpy(file_name,"/usr/share/brxydata/music/volumediscrease.wav");
				printf("play music      /usr/share/brxydata/music/volumediscrease.wav\n");
				break;
			case 5:
				strcpy(file_name,"/usr/share/brxydata/music/projectoron.wav");
				printf("play music      /usr/share/brxydata/music/projectoron.wav\n");
				break;
			case 6:
				strcpy(file_name,"/usr/share/brxydata/music/projectoroff.wav");
				printf("play music      /usr/share/brxydata/music/projectoroff.wav\n");
				break;
			case 7:
				strcpy(file_name,"/usr/share/brxydata/music/poeon.wav");
				printf("play music     /usr/share/brxydata/music/poeon.wav \n");
				break;
			case 8:
				strcpy(file_name,"/usr/share/brxydata/music/poeoff.wav");
				printf("play music      /usr/share/brxydata/music/poeoff.wav\n");
				break;
			case 9:
				strcpy(file_name,"/usr/share/brxydata/music/epoutside.wav");
				printf("play music      /usr/share/brxydata/music/epoutside.wav\n");
				break;
			case 10:
				strcpy(file_name,"/usr/share/brxydata/music/epinterval.wav");
				printf("play music      \n");
				break;
			case 11:
				strcpy(file_name,"/usr/share/brxydata/music/standunlock.wav");
				printf("play music      /usr/share/brxydata/music/standunlock.wav\n");
				break;
			case 12:
				strcpy(file_name,"/usr/share/brxydata/music/standlock.wav");
				printf("play music      /usr/share/brxydata/music/standlock.wav\n");
				break;
			case 13:
				strcpy(file_name,"/usr/share/brxydata/music/closemachine.wav");
				printf("play music      /usr/share/brxydata/music/closemachine.wav\n");
				break;
			case 14:
				strcpy(file_name,"/usr/share/brxydata/music/closepc.wav");
				printf("play music      /usr/share/brxydata/music/closepc.wav\n");
				break;
			case 15:
				strcpy(file_name,"/usr/share/brxydata/music/writecardok.wav");
				printf("play music     /usr/share/brxydata/music/writecardok.wav \n");
				break;
			case 16:
				strcpy(file_name,"/usr/share/brxydata/music/writecardfalse.wav");
				printf("play music     /usr/share/brxydata/music/writecardfalse.wav \n");
				break;
			case 17:
				strcpy(file_name,"/usr/share/brxydata/music/pcout.wav");
				printf("play music      /usr/share/brxydata/music/pcout.wav\n");
				break;
			case 18:
				strcpy(file_name,"/usr/share/brxydata/music/systemstarting.wav");
				printf("play music      /usr/share/brxydata/music/systemstarting.wav\n");
				break;
			case 19:
				strcpy(file_name,"/usr/share/brxydata/music/systemstartfinish.wav");
				printf("play music      /usr/share/brxydata/music/systemstartfinish.wav\n");
				break;
			case 20:
				strcpy(file_name,"/usr/share/brxydata/music/cancelmute.wav");
				printf("play music      \n");
				break;
			case 21:
				strcpy(file_name,"/usr/share/brxydata/music/revertfactory.wav");
				printf("play music      /usr/share/brxydata/music/revertfactory.wav\n");
				break;
			case 22:
				strcpy(file_name,"/usr/share/brxydata/music/1miniteshutdown.wav");
				printf("play music      \n");
				break;
			case 23:
				strcpy(file_name,"/usr/share/brxydata/music/miniteshutdown.wav");
				printf("play music      /usr/share/brxydata/music/2miniteshutdown.wav\n");
				break;
			case 24:
				strcpy(file_name,"/usr/share/brxydata/music/3miniteshutdown.wav");
				printf("play music      /usr/share/brxydata/music/3miniteshutdown.wav\n");
				break;
			case 25:
				strcpy(file_name,"/usr/share/brxydata/music/4miniteshutdown.wav");
				printf("play music      /usr/share/brxydata/music/4miniteshutdown.wav\n");
				break;
			case 26:
				strcpy(file_name,"/usr/share/brxydata/music/miniteshutdown.wav");
				printf("play music     /usr/share/brxydata/music/5miniteshutdown.wav \n");
				break;
			case 27:
				strcpy(file_name, "/usr/share/brxydata/music/mute.wav");
				printf("play music      /usr/share/brxydata/music/mute.wav\n");
				break;
			default:
				re=-1;
				break;
				
		}
	return re;
		
}
//调用函数进行语音播放
void local_alsa_play_inaction(int arg)
{
	int re;
	char path[50];
	char *buf;
	int k=44100;
	int prepare_count=0;
	int fd;
	int r;
	int readlen;
	int sample;
	int frame_period_second;
	int byte_period_second;
	struct snd_pcm_t *local_handle;
	struct snd_pcm_hw_params_t *hw_params;
	struct local_play_thread *play_thread;
	play_thread=(struct local_play_thread *)arg;
	memset(path,0,50);
	if(get_play_file_name(arg,path)<0)
		{
			printf("can not get the music file path\n");
			
			return;
		}
	if(snd_pcm_open(&local_handle,"dmix",SND_PCM_STREAM_PLAYBACK,0)<0)
		{
			printf("open alsa dmix false\n");
			
			return;
		}
	if(snd_pcm_hw_params_malloc(&hw_params)!=0)
		{
			snd_pcm_close(local_handle);
			return;
		}
	if(snd_pcm_hw_params_any(local_handle,hw_params)!=0)
		{
			snd_pcm_close(local_handle);
			return;
		}
	if(snd_pcm_hw_params_set_access(local_handle,hw_params,SND_PCM_ACCESS_RW_INTERLEAVED)!=0)
		{
			snd_pcm_close(local_handle);
			return;
		}
	if(snd_pcm_hw_params_set_format(local_handle,hw_params,SND_PCM_FORMAT_S16_LE)!=0)
		{
			snd_pcm_close(local_handle);
			return;
		}
	if(snd_pcm_hw_params_set_rate_near(local_handle,hw_params,&k,0)!=0)
		{
			snd_pcm_close(local_handle);
			return;
		}
	if(snd_pcm_hw_params_set_channels(local_handle,hw_params,2)!=0)
		{
			snd_pcm_close(local_handle);
			return;
		}
	if(snd_pcm_hw_params_get_period_size(hw_params,&frame_period_second,0)!=0)
		{
			return;
		}
	sample=16;
	byte_period_second=sample*frame_period_second*2/8;
	if(snd_pcm_hw_params(local_handle,hw_params)!=0)
		{
			snd_pcm_close(local_handle);
			return;
		}
	snd_pcm_hw_params_free(hw_params);

	fd=open(path,O_RDWR);
	if(fd<0)
		{
			printf("open file false----  %s",path);
			snd_pcm_close(local_handle);
			return;
		}
	//buf=(char *)malloc(byte_period_second);
	while((buf=(char *)malloc(byte_period_second))==NULL)
		{
			printf("malloc false\n");
			sleep(2);
		}
	while(1)
		{
			readlen=read(fd,buf,byte_period_second);
			r=snd_pcm_writei(local_handle,buf,byte_period_second/4);
			if(readlen<byte_period_second)
				{
					break;
				}
			if(!(r>0))
				{
					if(prepare_count>5)
						{
							break;
						}
					prepare_count++;
					if(snd_pcm_prepare(local_handle)!=0)
						{
							break;
						}
					
				}
		}
	free(buf);
	printf("end of local_alsa_play");
	snd_pcm_close(local_handle);
	close(fd);
	//play_thread->del=1;
	return;
	
}

void *local_alsa_play(void *arg)
{
	int re;
	char path[50];
	char *buf;
	int k=44100;
	int prepare_count=0;
	int fd;
	int r;
	int readlen;
	int sample;
	int frame_period_second;
	int byte_period_second;
	struct snd_pcm_t *local_handle;
	struct snd_pcm_hw_params_t *hw_params;
	struct local_play_thread *play_thread;
	//pthread_detach(pthread_self());
	play_thread=(struct local_play_thread *)arg;
	memset(path,0,50);
	if(get_play_file_name(play_thread->file_serial,path)<0)
		{
			printf("can not get the music file path\n");
			
			return;
		}
	if(snd_pcm_open(&local_handle,"dmix",SND_PCM_STREAM_PLAYBACK,0)<0)
		{
			printf("open alsa dmix false\n");
			
			return;
		}
	if(snd_pcm_hw_params_malloc(&hw_params)!=0)
		{
			snd_pcm_close(local_handle);
			return;
		}
	if(snd_pcm_hw_params_any(local_handle,hw_params)!=0)
		{
			snd_pcm_close(local_handle);
			return;
		}
	if(snd_pcm_hw_params_set_access(local_handle,hw_params,SND_PCM_ACCESS_RW_INTERLEAVED)!=0)
		{
			snd_pcm_close(local_handle);
			return;
		}
	if(snd_pcm_hw_params_set_format(local_handle,hw_params,SND_PCM_FORMAT_S16_LE)!=0)
		{
			snd_pcm_close(local_handle);
			return;
		}
	if(snd_pcm_hw_params_set_rate_near(local_handle,hw_params,&k,0)!=0)
		{
			snd_pcm_close(local_handle);
			return;
		}
	if(snd_pcm_hw_params_set_channels(local_handle,hw_params,2)!=0)
		{
			snd_pcm_close(local_handle);
			return;
		}
	if(snd_pcm_hw_params_get_period_size(hw_params,&frame_period_second,0)!=0)
		{
			return;
		}
	sample=16;
	byte_period_second=sample*frame_period_second*2/8;
	if(snd_pcm_hw_params(local_handle,hw_params)!=0)
		{
			snd_pcm_close(local_handle);
			return;
		}
	snd_pcm_hw_params_free(hw_params);

	fd=open(path,O_RDWR);
	if(fd<0)
		{
			printf("open file false----  %s",path);
			snd_pcm_close(local_handle);
			return;
		}
	//buf=(char *)malloc(byte_period_second);
	while((buf=(char *)malloc(byte_period_second))==NULL)
		{
			printf("malloc false\n");
			sleep(2);
		}
	while(1)
		{
			readlen=read(fd,buf,byte_period_second);
			r=snd_pcm_writei(local_handle,buf,byte_period_second/4);
			if(readlen<byte_period_second)
				{
					break;
				}
			if(!(r>0))
				{
					if(prepare_count>5)
						{
							break;
						}
					prepare_count++;
					if(snd_pcm_prepare(local_handle)!=0)
						{
							break;
						}
					
				}
			
		}
	free(buf);
	//printf("end of local_alsa_play");
	snd_pcm_close(local_handle);
	close(fd);
	play_thread->del=1;
	return;
	
}

int play_music(int file_seri)
{
	/*struct local_play_thread *sn;
	struct local_play_thread *snn;
	int ret;
	sn=(struct local_play_thread *)malloc(sizeof(struct local_play_thread));
	if(sn==NULL)
		{
			printf("play_music malloc false!\n");
			//exit(0);
			return -1;
		}
	sn->del=0;
	sn->file_serial=file_seri;
	sn->last=1;
	//pthread_attr_init(&sn->attr);
	//pthread_attr_setdetachstate(&sn->attr, PTHREAD_CREATE_DETACHED); 
	ret=pthread_create(&sn->threak_id, NULL, &local_alsa_play, (void *)sn);//LUCI socket 通信
	if(ret!=0)	
	{  
		printf("Create local_alsa_play pthread error!\n");	
		free(sn);
		return -1;	
	}
	pthread_mutex_lock(&local_play_threadheader.mutex);//上锁 
	if(local_play_threadheader.number==0)
		{
			local_play_threadheader.next=sn;
			local_play_threadheader.number++;
		}
	else
		{
			snn=local_play_threadheader.next;
			while(1)
				{
					if(snn->last==1)
						{
							snn->next=sn;
							snn->last=0;
							local_play_threadheader.number++;
							break;
						}
					else
						{
							snn=snn->next;
						}
				}
		}
	pthread_mutex_unlock(&local_play_threadheader.mutex);//上锁 */
	return 1;
}

void *local_play_thread_t_free(void *ptr)
{
	struct local_play_thread *sn;
	struct local_play_thread *snn;
	struct local_play_thread *snnn;
	int not_free_number;
	void *thrd_ret;
	while(1)
		{
			sleep(4);
			not_free_number=0;
			if(local_play_threadheader.number>5)
				{
					printf("now free local play pthread source================================\n");
					pthread_mutex_lock(&local_play_threadheader.mutex);//上锁 
					
					
					while(1)
						{
							snn=local_play_threadheader.next;
							if(snn->last==1)
								{
									pthread_join(snn->threak_id,&thrd_ret);
									free(snn);
									
									break;
								}
							else
								{
									local_play_threadheader.next=snn->next;
									pthread_join(snn->threak_id,&thrd_ret);
									free(snn);
								}
							
						}
					
					local_play_threadheader.number=0;

					pthread_mutex_unlock(&local_play_threadheader.mutex);//上锁 
					
				}
			
		}
}

void *local_play_thread_t_free_old(void *ptr)
{
	struct local_play_thread *sn;
	struct local_play_thread *snn;
	struct local_play_thread *snnn;
	int not_free_number;
	void *thrd_ret;
	while(1)
		{
			sleep(4);
			not_free_number=0;
			if(local_play_threadheader.number>5)
				{
					printf("now free local play pthread source================================\n");
					pthread_mutex_lock(&local_play_threadheader.mutex);//上锁 
					snn=local_play_threadheader.next;
					sn=snn->next;
					while(1)
						{
							if(sn->last==1)
								{
									if(sn->del==1)
										{
											pthread_join(sn->threak_id,&thrd_ret);
											free(sn);
											local_play_threadheader.number--;
											
										}
									break;
									
								}
							else
								{
									if(sn->del==1)
										{
											snn->next=sn->next;
											pthread_join(sn->threak_id,&thrd_ret);
											free(sn);
											local_play_threadheader.number--;
											sn=snn->next;
										}
									else
										{
											snn=snn->next;
											sn=snn->next;
										}
									
								}
						}
					snn=local_play_threadheader.next;
					if(local_play_threadheader.next->del==1)
						{
							
							local_play_threadheader.next=local_play_threadheader.next->next;
							pthread_join(snn->threak_id,&thrd_ret);
							free(snn);
							local_play_threadheader.number--;
						}

					pthread_mutex_unlock(&local_play_threadheader.mutex);//上锁 
					
				}
			
		}
}


/*************************************************************************************
按键线程

*************************************************************************************/

int init_key_button(void)
{
	int ret;
	key_button_info.fd=open("/dev/input/event0",O_RDWR);
	if(key_button_info.fd<0)
		{
			perror("open event0");
			return -1;
		}
	if((ret=sem_init(&key_button_info.wake_key_deal,0,0))!=0)
		{
			perror("init semphare key_button_info.wake_key_deal fail");
			return -1;
		}
	return 1;
}
void *read_button_event(void)
{
	int ret;
	int flag=0;
	while(1)
		{
			ret=read(key_button_info.fd,&key_button_info.event,sizeof(struct input_event));
			
			if((ret>0)&&(key_button_info.event.code!=0))
				{
					flag=0;
					pthread_mutex_lock(&key__mutex);
					if((key_button_info.key_deal_flag==0)&&(key_button_info.count==0))
						{
							flag=1;
						}
					pthread_mutex_unlock(&key__mutex);
					if(flag==1)
						{
							key_button_info.code=(unsigned int)(key_button_info.event.code&0xffff);
							key_button_info.value=(unsigned int)(key_button_info.event.value&0xffff);
							printf("code=%d   value=%d\n",key_button_info.code,key_button_info.value);
							sem_post(&key_button_info.wake_key_deal);
							key_button_info.count=1;
						}
				}
		}
}

int deal_mcu_key_control(void)
{
	unsigned char getdata;
	if(i2c_read(i2c_address_mcu,mcu_i2c_info.key_control_register,&getdata)<0)
		{
			printf("deal_mcu_key_control read false\n");
			return -1;
		}
	if(getdata==1)
		{
			if(base_info.openlimit==1)
				{
					open_machine();
					printf(" mcu key do open_machine");
				}
		}
	else if(getdata==2)
		{
			close_machine();
			printf(" mcu key do close_machine");
		}
	else if(getdata==3)
		{
			pc_open();
			printf(" mcu key do pc_open");
		}
	else if(getdata==4)
		{
			pc_close();
			printf(" mcu key do pc_close");
		}
	else if(getdata==5)
		{
			projector_open();
			printf(" mcu key do projector_open");
		}
	else if(getdata==6)
		{
			projector_close();
			printf(" mcu key do projector_close");
		}
	else if(getdata==7)
		{
			ep_switch_to_in();
			printf(" mcu key do ep_switch_to_in");
		}
	else if(getdata==8)
		{
			ep_switch_to_out();
			printf(" mcu key do ep_switch_to_out");
		}
	else if(getdata==9)
		{
			volume_add();
			printf(" mcu key do volume_add");
			
		}
	else if(getdata==10)
		{
			volume_discrease();
			printf(" mcu key do volume_discrease");
		}
	else if(getdata==11)
		{
			if(volume_3105.mute==1)
				{
					volume_mute_disable();
					printf(" mcu key do volume_mute_disable");
				}
			else
				{
					volume_mute();
					printf(" mcu key do volume_mute");
				}
		}
	else if(getdata==12)
		{
			printf("screen stop\n");
			printf(" mcu key do screen stop");
		}
	else if(getdata==13)
		{
			printf("screen up\n");
			printf(" mcu key do screen up");
		}
	else if(getdata==14)
		{
			printf("screen down");
			printf(" mcu key do screen down");
		}
	else if(getdata==15)
		{
			stand_open();
			printf(" mcu key do stand_open");
		}
	else if(getdata==16)
		{
			stand_close();
			printf(" mcu key do stand_close");
		}
	return 1;
}





void *deal_with_button_event(void)
{
	int ret;
	sleep(3);
	key_button_info.key_deal_flag=0;
	while(1)
		{
			sem_wait(&key_button_info.wake_key_deal);
			pthread_mutex_lock(&key__mutex);
			key_button_info.key_deal_flag=1;
			pthread_mutex_unlock(&key__mutex);
			printf("deal with key\n");
			printf("key_button_info.code = %d\n",key_button_info.code);
			if(key_button_info.code==263)
				{
					if(key_button_info.value==2)
							{
							
									if(device_status.k4==0)
										{
											if(base_info.openlimit==1)
												{
													open_machine();
												}
												
										}
									else
										{
											close_machine();
										}
									
							}
				}
			else if(key_button_info.code==262)
				{
					if(key_button_info.value==2)
							{
								if(device_status.pc==0)
									{
										pc_open();
									}
								else
									{
										pc_close();
									}
							}
				}
			else if(key_button_info.code==264)
				{
					
					if(key_button_info.value==1)
						{
							deal_mcu_key_control();
						}
				}
			else if(key_button_info.code==261)
				{
					if(key_button_info.value==0)
							{
								if(device_status.projector==0)
									{
										projector_open();
									}
								else
									{
										projector_close();
									}
							}
				}
			else if(key_button_info.code==260)
				{
					if(key_button_info.value==0)
							{
								ep_change();
							}
				}
			else if(key_button_info.code==259)
				{
					if(key_button_info.value==0)
							{
								volume_add();
							}
				}
			else if(key_button_info.code==258)
				{
					if(key_button_info.value==0)
							{
								volume_discrease();
							}
						if(key_button_info.value==2)
							{
								volume_mute();
							}
				}
			else if(key_button_info.code==257)
				{
					if(key_button_info.value==1)
							{
								if(device_status.stand_lock==0)
									{
										stand_open();
										
									}
								else
									{
										stand_close();
									}
							}
				}

			key_button_info.key_deal_flag=0;
			key_button_info.count=0;
		}
}









/*************************************************************************************
串口设置及初始化

*************************************************************************************/


/*
    设置串口中最基本的包括波特率设置，校验位和停止位设置。串口的设置主
要是设置struct termios结构体的各成员值，如下所示：
＃include
struct termio
{      
      unsigned short c_iflag; // 输入模式标志
      unsigned short c_oflag; // 输出模式标志
      unsigned short c_cflag; // 控制模式标志
      unsigned short c_lflag; //本地模式标志 
      unsigned char c_line; // line discipline 
      unsigned char c_cc[NCC]; // control characters 
};
在这个结构中最为重要的是c_cflag，通过对它的赋值，用户可以设置波特率、字符大小、
数据位、停止位、奇偶校验位和硬件流控等。另外c_iflag 和c_cc 也是比较常用的标志。在
此主要对这3 个成员进行详细说明。
c_cflag支持的常量名称a
CBAUD        波特率的位掩码
B0           0波特率（放弃DTR）
B1800        1800波特率
B2400        2400波特率
B4800        4800波特率
B9600        9600波特率
B19200       19200波特率
B38400       38400波特率
B57600       57600波特率
B115200      115200波特率
EXTA         外部时钟率
EXTB         外部时钟率
CSIZE        数据位的位掩码
CS5          5个数据位
CS6          6个数据位
CS7          7个数据位
CS8          8个数据位
CSTOPB       2个停止位（不设则是1个停止位）
CREAD        接收使能
PARENB       校验位使能
PARODD       使用奇校验而不使用偶校验
HUPCL        最后关闭时挂线（放弃DTR）
CLOCAL       本地连接（不改变端口所有者）
LOBLK        块作业控制输出
CNET_CTSRTS  硬件流控制使能

c_iflag支持的常量名称b
INPCK        奇偶校验使能
IGNPAR       忽略奇偶校验错误
PARMRK       奇偶校验错误掩码
ISTRIP       除去奇偶校验位
IXON         启动出口硬件流控
IXOFF        启动入口软件流控
IXANY        允许字符重新启动流控
IGNBRK       忽略中断情况
BRKINT       当发生中断时发送SIGINT信号
INLCR        将NL映射到CR
IGNCR        忽略CR
ICRNL        将CR映射到NL
IUCLC        将高位情况映射到低位情况
IMAXBEL      当输入太长时回复ECHO

串口控制函数
Tcgetattr         取属性(termios结构)
Tcsetattr         设置属性(termios结构)
cfgetispeed     得到输入速度
Cfgetospeed           得到输出速度
Cfsetispeed            设置输入速度
Cfsetospeed           设置输出速度
Tcdrain           等待所有输出都被传输
tcflow           挂起传输或接收
tcflush           刷清未决输入和/或输出
Tcsendbreak           送BREAK字符
tcgetpgrp              得到前台进程组ID
tcsetpgrp               设置前台进程组ID

设置奇偶校验位，使用c_cflag和c_iflag.
设置奇校验:
newtio.c_cflag |= PARENB;
newtio.c_cflag |= PARODD;
newtio.c_iflag |= (INPCK | ISTRIP);
设置偶校验:
newtio.c_iflag |= (INPCK｜ISTRIP);
newtio.c_cflag |= PARENB;
newtio.c_cflag |= ~PARODD;
	*/ 
/*****************************
*设置串口波特率 校验位 停止位
*Author  mleaf_hexi
*Mail:350983773@qq.com
*****************************/ 

int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio,oldtio;
	if  ( tcgetattr( fd,&oldtio)  !=  0) 
	{ 
	 perror("SetupSerial 1");
	 return -1;
	}
	bzero( &newtio, sizeof( newtio ) );
	newtio.c_cflag  |=  CLOCAL | CREAD; 
	newtio.c_cflag &= ~CSIZE; 

	switch( nBits )
	{
	case 7:
	 newtio.c_cflag |= CS7;
	 break;
	case 8:
	 newtio.c_cflag |= CS8;
	 break;
	}

	switch( nEvent )
	{
	case 'O':					   //奇校验
	 newtio.c_cflag |= PARENB;
	 newtio.c_cflag |= PARODD;
	 newtio.c_iflag |= (INPCK | ISTRIP);
	 break;
	case 'E':					   //偶校验
	 newtio.c_iflag |= (INPCK | ISTRIP);
	 newtio.c_cflag |= PARENB;
	 newtio.c_cflag &= ~PARODD;
	 break;
	case 'N':					  //无校验
	 newtio.c_cflag &= ~PARENB;
	 break;
	}

	switch( nSpeed )
	{
	case 50:
	 cfsetispeed(&newtio, B50);
	 cfsetospeed(&newtio, B50);
	 break;
	case 75:
	 cfsetispeed(&newtio, B75);
	 cfsetospeed(&newtio, B75);
	 break;
	case 110:
	 cfsetispeed(&newtio, B110);
	 cfsetospeed(&newtio, B110);
	 break;
	 case 134:
	 cfsetispeed(&newtio, B134);
	 cfsetospeed(&newtio, B134);
	 break;
	 case 150:
	 cfsetispeed(&newtio, B150);
	 cfsetospeed(&newtio, B150);
	 break;
	 case 200:
	 cfsetispeed(&newtio, B200);
	 cfsetospeed(&newtio, B200);
	 break;
	 case 300:
	 cfsetispeed(&newtio, B300);
	 cfsetospeed(&newtio, B300);
	 break;
	 case 600:
	 cfsetispeed(&newtio, B600);
	 cfsetospeed(&newtio, B600);
	 break;
	 case 1200:
	 cfsetispeed(&newtio, B1200);
	 cfsetospeed(&newtio, B1200);
	 break;
	 case 1800:
	 cfsetispeed(&newtio, B1800);
	 cfsetospeed(&newtio, B1800);
	 break;
	 case 2400:
	 cfsetispeed(&newtio, B2400);
	 cfsetospeed(&newtio, B2400);
	 break;
	 case 4800:
	 cfsetispeed(&newtio, B4800);
	 cfsetospeed(&newtio, B4800);
	 break;
	 case 9600:
	 cfsetispeed(&newtio, B9600);
	 cfsetospeed(&newtio, B9600);
	 break;
	 case 14400:
	 cfsetispeed(&newtio, B9600);
	 cfsetospeed(&newtio, B9600);
	 break;
	 case 19200:
	 cfsetispeed(&newtio, B19200);
	 cfsetospeed(&newtio, B19200);
	 break;
	 case 38400:
	 cfsetispeed(&newtio, B38400);
	 cfsetospeed(&newtio, B38400);
	 break;
	 case 57600:
	 cfsetispeed(&newtio, B57600);
	 cfsetospeed(&newtio, B57600);
	 break;
	 case 115200:
	 cfsetispeed(&newtio, B115200);
	 cfsetospeed(&newtio, B115200);
	  break;
	  case 230400:
	 cfsetispeed(&newtio, B230400);
	 cfsetospeed(&newtio, B230400);
	  break;
	  case 460800:
	 cfsetispeed(&newtio, B460800);
	 cfsetospeed(&newtio, B460800);
	  break;
	  case 500000:
	 cfsetispeed(&newtio, B500000);
	 cfsetospeed(&newtio, B500000);
	  break;
	  case 576000:
	 cfsetispeed(&newtio, B576000);
	 cfsetospeed(&newtio, B576000);
	  break;
	  case 921600:
	 cfsetispeed(&newtio, B921600);
	 cfsetospeed(&newtio, B921600);
	  break;
	  case 1000000:
	 cfsetispeed(&newtio, B1000000);
	 cfsetospeed(&newtio, B1000000);
	  break;
	  case 1152000:
	 cfsetispeed(&newtio, B1152000);
	 cfsetospeed(&newtio, B1152000);
	  break;
	  case 1500000:
	 cfsetispeed(&newtio, B1500000);
	 cfsetospeed(&newtio, B1500000);
	  break;
	  case 2000000:
	 cfsetispeed(&newtio, B2000000);
	 cfsetospeed(&newtio, B2000000);
	  break;
	  case 2500000:
	 cfsetispeed(&newtio, B2500000);
	 cfsetospeed(&newtio, B2500000);
	  break;
	  case 3000000:
	 cfsetispeed(&newtio, B3000000);
	 cfsetospeed(&newtio, B3000000);
	  break;
	  case 3500000:
	 cfsetispeed(&newtio, B3500000);
	 cfsetospeed(&newtio, B3500000);
	  break;
	  case 4000000:
	 cfsetispeed(&newtio, B4000000);
	 cfsetospeed(&newtio, B4000000);
	  break;
	default:
	 cfsetispeed(&newtio, B9600);
	 cfsetospeed(&newtio, B9600);
	 break;
	}
	if( nStop == 1 )
	{
	 newtio.c_cflag &=	~CSTOPB;
	}
	else if ( nStop == 2 )
	{
	 newtio.c_cflag |=	CSTOPB;
	}
	newtio.c_cc[VTIME]  = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd,TCIFLUSH);
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)
	{
	 perror("com set error");
	 return -1;
	}
	printf("set done!\n");
	return 0;
}
static int set_serial(void)
{
	if((set_opt(projector_info.serial_fd,19200,8,'N',1))<0)//115200无奇偶校验
    {
        perror("set_opt error");
        return -1;
    }
	return 1;
	
}

static int init_serial(void)  
{   int i;
	struct sigaction saio; /*definition of signal axtion */
	
	projector_info.serial_fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);  
	if (projector_info.serial_fd < 0) {  
	    perror("open");  
	    return -1;  
	}

	if((set_serial())<0)//115200无奇偶校验
    {
        perror("set_opt error");
        return -1;
    }
	return 0;
}

//发送串口数据
int uart_send_data(unsigned char *data,int length)
{
	int len = 0;
	//pthread_mutex_lock(&projector_info.mutex);
	len=write(projector_info.serial_fd,data,length);//实际写入的长度  
	if(len==length)
		{
			return 1;
		}
	else 
		{
			tcflush(projector_info.serial_fd,TCOFLUSH);//TCOFLUSH刷新写入的数据但不传送 
			return -1;
		}
	
	return 1;
}
//处理mcu返回数据函数

int deal_mcu_control_to_initiative(unsigned char *data)
{
	unsigned char *buf;
	buf=data;

	if(buf[6]==0x01)
		{
			if(buf[7]==0)
				{
					device_status.k4=0;
					play_music(13);
				}
			else
				{
					device_status.k4=1;
					play_music(1);
				}
			return 1;
		}
	else if(buf[6]==0x02)
		{
			if(buf[7]==0)
				{
					device_status.pc=0;
					play_music(14);
				}
			else
				{
					device_status.pc=1;
					play_music(2);
				}
			return 1;
		}
	else if(buf[6]==0x03)
		{
			if(buf[7]==1)
				{
					device_status.ep=1;
					play_music(10);
				}
			else if(buf[7]==2)
				{
					device_status.ep=2;
					play_music(9);
				}
			else if(buf[7]==3)
				{
					device_status.ep=3;
				}
			return 1;
		}
	else if(buf[6]==0x04)
		{
			if(buf[7]==0)
				{
					device_status.projecter_power=0;
					//play_music();
				}
			else
				{
					device_status.projecter_power=1;
					//play_music();
				}
			return 1;
		}
	else if(buf[6]==0x05)
		{
			if(buf[7]==0)
				{
					device_status.projector=0;
					play_music(6);
				}
			else
				{
					device_status.projector=1;
					play_music(5);
				}
			return 1;
		}
	else if(buf[6]==0x06)
		{
			if(buf[7]==1)
				{
					device_status.projecter_channel=1;
				}
			else if(buf[7]==2)
				{
					device_status.projecter_channel=2;
				}
			else if(buf[7]==3)
				{
					device_status.projecter_channel=3;
				}
			return 1;
		}
	else if(buf[6]==0x07)
		{
			return 1;
		}
	else if(buf[6]==0x08)
		{
			
			volume_3105.volume=buf[7];
			printf("buf[8]=%02x\n",buf[8]);
			if(buf[8]==0)
				{
					play_music(27);
				}
			else if(buf[8]==1)
				{
					//play_music(4);
				}
			else if(buf[8]==2)
				{
					//play_music(3);
				}
			return 1;
		}
	else if(buf[6]==0x09)
		{
			return 1;
		}
	else if(buf[6]==0x0a)
		{
			if(buf[7]==0)
				{
					device_status.stand_lock=0;
					play_music(12);
				}
			else
				{
					device_status.stand_lock=1;
					play_music(11);
				}
			return 1;
		}
	else if(buf[6]==0x0b)
		{
			return 1;
		}
	else if(buf[6]==0x0c)
		{
			return 1;
		}
	else if(buf[6]==0x20)
		{
			memset(rfid_info.rfid_card_id,0,4);
			memmove(rfid_info.rfid_card_id,&buf[7],4);
			if(rfid_decision(rfid_info.rfid_card_id)==2)
				{
					mcu_open_system();
				}
			return 1;
		}
	else if(buf[6]==0x21)
		{
			rfid_info.section=buf[6];
			return 1;
		}
	else if(buf[6]==0x22)
		{
			memset(rfid_info.rfid_card_projector_id_read,0,46);
			memmove(rfid_info.rfid_card_projector_id_read,&buf[6],16);
			return 1;
		}
	else if(buf[6]==0x31)
		{
			if(buf[7]==1)
				{
					play_music(22);
				}
			else if(buf[7]==2)
				{
					play_music(23);
				}
			else if(buf[7]==3)
				{
					play_music(24);
				}
			else if(buf[7]==4)
				{
					play_music(25);
				}
			else if(buf[7]==5)
				{
					play_music(26);
				}
			return 1;
		}
	return 1;
}

int deal_mcu_control_read_return(unsigned char *data)
{
	unsigned char *buf;
	buf=data;

	if(buf[6]==0x01)
		{
			if(buf[7]==0)
				{
					
					device_status.k4=0;
				}
			else
				{
					device_status.k4=1;
				}
			printf("------------------------buf[7] = %d\n",buf[7]);
			return 1;
		}
	else if(buf[6]==0x02)
		{
			if(buf[7]==0)
				{
					device_status.pc=0;
				}
			else
				{
					device_status.pc=1;
				}
			return 1;
		}
	else if(buf[6]==0x03)
		{
			if(buf[7]==1)
				{
					device_status.ep=1;
				}
			else if(buf[7]==2)
				{
					device_status.ep=2;
				}
			else if(buf[7]==3)
				{
					device_status.ep=3;
				}
			return 1;
		}
	else if(buf[6]==0x04)
		{
			if(buf[7]==0)
				{
					device_status.projecter_power=0;
				}
			else
				{
					device_status.projecter_power=1;
				}
			return 1;
		}
	else if(buf[6]==0x05)
		{
			if(buf[7]==0)
				{
					device_status.projector=0;
				}
			else
				{
					device_status.projector=1;
				}
			return 1;
		}
	else if(buf[6]==0x06)
		{
			if(buf[7]==1)
				{
					device_status.projecter_channel=1;
				}
			else if(buf[7]==2)
				{
					device_status.projecter_channel=2;
				}
			else if(buf[7]==3)
				{
					device_status.projecter_channel=3;
				}
			return 1;
		}
	else if(buf[6]==0x07)
		{
			return 1;
		}
	else if(buf[6]==0x08)
		{
			
			volume_3105.volume=buf[7];
			printf("get volume = %d\n",volume_3105.volume);
			return 1;
		}
	else if(buf[6]==0x09)
		{
			return 1;
		}
	else if(buf[6]==0x0a)
		{
			if(buf[7]==0)
				{
					device_status.stand_lock=0;
				}
			else
				{
					device_status.stand_lock=1;
				}
			return 1;
		}
	else if(buf[6]==0x0b)
		{
			return 1;
		}
	else if(buf[6]==0x0c)
		{
			return 1;
		}
	else if(buf[6]==0x20)
		{
			memset(rfid_info.rfid_card_id,0,4);
			memmove(rfid_info.rfid_card_id,&buf[7],4);
			rfid_info.read_card_id_return=1;
			return 1;
		}
	else if(buf[6]==0x21)
		{
			rfid_info.section=buf[6];
			return 1;
		}
	else if(buf[6]==0x22)
		{
			memset(rfid_info.rfid_card_projector_id_read,0,46);
			memmove(rfid_info.rfid_card_projector_id_read,&buf[6],16);
			return 1;
		}
	return 1;
}

int deal_mcu_control_write_return(unsigned char *data)
{
	unsigned char *buf;
	buf=data;

	if(buf[6]==0x01)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
			return 1;
		}
	else if(buf[6]==0x02)
		{

			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
			return 1;
		}
	else if(buf[6]==0x03)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
			return 1;
		}
	else if(buf[6]==0x04)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
			return 1;
		}
	else if(buf[6]==0x05)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
			return 1;

		}
	else if(buf[6]==0x06)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
			return 1;
		}
	else if(buf[6]==0x07)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
			return 1;
		}
	else if(buf[6]==0x08)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
			return 1;
		}
	else if(buf[6]==0x09)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
			return 1;
		}
	else if(buf[6]==0x0a)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
			return 1;
		}
	else if(buf[6]==0x0b)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
			return 1;
		}
	else if(buf[6]==0x0c)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
			return 1;
		}
	else if(buf[6]==0x20)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
			return 1;
		}
	else if(buf[6]==0x21)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
			return 1;
		}
	else if(buf[6]==0x22)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
			return 1;
		}
	return 1;
}


int deal_mcu_config_read_return(unsigned char *data)
{
	unsigned char *buf;
	unsigned short len=0;
	int i;
	int mm;
	int m;
	unsigned int open0;
	unsigned int open1;
	unsigned int open2;
	buf=data;

	if(buf[6]==0x00)
		{
			
			return 1;
		}
	else if(buf[6]==0x01)
		{
			
			return 1;
		}
	else if(buf[6]==0x02)
		{

			switch(buf[7])
				{
					
					case 0:
						projector_info.baudrate=9600;
						break;
					case 1:
						projector_info.baudrate=14400;
						break;
					case 2:
						projector_info.baudrate=19200;
						break;
					case 3:
						projector_info.baudrate=38400;
						break;
					case 4:
						projector_info.baudrate=57600;
						break;
					case 5:
						projector_info.baudrate=115200;
						break;
					default :
						projector_info.baudrate=115200;
						break;
				}

			
			//projector_info.baudrate=buf[7];
			projector_info.databyte=buf[8];
			if(buf[9]==0)
				{
					projector_info.check='N';
				}
			else if(buf[9]==1)
				{
					projector_info.check='O';
				}
			else if(buf[9]==2)
				{
					projector_info.check='E';
				}
			projector_info.stop=buf[10]+1;
			return 1;
		}
	else if(buf[6]==0x03)
		{
			
			return 1;
		}
	else if(buf[6]==0x04)
		{
			len=buf[2];
			len=len<<8;
			len+=buf[3];
			len=len-1;
			if(len>510)
				{
					return -1;
				}
			memset(projector_info.start_code,0,512);
			for(i=0;i<len;i++)
				{
					mm=i+7;
					m=i*2;
					sprintf(&projector_info.start_code[m],"%02x",buf[mm]);
				}
			printf("  len = %d projector_info.start_code = %s\n",len,projector_info.start_code);
			//sprintf(sendbuf,"%02x%02x%02x%02x",rfid_info.rfid_card_id[0],rfid_info.rfid_card_id[1],rfid_info.rfid_card_id[2],rfid_info.rfid_card_id[3]);
			return 1;
		}
	else if(buf[6]==0x05)
		{
			len=buf[2];
			len=len<<8;
			len+=buf[3];
			len=len-1;
			if(len>510)
				{
					return -1;
				}
			memset(projector_info.shutdown_code,0,512);
			for(i=0;i<len;i++)
				{
					mm=i+7;
					m=i*2;
					sprintf(&projector_info.shutdown_code[m],"%02x",buf[mm]);
				}
			printf("  len = %d projector_info.shutdown_code = %s\n",len,projector_info.shutdown_code);
			
			return 1;
		}
	else if(buf[6]==0x06)
		{
			len=buf[2];
			len=len<<8;
			len+=buf[3];
			len=len-1;
			if(len>510)
				{
					return -1;
				}
			memset(projector_info.show_channel,0,512);
			if(len>510)
				{
					return -1;
				}
			for(i=0;i<len;i++)
				{
					mm=i+7;
					m=i*2;
					sprintf(&projector_info.show_channel[m],"%02x",buf[mm]);
				}
			printf("  len = %d projector_info.show_channel = %s\n",len,projector_info.show_channel);
			return 1;
		}
	else if(buf[6]==0x07)
		{
			
			volume_3105.volume_default=buf[7];
			return 1;
		}
	else if(buf[6]==0x08)
		{
			return 1;
		}
	else if(buf[6]==0x09)
		{
			return 1;
		}
	else if(buf[6]==0x0a)
		{
			return 1;
		}
	else if(buf[6]==0x0b)
		{
			return 1;
		}
	else if(buf[6]==0x0c)
		{
			base_info.pc_automatic=buf[7];
			return 1;
		}
	else if(buf[6]==0x0d)
		{
			projector_info.start_delay=buf[7];
			return 1;
		}
	else if(buf[6]==0x0e)
		{
			projector_info.shutdown_delay=buf[7];
			return 1;
		}
	else if(buf[6]==0x0f)
		{
			projector_info.interval=buf[7];
			return 1;
		}
	else if(buf[6]==0x10)
		{
			return 1;
		}
	else if(buf[6]==0x11)
		{
			return 1;
		}
	else if(buf[6]==0x12)
		{
			return 1;
		}
	else if(buf[6]==0x13)
		{
			open1=buf[7]&0x02;
			if(open1>0)
				{
					base_info.openlimit=1;
				}
			else
				{
					base_info.openlimit=0;
				}
			return 1;
		}
	else if(buf[6]==0x13)
		{
			open1=buf[7]&0x02;
			if(open1>0)
				{
					base_info.openlimit=1;
				}
			else
				{
					base_info.openlimit=0;
				}
			return 1;
		}
	else if(buf[6]==0x22)
		{
			rfid_info.opentime=buf[7]*60;
			return 1;
		}
	else if(buf[6]==0x24)
		{
			memmove(base_info.sn_mcu,&buf[7],8);
			projector_info.get_qq_sn_return_flag=1;
			return 1;
		}
	return 1;
}

int deal_mcu_config_write_return(unsigned char *data)
{
	unsigned char *buf;
	buf=data;

	if(buf[6]==0x00)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x01)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x02)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x03)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x04)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x05)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x06)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x07)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x08)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x09)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x0a)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x0b)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x0c)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x0d)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x0e)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x0f)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x10)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x11)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x12)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x13)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	else if(buf[6]==0x22)
		{
			if(buf[7]==0)
				{
					projector_info.send_return_status=1;
				}
			else
				{
					projector_info.send_return_status=2;
				}
		}
	return 1;
}

int deal_mcu_block_read_return(unsigned char *data)
{
	unsigned char *buf;
	unsigned short len=0;
	
	buf=data;
	len=buf[2];
	len=len<<8;
	len+=buf[3];
	memset(projector_info.get_qq_license,0,256);
	memmove(projector_info.get_qq_license,&buf[7],(len-1));
	projector_info.get_qq_license_return_flag=1;
	return 1;
}

int deal_mcu_block_write_return(unsigned char *data)
{
	unsigned char *buf;
	unsigned short len=0;
	
	buf=data;
	len=buf[2];
	len=len<<8;
	len+=buf[3];
	
	memset(projector_info.get_qq_license,0,256);
	memmove(projector_info.get_qq_license,&buf[7],(len-1));
	printf("buf[5]=%02x  buf[6]=%02x\n",buf[6],buf[7]);
	projector_info.get_qq_license_return_flag=1;
	return 1;
}

int deal_mcu_projector_test_return(unsigned char *data)
{
	unsigned char *buf;
	unsigned short len=0;
	
	buf=data;
	if(buf[6]==0)
		{
			projector_info.send_return_status=1;
		}
	else
		{
			projector_info.send_return_status=2;
			
		}
	return 1;
}

/*************************************************************************************
初始化读取mcu 中的配置参数及各个设备状态函数


*************************************************************************************/
//具体数据见协议控制表
int uart_read_status(int order_numeber)
{
	unsigned char sendbuf[512];
	unsigned short len=0;
	int order_length=0;
	int i;
	len=1;
	sendbuf[0]=0xaa;
	sendbuf[1]=0x55;
	
	sendbuf[2]=(len>>8);
	sendbuf[3]=len;
	
	sendbuf[4]=0x00;
	sendbuf[5]=0x00;

	sendbuf[6]=order_numeber;
	
	order_length=7+len;
	sendbuf[order_length-1]=0;
	printf("now send data to mcu----------------\n");
	printf("uart_read_status  ");
	for(i=0;i<order_length-1;i++)
		{
			sendbuf[order_length-1]+=sendbuf[i];
			printf(" %02x",sendbuf[i]);
		}
	printf(" %02x\n",sendbuf[order_length-1]);
	//usleep(200000);
	uart_send_data(sendbuf,order_length);
	usleep(200000);
	return 1;
}
int uart_send_status(int order_numeber,int status)
{
	unsigned char sendbuf[512];
	unsigned short len=0;
	int order_length=0;
	int i;
	len=2;
	sendbuf[0]=0xaa;
	sendbuf[1]=0x55;

	sendbuf[2]=(len>>8);
	sendbuf[3]=len;
	
	sendbuf[4]=0x00;
	sendbuf[5]=0x01;

	sendbuf[6]=order_numeber;
	sendbuf[7]=status;

	order_length=7+len;
	sendbuf[order_length-1]=0;
	printf("now send data to mcu----------------\n");
	printf("uart_send_status  ");
	for(i=0;i<order_length-1;i++)
		{
			sendbuf[order_length-1]+=sendbuf[i];
			//printf("uart_send_status  %02x\n",sendbuf[i]);
			printf(" %02x",sendbuf[i]);
		}
	printf(" %02x\n",sendbuf[order_length-1]);
	//usleep(200000);
	uart_send_data(sendbuf,order_length);
	usleep(200000);
	return 1;
}


int uart_read_config(int order_numeber)
{
	unsigned char sendbuf[512];
	unsigned short len=0;
	int order_length=0;
	int i;
	len=1;
	sendbuf[0]=0xaa;
	sendbuf[1]=0x55;

	sendbuf[2]=(len>>8);
	sendbuf[3]=len;

	sendbuf[4]=0x00;
	sendbuf[5]=0x02;

	sendbuf[6]=order_numeber;

	order_length=7+len;
	sendbuf[order_length-1]=0;
	printf("now send data to mcu----------------\n");
	printf("uart_read_config  ");
	for(i=0;i<order_length-1;i++)
		{
			sendbuf[order_length-1]+=sendbuf[i];
			//printf("uart_read_config  %02x\n",sendbuf[i]);
			printf(" %02x",sendbuf[i]);
		}
	printf(" %02x\n",sendbuf[order_length-1]);
	//usleep(200000);
	uart_send_data(sendbuf,order_length);
	usleep(200000);
	return 1;
}

//length是除去指令编号的数据长度
int uart_send_config(int order_numeber,unsigned char *data,int length)
{
	unsigned char sendbuf[525]={0};
	unsigned short len=0;
	int order_length=0;
	int i;
	len=length+1;
	sendbuf[0]=0xaa;
	sendbuf[1]=0x55;

	sendbuf[2]=(len>>8);
	sendbuf[3]=len;

	sendbuf[4]=0x00;
	sendbuf[5]=0x03;

	

	sendbuf[6]=order_numeber;
	
	for(i=0;i<length;i++)
		{
			sendbuf[i+7]=data[i];
		}

	order_length=7+len;
	sendbuf[order_length-1]=0;
	printf("now send data to mcu----------------\n");
	printf("uart_send_config  ");
	for(i=0;i<order_length-1;i++)
		{
			sendbuf[order_length-1]+=sendbuf[i];
			//printf("uart_read_config  %02x\n",sendbuf[i]);
			printf(" %02x",sendbuf[i]);
		}
	printf(" %02x\n",sendbuf[order_length-1]);
	//usleep(200000);
	uart_send_data(sendbuf,order_length);
	usleep(200000);
	return 1;
}



int read_status_from_mcu(void)
{
	int i=0;
	for(i=1;i<13;i++)
		{
			//usleep(200000);
			uart_read_status(i);
			
		}
	
	return 1;
}

int read_config_from_mcu(void)
{
	int i=0;
	for(i=0;i<20;i++)
		{
			//usleep(200000);
			uart_read_config(i);
			
		}
	uart_read_config(0x22);
	return 1;
}

int read_all_data_from_mcu()
{
	read_status_from_mcu();
	read_config_from_mcu();
	return 1;
}

int apply_net_work()
{
	unsigned char buf[10];
	buf[0]=1;
	uart_send_config(0x12,buf,1);
	return 1;
}

int apply_openpermision(int n)
{
	unsigned char buf[10];
	if(n==1)
		{
			buf[0]=0x07;
		}
	else
		{
			buf[0]=0x05;
		}
	uart_send_config(0x13,buf,1);
	return 1;
	
}

int apply_net_work_to_mcu_and_openpermision()
{
	apply_net_work();
	//usleep(200000);
	apply_openpermision(base_info.openlimit);
}


//最基本的控制
int mcu_open_system(void)
{
	uart_send_status(1,1);
	//usleep(200000);
	//uart_read_status(1);
	
}

int mcu_shutdown_system(void)
{
	uart_send_status(1,0);
	//usleep(200000);
	//uart_read_status(1);
}

int mcu_open_pc(void)
{
	uart_send_status(2,1);
	//usleep(200000);
	//uart_read_status(2);
}
int mcu_shutdown_pc(void)
{
	uart_send_status(2,0);
	//usleep(200000);
	//uart_read_status(2);
}


int mcu_change_channel(int channel)
{
	uart_send_status(3,channel);
	//usleep(200000);
	//uart_read_status(3);
}

int mcu_open_projector(void)
{
	uart_send_status(5,1);
	//usleep(200000);
	//uart_read_status(5);
}
int mcu_shutdown_projector(void)
{
	uart_send_status(5,0);
	//usleep(200000);
	//uart_read_status(5);
}

int mcu_screen_control(int n)
{
	uart_send_status(7,n);
	//usleep(200000);
	//uart_read_status(7);
}

int mcu_volume_mute(void)
{
	uart_send_status(8,0);
	//usleep(200000);
	//uart_read_status(8);
}



int mcu_config_volume(int volume)
{
	uart_send_status(8,volume);
	//usleep(200000);
	//uart_read_status(8);
}







int mcu_volume_unmute(void)
{
	uart_send_status(8,1);
	//usleep(200000);
	//uart_read_status(8);
}
int mcu_open_stand(void)
{
	uart_send_status(10,1);
	//usleep(200000);
	//uart_read_status(10);
}
int mcu_shutdown_stand(void)
{
	uart_send_status(10,0);
	//usleep(200000);
	//uart_read_status(10);
}
//配置数据下发函数
int mcu_config_projector_uart_parameter(unsigned char *data,int len)
{
	uart_send_config(2,data,len);
	//usleep(200000);
	//uart_read_config(2);
}
int mcu_config_projector_start_code(unsigned char *data,int len)
{
	uart_send_config(4,data,len);
	//usleep(200000);
	//uart_read_config(4);
}
int mcu_config_projector_shutdown_code(unsigned char *data,int len)
{
	uart_send_config(5,data,len);
	//usleep(200000);
	//uart_read_config(5);
}
int mcu_config_projector_channel_code(unsigned char *data,int len)
{
	uart_send_config(6,data,len);
	//usleep(200000);
	//uart_read_config(6);
}
/*int mcu_config_volume(int volume)
{
	unsigned char *sendbuf[10];
	sendbuf[0]=volume;
	uart_send_config(7,sendbuf,1);
	usleep(200000);
	uart_read_config(7);
}*/
int mcu_config_pc_open_pc_automatic(int n)
{
	unsigned char *sendbuf[10];
	sendbuf[0]=n;
	uart_send_config(0x0c,sendbuf,1);
	//usleep(200000);
	//uart_read_config(0x0c);
}
int mcu_config_projector_start_delay(int delay)
{
	unsigned char *sendbuf[10];
	sendbuf[0]=delay;
	uart_send_config(0x0d,sendbuf,1);
	//usleep(200000);
	//uart_read_config(0x0d);
}
int mcu_config_projector_stop_delay(int delay)
{
	unsigned char *sendbuf[10];
	sendbuf[0]=delay;
	uart_send_config(0x0e,sendbuf,1);
	//usleep(200000);
	//uart_read_config(0x0d);
}
int mcu_config_projector_interval_delay(int delay)
{
	unsigned char *sendbuf[10];
	sendbuf[0]=delay;
	uart_send_config(0x0f,sendbuf,1);
	//usleep(200000);
	//uart_read_config(0x0d);
}
int mcu_config_net_work(void)
{
	apply_net_work();
	
}
int mcu_config_openpermision(int n)
{
	apply_openpermision(n);
	
}


//透传

int mcu_send_code_to_projector(unsigned char *data,int length)
{
	unsigned char sendbuf[32];
	unsigned short len=0;
	int order_length=0;
	int i;
	len=length;
	sendbuf[0]=0xaa;
	sendbuf[1]=0x55;

	sendbuf[2]=(len>>8);
	sendbuf[3]=len;

	sendbuf[4]=0x00;
	sendbuf[5]=0x31;


	for(i=0;i<length;i++)
		{
			sendbuf[i+6]=data[i];
		}

	order_length=6+len;
	sendbuf[order_length]=0;
	for(i=0;i<order_length;i++)
		{
			sendbuf[order_length]+=sendbuf[i];
		}
	printf("test projector send   ");
	for(i=0;i<order_length+1;i++)
		{
			printf(" %02x",sendbuf[i]);
		}
	printf("\n");
	//usleep(200000);
	uart_send_data(sendbuf,order_length+1);
	usleep(200000);
	return 1;
}

/************************************************************************ 
*串口接收数据 测试 通过select系统调用进行io多路切换，实现异步读取串口数据
*Version 1.0 
*Created on: 2015-2-2 
*Author:  mleaf_hexi
*Mail:350983773@qq.com
*
*FD_ISSET:
*int FD_ISSET(int fd,fd_set *fdset) /*is the bit for fd on in fdset
*判断描述符fd是否在给定的描述符集fdset中，通常配合select函数使用，
*由于select函数成功返回时会将未准备好的描述符位清零。
*通常我们使用FD_ISSET是为了检查在select函数返回后，
*某个描述符是否准备好，以便进行接下来的处理操作。
*当描述符fd在描述符集fdset中返回非零值，否则，返回零。
*
*************************************************************************/
static void serial_select_receive(void){

	
	fd_set rd;  
	int get_end=0;
	FD_ZERO(&rd);  
	FD_SET(projector_info.serial_fd, &rd); 

	unsigned char buf[256];
	unsigned char buff[1038];
	unsigned short data_length=0;
	unsigned char check;
	unsigned char *buff_pointer;
	unsigned short buff_pointer_len;
	unsigned short buff_pointer_length;
	int j,k;
	unsigned int Length;
	unsigned int Length1;
	int nread = 0;
	unsigned int count = 0;
	
	memset(buf, 0 , sizeof(buf));
	memset(buff, 0 , sizeof(buff));
	
	char *Receive;
	int Mcu_Length=0;
	int Mcu_Length2=0;
	int Mcu_Length3=0;
	int Mcu_Length4=0;
	int Mcu_Length5=0;
	int Length_Count=0;

	while(FD_ISSET(projector_info.serial_fd, &rd))  
	   {  
	  		printf("now serial in select wait !\n");
		   if(select(projector_info.serial_fd+1, &rd, NULL,NULL,NULL) < 0)  
		   {  
		   		perror("serial select error\n");  
		   }  
		   else  
		   {	

		   		bzero(buf, sizeof(buf));
				memset(buff,0,1038);
				data_length=0;
				count=0;
				check=0;
			  	//process_read_data();//处理串口接收到的数据
				get_end=0;
				while(1)	
   				{	
   					//get_end=0;
   					if((nread = read(projector_info.serial_fd, buf, sizeof(buf))) > 0)
   						{
   							get_end=0;
   						}
					else
						{
							if(get_end<10)
								{
									get_end++;
									usleep(1000);
									continue;
								}
							if(get_end==10)
								{
									break;
								}
							
						}
					if(count>1024)
						{
							buff[0]=0;
							continue;
						}
					//printf("Len %d   ",nread);
					//printf("nread = %d,%s\n",nread, buf);
					memcpy(&buff[count],buf,nread);
					bzero(buf, sizeof(buf));
					count+=nread;
				}
				buff_pointer=buff;
				printf("get buf from serial\n");
				while(1)
					{
						if(buff_pointer[0]!=0xaa&&buff_pointer[1]!=0x55)
							{
								
								break;
							}
						//buff_pointer_len=0;
						buff_pointer_len=buff_pointer[2];
						buff_pointer_len=buff_pointer_len<<8;
						buff_pointer_len+=buff_pointer[3];
						Length=6+buff_pointer_len;
						Length1=Length+1;
						check=0;
						printf("get buf = ");
						for(j=0;j<Length1;j++)
							{
								printf("%02x ",buff_pointer[j]);
								if(j<(Length))
									{
										check+=buff_pointer[j];
									}
							}
						printf("\n");
						//判断前导码
						printf("check = %d  buff[%d] = %d\n",check,Length,buff_pointer[Length]);
						if(check==buff_pointer[Length])
							{
								data_length=buff_pointer[2];
								data_length=data_length<<8;
								data_length+=buff_pointer[3];
								
								printf("get a value data\n");
								if(buff_pointer[4]==0x80)
									{
										if(buff_pointer[5]==0x00)
											{
												deal_mcu_control_read_return(buff_pointer);
											}
										if(buff_pointer[5]==0x01)
											{
												deal_mcu_control_write_return(buff_pointer);
											}		
										if(buff_pointer[5]==0x02)
											{
												deal_mcu_config_read_return(buff_pointer);
											}
										if(buff_pointer[5]==0x03)
											{
												deal_mcu_config_write_return(buff_pointer);
											}
										if(buff_pointer[5]==0x04)
											{
												deal_mcu_block_read_return(buff_pointer);
											}
										if(buff_pointer[5]==0x05)
											{
												deal_mcu_block_write_return(buff_pointer);
											}
										if(buff_pointer[5]==0x31)
											{
												deal_mcu_projector_test_return(buff_pointer);
											}
									}
								if(buff_pointer[4]==0x00)
									{
										if(buff_pointer[5]==0x01)
											{
												deal_mcu_control_to_initiative(buff_pointer);
												report_fuction1();
											}
									}
								
									
							}
						buff_pointer=buff_pointer+Length1;
					}
				
				
		   } 
		  
		}

}


/************************ 
*多线程处理函数
*Author:  mleaf_hexi
*Mail:350983773@qq.com
*************************/
/********************************************************************************************************
多线程调试记录:
1:使用信号触发方式接收串口 会导致多线程只运行一次就不能接收串口数据了。目前原因不明。
2:在线程函数中通过select系统调用进行io多路切换，实现异步读取串口数据，能稳定接收数据，保持串口不掉。
*********************************************************************************************************/
void* Serial_Read_Thread(void)
{	

	while(1)
	{	
		
		serial_select_receive();//select 方式异步获取串口数据
		
	}


}





/*************************************************************************************
rfid判断函数

*************************************************************************************/

static char *strupr(char *str)
{
   char *p = str;
   while (*p != '\0')
   {
      if(*p >= 'a' && *p <= 'z')
      *p -= 0x20;
      p++;
    }
   return str;
}

static unsigned char String2hex(char* str)
{
	unsigned long var=0;
	unsigned long t;
	unsigned char var1;
	int len = strlen(str);
	if (var > 8) //最长8位
	return -1;
	strupr(str);//统一大写 不然转换会出错
	for (; *str; str++)
	{
	if (*str>='A' && *str <='F')
	t = *str-55;//a-f之间的ascii与对应数值相差55 如'A'为65,65-55即为A
	else
	t = *str-48;
	var<<=4;
	var|=t;
	}
	var1=var;
	/*if((str[0]<='F')&&(str[0]>='A'))
		{
			t=str[0]-55;
		}
	else
		{
			t=str[0]-87;
		}
	t=t<<4;
	var1=t;
	if((str[1]<='F')&&(str[1]>='A'))
			{
				t=str[1]-55;
			}
	else
			{
				t=str[1]-87;
			}
	var1|=t;*/

	
	return var1;
}
int rfid_opensystem_report_log(struct k4_rfid_card_id *b)
{
	char rfidid[36]={0};
	json_object *sendbuf;
	char getstring[1024]={0};
	sprintf(rfidid,"%02x%02x%02x%02x",b->id[0],b->id[1],b->id[2],b->id[3]);
	sendbuf=json_object_new_object();
	json_object_object_add(sendbuf,"function",json_object_new_string("reportrfidlog"));
	json_object_object_add(sendbuf,"rfidid",json_object_new_string(rfidid));
	json_object_object_add(sendbuf,"devicename",json_object_new_string(base_info.name));
	json_object_object_add(sendbuf,"devicemac",json_object_new_string(base_info.id));
	json_object_object_add(sendbuf,"rfididname",json_object_new_string(b->name));
	json_object_object_add(sendbuf,"rfidoperationtype",json_object_new_string("opensystem"));
	sprintf(getstring,"%s",json_object_to_json_string(sendbuf));
	if(nopoll_info.nopoll_isok_flag==1)
		{
			if(nopoll_conn_is_ready(nopoll_info.conn))
				{
					if(nopoll_conn_send_text(nopoll_info.conn,getstring,strlen(getstring))!=strlen(getstring))
						{
							printf("report rfid open system log  false\n");
							
						}
				}
		}
	json_object_put(sendbuf);
	return 1;
}
static int rfid_decision(char *data)
{
	struct k4_rfid_card_id *b;
	unsigned char getdata[4];
	int status=1;
	memmove(getdata,data,4);
	if(rfid_info.number_local>0)
		{
			b=rfid_info.next_local;
			while(1)
				{
					printf("b-> card id =%02x%02x%02x%02x\n",b->id[0],b->id[1],b->id[2],b->id[3]);
					printf("data card id =%02x%02x%02x%02x\n",getdata[0],getdata[1],getdata[2],getdata[3]);
					if((b->del!=1)&&(b->id[0]==getdata[0])&&(b->id[1]==getdata[1])&&(b->id[2]==getdata[2])&&(b->id[3]==getdata[3]))
						{
							status=2;
							if(device_status.k4==0)
								{
									rfid_opensystem_report_log(b);
								}
							break;
						}
					if(b->last==1)
						{
							break;
						}
					b=b->next;
					
				}
		}
	if((rfid_info.number_net>0)&&(status==1))
		{
			b=rfid_info.next_net;
			while(1)
				{
					//printf("b-> card id =%02x%02x%02x%02x\n",b->id[0],b->id[1],b->id[2],b->id[3]);
					if((b->del!=1)&&(b->id[0]==getdata[0])&&(b->id[1]==getdata[1])&&(b->id[2]==getdata[2])&&(b->id[3]==getdata[3]))
						{
							status=2;
							if(device_status.k4==0)
								{
									rfid_opensystem_report_log(b);
								}
							break;
						}
					if(b->last==1)
						{
							break;
						}
					b=b->next;
				}
		}
	printf("status = %d\n",status);
	return status;
}

/*************************************************************************************
luci通信函数

*************************************************************************************/


int luci_socket_init(void)
{
	int reuse = 1;//默认值设置为1 设置为0的时候通信异常
		memset(&luci_socket_info.my_addr,0,sizeof(luci_socket_info.my_addr)); //数据初始化--清零	
		luci_socket_info.my_addr.sin_family=AF_INET; //设置为IP通信	
		luci_socket_info.my_addr.sin_addr.s_addr=INADDR_ANY;//服务器IP地址--允许连接到所有本地地址上  
		luci_socket_info.my_addr.sin_port=htons(7574); //服务器端口号  
		  
		/*创建服务器端套接字--IPv4协议，面向连接通信，TCP协议*/  
		if((luci_socket_info.server_sockfd=socket(PF_INET,SOCK_STREAM,0))<0)  
		{	 
			perror("socket");  
			return -1;  
		}  

	   
	   /*
	   错误提示
	   bind: Address already in use
	   
	   root@Brxy:~# netstat –apn | grep 8384
		tcp       17      0 localhost:8384          localhost:40018         CLOSE_WAIT  
		tcp       30      0 localhost:8384          localhost:40021         CLOSE_WAIT  
		tcp       17      0 localhost:8384          localhost:40020         CLOSE_WAIT  
		tcp       30      0 localhost:8384          localhost:40023         CLOSE_WAIT  
		tcp       30      0 localhost:8384          localhost:40019         CLOSE_WAIT  
		tcp       17      0 localhost:8384          localhost:40022         CLOSE_WAIT

	   有方法可以避开 TIME_WAIT状态。
	   可以给套接字应用 SO_REUSEADDR 套接字选项，
	   以便端口可以马上重用。
	   */
       //函数原型 int setsockopt( int s, int level,  int optname,   const void * optval,  socklen_t optlen );
		if (setsockopt(luci_socket_info.server_sockfd,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse)) < 0)
		{
	        perror("setsockopet error\n");
	        return -1;
		}
			/*将套接字绑定到服务器的网络地址上*/  
		if (bind(luci_socket_info.server_sockfd,(struct sockaddr *)&luci_socket_info.my_addr,sizeof(struct sockaddr))<0)	
		{  
			perror("bind");  
			return -1;  
		} 
		return 1;
}

int int_to_str(int indata,char *outdata)
{
	char data[10]={0};
	int i=0;
	int j=0;
	int number;
	int y;
	int s;
	number=indata;
	
	while(1)
		{
			y=number%10;
			s=number/10;
			data[j]=y+48;
			number=s;
			if((number==0)||(j>10))
				{
					break;
				}
			j++;
		}
	for(i=0;j>=0;i++,j--)
		{
			*(outdata+i)=data[j];
		}
	return 1;
	
}

int read_rfid_id(char *sendbuf)
{
	unsigned char op1[8]={0};
	unsigned char op2[2]={0};
	unsigned char op3[4]={0};
	int i;
	unsigned int op4;
	unsigned int op5;
	unsigned int op6;
	int ret;
	//pthread_mutex_lock(&i2c_mutex);
	//ret=rfid_read();
	//pthread_mutex_unlock(&i2c_mutex);


	pthread_mutex_lock(&projector_info.mutex);
	//projector_info.send_return_status=0;
	rfid_info.read_card_id_return=0;
	uart_read_status(0x20);
	usleep(200000);
	if(rfid_info.read_card_id_return==1)
		{
			ret=1;
		}
	else
		{
			ret=0;
		}


	//projector_info.send_return_status=1;
	pthread_mutex_unlock(&projector_info.mutex);

	
	if(ret==1)
		{
			/*memmove(op3,rfid_info.rfid_card_id,4);
			for(i=0;i<4;i++)
				{
					op4=op3[i];
					op5=op4&0x0000000f;
					op6=(op4>>4)&0x0000000f;
					if((op5>=0)&&(op5<10))
						{
							op1[i*2]=op5+48;
						}
					else if((op5>9)&&(op5<16))
						{
							op1[i*2]=op5+55;
						}
					if((op6>=0)&&(op6<10))
						{
							op1[i*2+1]=op6+48;
						}
					else if((op6>9)&&(op6<16))
						{
							op1[i*2+1]=op6+55;
						}
						
				}
			strcpy(sendbuf,op1);*/
			

			
			memmove(op3,rfid_info.rfid_card_id,4);
			sprintf(sendbuf,"%02x%02x%02x%02x",rfid_info.rfid_card_id[0],rfid_info.rfid_card_id[1],rfid_info.rfid_card_id[2],rfid_info.rfid_card_id[3]);
		}
	else
		{
			strcpy(sendbuf,"false");
		}
	return 1;
}
int get_sn_license(char *sendbuf)
{
	char *header="SETD";
	char *cut="B@_R@_X@_Y@_C@_U@_T@";
	strcpy(sendbuf,header);
	strcat(sendbuf,cut);

	strcat(sendbuf,"getDeviceSN");
	strcat(sendbuf,cut);
	strcat(sendbuf,base_info.sn);
	strcat(sendbuf,cut);
	if(base_info.qqlicense[1023]==0)
		{
			strcat(sendbuf,"getDevicelicense");
			strcat(sendbuf,cut);
			if(base_info.qq_init_success==1)
				{
					strcat(sendbuf,base_info.qqlicense);
				}
			else
				{
					strcat(sendbuf,"000");
				}
			strcat(sendbuf,cut);
		}
	else
		{
			base_info.qqlicense[1023]=0;
			strcat(sendbuf,"getDevicelicense");
			strcat(sendbuf,cut);
			strcat(sendbuf,"000");
			strcat(sendbuf,cut);
		}

	

	strcat(sendbuf,"nothing");
	printf("sendbuf=  %s\n",sendbuf);
	return 1;
}

int get_rfid_config(char *sendbuf)
{
	char *header="SETD";
	char *cut="B@_R@_X@_Y@_C@_U@_T@";
	char opentime1[11]={0};
	int ot;
	char workmode1[2]={0};

	strcpy(sendbuf,header);
	strcat(sendbuf,cut);



	int opentime;
	int workmode;

	workmode1[0]=rfid_info.workmode+48;
	ot=rfid_info.opentime/60;
	int_to_str(ot,opentime1);

	
	strcat(sendbuf,"RFIDWorkMode");
	strcat(sendbuf,cut);
	strcat(sendbuf,workmode1);
	strcat(sendbuf,cut);

	strcat(sendbuf,"RFIDWorkTime");
	strcat(sendbuf,cut);
	strcat(sendbuf,opentime1);
	strcat(sendbuf,cut);
	

	strcat(sendbuf,"nothing");
	printf("sendbuf=  %s\n",sendbuf);
	return 1;


	
}



int get_rs232_config(char *sendbuf)
{
	char *header="SETD";
	char *cut="B@_R@_X@_Y@_C@_U@_T@";


	char bundrate1[11]={0};
	char databyte1[2]={0};
	char check1[2]={0};
	char stop1[2]={0};
	char start_delay1[11]={0};
	char shutdown_delay1[11]={0};
	char interval1[11]={0};
	strcpy(sendbuf,header);
	strcat(sendbuf,cut);


	
	int_to_str(projector_info.baudrate,bundrate1);
	printf("bundrate = %s\n",bundrate1);
	int_to_str(projector_info.start_delay,start_delay1);
	int_to_str(projector_info.shutdown_delay,shutdown_delay1);
	int_to_str(projector_info.interval,interval1);

	databyte1[0]=projector_info.databyte+48;
	check1[0]=projector_info.check;
	stop1[0]=projector_info.stop+48;
	
	strcat(sendbuf,"baudRate");
	strcat(sendbuf,cut);
	strcat(sendbuf,bundrate1);
	strcat(sendbuf,cut);

	strcat(sendbuf,"checkSum");
	strcat(sendbuf,cut);
	strcat(sendbuf,check1);
	strcat(sendbuf,cut);

	strcat(sendbuf,"dataBits");
	strcat(sendbuf,cut);
	strcat(sendbuf,databyte1);
	strcat(sendbuf,cut);

	strcat(sendbuf,"stopBits");
	strcat(sendbuf,cut);
	strcat(sendbuf,stop1);
	strcat(sendbuf,cut);

	strcat(sendbuf,"projectorOn");
	strcat(sendbuf,cut);
	strcat(sendbuf,projector_info.start_code);
	strcat(sendbuf,cut);
	printf("projector_info.start_code=%s\n",projector_info.start_code);
	strcat(sendbuf,"projectorOff");
	strcat(sendbuf,cut);
	strcat(sendbuf,projector_info.shutdown_code);
	strcat(sendbuf,cut);

	strcat(sendbuf,"projectorChannel");
	strcat(sendbuf,cut);
	strcat(sendbuf,projector_info.show_channel);
	strcat(sendbuf,cut);

	strcat(sendbuf,"projectorOpenDelay");
	strcat(sendbuf,cut);
	strcat(sendbuf,start_delay1);
	strcat(sendbuf,cut);

	strcat(sendbuf,"projectorOperationIntervalTime");
	strcat(sendbuf,cut);
	strcat(sendbuf,interval1);
	strcat(sendbuf,cut);

	strcat(sendbuf,"projectorCoolingDelay");
	strcat(sendbuf,cut);
	strcat(sendbuf,shutdown_delay1);
	strcat(sendbuf,cut);



	strcat(sendbuf,"nothing");
	printf("sendbuf=  %s\n",sendbuf);
	return 1;
}

int get_device_config(char *sendbuf)
{
	char *header="SETD";
	char *cut="B@_R@_X@_Y@_C@_U@_T@";
	

	char isPCAutoStartUp[2]={0};
	char isButtonStartUp[2]={0};
	char volume[2]={0};
	char nopollstatus[2]={0};
	nopollstatus[0]=nopoll_info.nopoll_isok_flag+48;

	strcpy(sendbuf,header);
	strcat(sendbuf,cut);


	
	isPCAutoStartUp[0]=base_info.pc_automatic+48;
	isButtonStartUp[0]=base_info.openlimit+48;
	volume[0]=volume_3105.volume_default+48;
	strcat(sendbuf,"deviceName");
	strcat(sendbuf,cut);
	strcat(sendbuf,base_info.name);
	strcat(sendbuf,cut);


	strcat(sendbuf,"deviceMac");
	strcat(sendbuf,cut);
	strcat(sendbuf,base_info.id);
	strcat(sendbuf,cut);

	strcat(sendbuf,"isPCAutoStartUp");
	strcat(sendbuf,cut);
	strcat(sendbuf,isPCAutoStartUp);
	strcat(sendbuf,cut);

	strcat(sendbuf,"isButtonStartUp");
	strcat(sendbuf,cut);
	strcat(sendbuf,isButtonStartUp);
	strcat(sendbuf,cut);

	strcat(sendbuf,"volume");
	strcat(sendbuf,cut);
	strcat(sendbuf,volume);
	strcat(sendbuf,cut);
	
	strcat(sendbuf,"serverIP");
	strcat(sendbuf,cut);
	strcat(sendbuf,base_info.ip);
	strcat(sendbuf,cut);
	
	strcat(sendbuf,"deviceStatus");
	strcat(sendbuf,cut);
	strcat(sendbuf,nopollstatus);
	strcat(sendbuf,cut);

	strcat(sendbuf,"mediaa");
	strcat(sendbuf,cut);
	strcat(sendbuf,base_info.mediaa);
	strcat(sendbuf,cut);

	strcat(sendbuf,"mediab");
	strcat(sendbuf,cut);
	strcat(sendbuf,base_info.mediab);
	strcat(sendbuf,cut);

	strcat(sendbuf,"nothing");
	printf("sendbuf=  %s\n",sendbuf);
	return 1;
}




int get_all_data(char *sendbuf)
{
	struct uci_package *pkg = NULL;
	struct uci_section *section;
	struct uci_option *option;
	struct uci_context *ctx = NULL;
	char *uci_name,*uci_id,*uci_poe,*uci_openlimit,*uci_pc_automatic;
	char *uci_baudrate,*uci_databyte,*uci_chech,*uci_stop;
	char *uci_start_code,*uci_shutdown_code,*uci_show_channel;
	char *uci_start_delay,*uci_shutdown_delay,*uci_interval;
	char *uci_high,*uci_low,*uci_volume;
	char *uci_section1,*uci_opentime;



	char *header="SETD";
	char *cut="B@_R@_X@_Y@_C@_U@_T@";
	strcpy(sendbuf,header);
	strcat(sendbuf,cut);
	
	strcat(sendbuf,"deviceMac");
	strcat(sendbuf,cut);
	strcat(sendbuf,base_info.id);
	strcat(sendbuf,cut);


	
	if(NULL==(ctx=uci_alloc_context()))
		{
			printf("get ctx for uci false\n");
			return -1;
		}
	
	if (UCI_OK!= uci_load(ctx, brxy, &pkg))
	{
		printf("init uci load false\n");
        goto loaderr;
	}

	//初始化base_info
	if(NULL==(section=uci_lookup_section(ctx,pkg,"baseinfo")))
		{
			printf("get baseinfo false\n");
			goto err;
		}
	else
		{
			if(NULL==(uci_name=uci_lookup_option_string(ctx,section,"name")))
				{
					printf("get name false\n");
					goto err;
				}
			else
				{
					if(strcmp(uci_name,"9999999999-9999999999"))
						{
							//strcpy(base_info.name,base_info.id);
							strcat(sendbuf,"deviceName");
							strcat(sendbuf,cut);
							strcat(sendbuf,base_info.id);
							strcat(sendbuf,cut);
						}
					else
						{
							//strcpy(base_info.name,uci_name);
							strcat(sendbuf,"deviceName");
							strcat(sendbuf,cut);
							strcat(sendbuf,uci_name);
							strcat(sendbuf,cut);
						}
					
				}
			if(NULL==(uci_poe=uci_lookup_option_string(ctx,section,"poe")))
				{
					printf("get poe false\n");
					goto err;
				}
			else
				{
					//base_info.poe=atoi(uci_poe);
					strcat(sendbuf,"poe");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_poe);
					strcat(sendbuf,cut);
				}
			if(NULL==(uci_openlimit=uci_lookup_option_string(ctx,section,"openlimit")))
				{
					printf("get openlimit false\n");
					goto err;
				}
			else
				{
					//base_info.openlimit=atoi(uci_openlimit);
					strcat(sendbuf,"openlimit");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_openlimit);
					strcat(sendbuf,cut);
				}
			if(NULL==(uci_pc_automatic=uci_lookup_option_string(ctx,section,"pc_automatic")))
				{
					printf("get openlimit false\n");
					goto err;
				}
			else
				{
					//base_info.pc_automatic=atoi(uci_pc_automatic);
					
					strcat(sendbuf,"isAutoStartUp");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_pc_automatic);
					strcat(sendbuf,cut);
				}
		}
	//初始化projector
	if(NULL==(section=uci_lookup_section(ctx,pkg,"projector")))
		{
			printf("get projector false\n");
			goto err;
		}
	else
		{
			//串口设置参数
			//波特率
			if(NULL==(uci_baudrate=uci_lookup_option_string(ctx,section,"baudrate")))
				{
					printf("get baudrate false\n");
					goto err;
				}
			else
				{
					//projector_info.baudrate=atoi(uci_baudrate);
					strcat(sendbuf,"baudRate");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_baudrate);
					strcat(sendbuf,cut);
				}
			//字节数
			if(NULL==(uci_databyte=uci_lookup_option_string(ctx,section,"databyte")))
				{
					printf("get databyte false\n");
					goto err;
				}
			else
				{
					//projector_info.databyte=atoi(uci_databyte);
					strcat(sendbuf,"dataBits");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_databyte);
					strcat(sendbuf,cut);
				}
			//校验
			if(NULL==(uci_chech=uci_lookup_option_string(ctx,section,"check")))
				{
					printf("get check false\n");
					goto err;
				}
			else
				{
					//projector_info.check=uci_chech[0];
					strcat(sendbuf,"checkSum");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_chech);
					strcat(sendbuf,cut);
				}
			//停止位
			if(NULL==(uci_stop=uci_lookup_option_string(ctx,section,"stop")))
				{
					printf("get stop false\n");
					goto err;
				}
			else
				{
					//projector_info.stop=atoi(uci_stop);
					strcat(sendbuf,"stopBits");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_stop);
					strcat(sendbuf,cut);
				}
			//投影仪码参数
			//开机码
			if(NULL==(uci_start_code=uci_lookup_option_string(ctx,section,"start_code")))
				{
					printf("get uci_start_code false\n");
					goto err;
				}
			else
				{
					//strcpy(projector_info.start_code,uci_start_code);
					strcat(sendbuf,"projectorOn");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_start_code);
					strcat(sendbuf,cut);
					
				}
			//关机吗
			if(NULL==(uci_shutdown_code=uci_lookup_option_string(ctx,section,"shutdown_code")))
				{
					printf("get uci_shutdown_code false\n");
					goto err;
				}
			else
				{
					//strcpy(projector_info.shutdown_code,uci_shutdown_code);
					strcat(sendbuf,"projectorOff");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_shutdown_code);
					strcat(sendbuf,cut);
					
				}
			//通道切换码
			if(NULL==(uci_show_channel=uci_lookup_option_string(ctx,section,"show_channel")))
				{
					printf("get uci_show_channel false\n");
					goto err;
				}
			else
				{
					//strcpy(projector_info.show_channel,uci_show_channel);
					strcat(sendbuf,"projectorChannel");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_show_channel);
					strcat(sendbuf,cut);
					
				}
			//投影仪延迟时间参数
			//开机延迟时间
			if(NULL==(uci_start_delay=uci_lookup_option_string(ctx,section,"start_delay")))
				{
					printf("get uci_start_delay false\n");
					goto err;
				}
			else
				{
					//projector_info.start_delay=atoi(uci_start_delay);
					strcat(sendbuf,"projectorOpenDelay");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_start_delay);
					strcat(sendbuf,cut);
					
				}
			//关机延迟时间
			if(NULL==(uci_shutdown_delay=uci_lookup_option_string(ctx,section,"shutdown_delay")))
				{
					printf("get uci_shutdown_delay false\n");
					goto err;
				}
			else
				{
					//projector_info.shutdown_delay=atoi(uci_shutdown_delay);
					strcat(sendbuf,"projectorCoolingDelay");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_shutdown_delay);
					strcat(sendbuf,cut);
				}
			//间隔时间
			if(NULL==(uci_interval=uci_lookup_option_string(ctx,section,"interval")))
				{
					printf("get uci_interval false\n");
					goto err;
				}
			else
				{
					//projector_info.interval=atoi(uci_interval);
					strcat(sendbuf,"projectorOperationIntervalTime");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_interval);
					strcat(sendbuf,cut);
				}
		}
	//声音初始化
	if(NULL==(section=uci_lookup_section(ctx,pkg,"volume")))
		{
			printf("get volume false\n");
			goto err;
		}
	else
		{
			//高音
			if(NULL==(uci_high=uci_lookup_option_string(ctx,section,"high")))
				{
					printf("get uci_high false\n");
					goto err;
				}
			else
				{
					//volume_3105.high=atoi(uci_high);
					strcat(sendbuf,"volumeHigh");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_high);
					strcat(sendbuf,cut);
				}
			//低音
			if(NULL==(uci_low=uci_lookup_option_string(ctx,section,"low")))
				{
					printf("get uci_low false\n");
					goto err;
				}
			else
				{
					//volume_3105.low=atoi(uci_low);
					strcat(sendbuf,"volumeLow");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_low);
					strcat(sendbuf,cut); 
				}
			//音量
			if(NULL==(uci_volume=uci_lookup_option_string(ctx,section,"volume")))
				{
					printf("get uci_volume false\n");
					goto err;
				}
			else
				{
					//volume_3105.volume=atoi(uci_volume);
					strcat(sendbuf,"volumeTotal");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_volume);
					strcat(sendbuf,cut);
				}
		}
	if(NULL==(section=uci_lookup_section(ctx,pkg,"rfid")))
		{
			printf("get volume false\n");
			goto err;
		}
	else
		{
			if(NULL==(uci_section1=uci_lookup_option_string(ctx,section,"section")))
				{
					printf("get uci_section false\n");
					goto err;
				}
			else
				{
					//rfid_info.section=atoi(uci_section1);
					//RFIDSection=rfid_info.section;
					strcat(sendbuf,"RFIDSctor");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_section1);
					strcat(sendbuf,cut);
				}
			if(NULL==(uci_opentime=uci_lookup_option_string(ctx,section,"opentime")))
				{
					printf("get uci_opentime false\n");
					goto err;
				}
			else
				{
					//rfid_info.opentime=atoi(uci_opentime);
					strcat(sendbuf,"RFIDWorkTime");
					strcat(sendbuf,cut);
					strcat(sendbuf,uci_opentime);
					strcat(sendbuf,cut);
				}
		}
	strcat(sendbuf,"nothing");
	printf("sendbuf=  %s\n",sendbuf);
	
good:
	uci_unload(ctx,pkg);
	uci_free_context(ctx);
	ctx=NULL;
	return 1;
	
loaderr:
	uci_free_context(ctx);
	ctx=NULL;
	return -1;
err:
	uci_unload(ctx,pkg);
	uci_free_context(ctx);
	ctx=NULL;
	return -1;	
}

static void get_the_internetthings(char *sendbuf)
{

	//char *devicenumber,*devicetype,*deviceid,*devicestatus;
	char *nu="1";
	char *cut="B@_R@_X@_Y@_C@_U@_T@";
	int i,j;
	//j=nrow;
	strcpy(sendbuf,"SEIT");
	strcat(sendbuf,cut);

	if(wifi_macheader.number>0)
		{
			struct wifi_internetthings_mac *maca;
			struct wifi_switch *switcha;
			struct wifi_sensor *sensora;
			struct wifi_screen *screena;
			struct wifi_rfid *rfida;
			char *type;
			maca=wifi_macheader.next;
			char sta[2];
			sta[1]='\0';
			while(1)
				{
				if(maca->aready_add==1)
					{
					if(maca->type==1)
						{
							switcha=(struct wifi_switch *)maca->thing;
							sta[0]=switcha->status+48;
							type="Two switches";
						}
					else if(maca->type==2)
						{
							sensora=(struct wifi_sensor *)maca->thing;
							sta[0]=sensora->status+48;
							type="Temperature humidity Illumination";
						}
					else if(maca->type==3)
						{
							screena=(struct wifi_screen *)maca->thing;
							sta[0]=screena->status+48;
							type="Screen";
						}
					else if(maca->type==4)
						{
							rfida=(struct wifi_rfid *)maca->thing;
							sta[0]=rfida->status+48;
							type="RFID";
						}
					if(maca->del==0)
						{
					strcat(sendbuf,nu);
					strcat(sendbuf,cut);
					strcat(sendbuf,type);//type
					strcat(sendbuf,cut);
					strcat(sendbuf,maca->mac);//mac
					strcat(sendbuf,cut);
					strcat(sendbuf,sta);//status
					strcat(sendbuf,cut);
					strcat(sendbuf,maca->name);//status
					strcat(sendbuf,cut);
						}
					
					}
				else
					{
						if(maca->del==0)
							{
						strcat(sendbuf,nu);
						strcat(sendbuf,cut);
						strcat(sendbuf,"unknown");//type
						strcat(sendbuf,cut);
						strcat(sendbuf,maca->mac);//mac
						strcat(sendbuf,cut);
						strcat(sendbuf,"0");//status
						strcat(sendbuf,cut);
						strcat(sendbuf,maca->name);//status
						strcat(sendbuf,cut);
							}
					}
				if(maca->last==1)
						break;
				else
					{
						maca=maca->next;
					}
					
				}
			
			
		}
	

	
	strcat(sendbuf,"nothing");
	printf("internetthings=  %s\n",sendbuf);
}

int get_rfid_card_id(char *sendbuf,char judge)
{
	char *cut="B@_R@_X@_Y@_C@_U@_T@";
	char *header="SETD";
	char *id;
	struct k4_rfid_card_id *a;
	
	strcpy(sendbuf,header);
	strcat(sendbuf,cut);

	if(judge=='0')
		{
			if(rfid_info.number_local>0)
				{
					a=rfid_info.next_local;
					while(1)
						{
							if(a->del!=1)
								{
									strcat(sendbuf,"cardid");
									strcat(sendbuf,cut);
									strcat(sendbuf,a->id1);
									strcat(sendbuf,cut);
									strcat(sendbuf,"usrname");
									strcat(sendbuf,cut);
									strcat(sendbuf,a->name);
									strcat(sendbuf,cut);
								}
							if(a->last==1)
								{
									break;
								}
							a=a->next;
						}
				}
		}
	if(judge=='1')
		{
			if(rfid_info.number_net>0)
				{
					a=rfid_info.next_net;
					while(1)
						{
							if(a->del!=1)
								{
									strcat(sendbuf,"cardid");
									strcat(sendbuf,cut);
									strcat(sendbuf,a->id1);
									strcat(sendbuf,cut);
									strcat(sendbuf,"usrname");
									strcat(sendbuf,cut);
									strcat(sendbuf,a->name);
									strcat(sendbuf,cut);
								}
							if(a->last==1)
								{
									break;
								}
							a=a->next;
						}
				}
		}

	
	strcat(sendbuf,"nothing");
	return 1;
	
}

int add_rfid_card_id(char *data)
{

	char *buf1,*buf2;

	char *cardid,*cardname;
	struct k4_rfid_card_id *a;
	struct k4_rfid_card_id *b;

	int i;
	char change[9]={0};
	char change1[3]={0};
	char change2[4]={0};
	char save_uci[156]={0};
	
	char *name,*value;
	int already_have_rfid=0;
	int cutlen;
	int num=0;
	char *cut="B@_R@_X@_Y@_C@_U@_T@";
	cutlen=strlen(cut);
	buf1=strstr(data,cut);
	buf2=buf1+cutlen;
	while(1)
		{
			
			buf1=strstr(buf2,cut);
			if(buf1==NULL)
				break;
			buf1[0]='\0';
			name=buf2;
			buf2=buf1+cutlen;
			buf1=strstr(buf2,cut);
			if(buf1==NULL)
				break;
			buf1[0]='\0';
			value=buf2;
			buf2=buf1+cutlen;
			if(strcmp(name,"addCardID")==0)
				{
					cardid=value;
				}
			if(strcmp(name,"addCardUsername")==0)
				{
					cardname=value;
					num=1;
				}
		}
	if(strlen(cardid)!=8)
		{
			printf("cardid is not right\n",cardid);
			return -1;
		}
	if(strlen(cardname)>60)
		{
			printf("care name is too long \n");
			return -1;
		}
	
	if(num==1)
		{
			strcpy(change,cardid);
			for(i=0;i<4;i++)
				{
					change1[0]=change[i*2];
					change1[1]=change[i*2+1];
					change2[i]=String2hex(change1);
				}

			
			//a=(struct k4_rfid_card_id *)malloc(sizeof(struct k4_rfid_card_id));
			while((a=(struct k4_rfid_card_id *)malloc(sizeof(struct k4_rfid_card_id)))==NULL)
				{
					printf("malloc false\n");
					sleep(2);
				}
			a->del=0;
			a->last=1;
			memset(a->id,0,4);
			memset(a->name,0,128);
			memmove(a->id,change2,4);
			strcpy(a->name,cardname);

			a->last=1;
			a->del=0;
			memmove(a->id,change2,4);
			a->id[5]=0;
			memset(a->name,0,128);
			memmove(a->id1,change,8);
			a->id1[8]=0;
			strcpy(a->name,cardname);
			if(rfid_info.number_local==0)
				{
					rfid_info.next_local=a;
					rfid_info.number_local++;
				}
			else
				{
					b=rfid_info.next_local;
					while(1)
						{
							
							if((b->del==0)&&(a->id[0]==b->id[0])&&(a->id[1]==b->id[1])&&(a->id[2]==b->id[2])&&(a->id[3]==b->id[3]))
								{
									free(a);
									printf("already have rfid = %s\n",cardid);
									return -1;
									already_have_rfid=1;
									break;
								}
							else if((b->del==1)&&(a->id[0]==b->id[0])&&(a->id[1]==b->id[1])&&(a->id[2]==b->id[2])&&(a->id[3]==b->id[3]))
								{
									
									
									memset(b->id,0,5);
									memmove(b->id,a->id,4);
									b->del=0;
									memset(b->name,0,128);
									strcpy(b->name,a->name);
									//printf("already have rfid = %s\n",cardid);
									free(a);
									//return -1;
									already_have_rfid=0;
									break;
								}
							if(b->last==1)
								{
									b->next=a;
									b->last=0;
									rfid_info.number_local++;
									break;
								}
							b=b->next;
						}
				}
			if(already_have_rfid==0)
				{
					memset(save_uci,0,156);
					strcpy(save_uci,a->id1);
					strcat(save_uci,cardname);
					struct uci_context *context;
					context=uci_alloc_context();
					struct uci_ptr ptr={
							.package="/usr/share/brxydata/card",
							.section="card",
							.option="id",
							.value=save_uci,
							};
					uci_add_list(context,&ptr);
					uci_commit(context,&ptr.p,false);
					uci_free_context(context);
				}
		}
	return 1;
}
int delete_rfid_card_id(char *data)
{


	struct k4_rfid_card_id *a;
	int i;
	unsigned char change[9]={0};
	unsigned char change1[3]={0};
	unsigned char change2[5]={0};
	unsigned char delete_uci[45]={0};
	int num=1;


	if(num==1)
		{
			strcpy(change,data);
			for(i=0;i<4;i++)
				{
					change1[0]=change[i*2];
					change1[1]=change[i*2+1];
					change2[i]=String2hex(change1);
				}

			if(rfid_info.number_local>0)
				{
					a=rfid_info.next_local;
					while(1)
						{
							if((a->del==0)&&(a->id[0]==change2[0])&&(a->id[1]==change2[1])&&(a->id[2]==change2[2])&&(a->id[3]==change2[3]))
								{
									a->del=1;
									num=2;
									break;
								}
							if(a->last==1)
								{
									num=3;
									break;
								}
							a=a->next;
						}
				}
			if(num==2)
				{	
					memset(delete_uci,0,44);
					strcpy(delete_uci,change);
					strcat(delete_uci,a->name);

					struct uci_context *context;
					context=uci_alloc_context();
					struct uci_ptr ptr={
							.package="/usr/share/brxydata/card",
							.section="card",
							.option="id",
							.value=delete_uci,
							};
					uci_del_list(context,&ptr);
					uci_commit(context,&ptr.p,false);
					uci_free_context(context);
				}
		}
}
int cnfig_base_info(char *data)
{
	char *buf1,*buf2;
	char *name,*value;
	char *copypoint;
	int copylength;
	unsigned char volume_default[3]={0};
	int cutlen;
	int num=1;
	char *cut="B@_R@_X@_Y@_C@_U@_T@";
	struct uci_context *context;
	int error_flag=0;
	
	cutlen=strlen(cut);
	buf1=strstr(data,cut);
	buf2=buf1+cutlen;
	while(1)
		{
			
			buf1=strstr(buf2,cut);
			if(buf1==NULL)
				break;
			buf1[0]='\0';
			name=buf2;
			buf2=buf1+cutlen;
			buf1=strstr(buf2,cut);
			if(buf1==NULL)
				break;
			buf1[0]='\0';
			value=buf2;
			buf2=buf1+cutlen;
			if(strcmp(name,"deviceName")==0)
				{

					copypoint=value;
					copylength=0;
					while(1)
						{
							if(copypoint[copylength]==NULL)
								{
									break;
								}
							copylength++;
							if(copylength>123)
								{
									copypoint[copylength]=NULL;
									break;
								}
						}
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="baseinfo",
						.option="name",
						.value=value,
						};
					
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					memset(base_info.name,0,128);
					strcpy(base_info.name,value);
					uci_free_context(context);
				}
			if(strcmp(name,"poe")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="baseinfo",
						.option="poe",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					base_info.poe=atoi(value);
					uci_free_context(context);
				}
			if(strcmp(name,"isButtonStartUp")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="baseinfo",
						.option="openlimit",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					base_info.openlimit=atoi(value);
					uci_free_context(context);
				}
			if(strcmp(name,"isPCAutoStartUp")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="baseinfo",
						.option="pc_automatic",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					base_info.pc_automatic=atoi(value);
					uci_free_context(context);
				}
			if(strcmp(name,"serverIP")==0)
				{
					if(strcmp(base_info.ip,value)!=0)
						{
							copypoint=value;
							copylength=0;
							while(1)
								{
									if(copypoint[copylength]==NULL)
										{
											break;
										}
									copylength++;
									if(copylength>32)
										{
											copypoint[copylength]=NULL;
											break;
										}
								}
							memset(base_info.ip,0,36);
							strcpy(base_info.ip,value);
					
							context=uci_alloc_context();
							struct uci_ptr ptr={
								.package="/usr/share/brxydata/brxy",
								.section="baseinfo",
								.option="serverip",
								.value=value,
								};
							uci_set(context,&ptr);
							uci_commit(context,&ptr.p,false);
							//base_info.pc_automatic=atoi(value);
					
							uci_free_context(context);
							if(nopoll_info.nopoll_isok_flag==1)
								{
									nopoll_loop_stop(nopoll_info.ctx);
									schema_play_info.is_working=0;
									broadcast_play_mp3_info.is_working=0;
								}
						}
					
				}
			if(strcmp(name,"volume")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="volume",
						.option="volume",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					volume_3105.volume_default=atoi(value);
					
					uci_free_context(context);
					//pthread_mutex_lock(&i2c_mutex);
					//i2c_write(i2c_address_wm,0x1c,wm8776_volume_table[volume_3105.volume]);
					//i2c_write(i2c_address_wm,0x15,0x8f);//打开通道 不静音
					//pthread_mutex_unlock(&i2c_mutex);
				}
			if(strcmp(name,"mediaa")==0)
				{
					copypoint=value;
					copylength=0;
					while(1)
						{
							if(copypoint[copylength]==NULL)
								{
									break;
								}
							copylength++;
							if(copylength>32)
								{
									copypoint[copylength]=NULL;
									break;
								}
						}
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="baseinfo",
						.option="mediaa",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					memset(base_info.mediaa,0,36);
					strcpy(base_info.mediaa,value);
					uci_free_context(context);
					//pthread_mutex_lock(&i2c_mutex);
					//i2c_write(i2c_address_wm,0x1c,wm8776_volume_table[volume_3105.volume]);
					//i2c_write(i2c_address_wm,0x15,0x8f);//打开通道 不静音
					//pthread_mutex_unlock(&i2c_mutex);
				}
			if(strcmp(name,"mediab")==0)
				{
					copypoint=value;
					copylength=0;
					while(1)
						{
							if(copypoint[copylength]==NULL)
								{
									break;
								}
							copylength++;
							if(copylength>32)
								{
									copypoint[copylength]=NULL;
									break;
								}
						}
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="baseinfo",
						.option="mediab",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					memset(base_info.mediab,0,36);
					strcpy(base_info.mediab,value);
					
					uci_free_context(context);
					//pthread_mutex_lock(&i2c_mutex);
					//i2c_write(i2c_address_wm,0x1c,wm8776_volume_table[volume_3105.volume]);
					//i2c_write(i2c_address_wm,0x15,0x8f);//打开通道 不静音
					//pthread_mutex_unlock(&i2c_mutex);
				}
			
		}
	pthread_mutex_lock(&projector_info.mutex);
	while(1)
		{
			//pthread_mutex_lock(&projector_info.mutex);
			projector_info.send_return_status=0;
			//mcu_config_volume(volume_3105.volume);
			//uart_send_status(8,volume_3105.volume_default);
			volume_default[0]=volume_3105.volume_default;
			uart_send_config(7,volume_default,1);
			usleep(300000);
			if(projector_info.send_return_status==1)
				{
					break;
				}
			else
				{
					printf("volume\n");
					error_flag=1;
					break;
				}
			projector_info.send_return_status=1;
			//pthread_mutex_unlock(&projector_info.mutex);
		}
	while(1)
		{
			//pthread_mutex_lock(&projector_info.mutex);
			projector_info.send_return_status=0;
			mcu_config_openpermision(base_info.openlimit);
			usleep(300000);
			if(projector_info.send_return_status==1)
				{
					break;
				}
			else
				{
					printf("openlimit\n");
					error_flag=1;
					break;
				}
			projector_info.send_return_status=1;
			//pthread_mutex_unlock(&projector_info.mutex);
		}
	while(1)
		{
			//pthread_mutex_lock(&projector_info.mutex);
			projector_info.send_return_status=0;
			mcu_config_pc_open_pc_automatic(base_info.pc_automatic);
			usleep(300000);
			if(projector_info.send_return_status==1)
				{
					break;
				}
			else
				{
					printf("pc_automatic\n");
					error_flag=1;
					break;
				}
			projector_info.send_return_status=1;
			//pthread_mutex_unlock(&projector_info.mutex);
		}
	
	//uart_read_status(i);
	uart_read_config(0x0c);
	//usleep(200000);
	uart_read_config(0x13);
	//usleep(200000);
	uart_read_config(0x07);
	//usleep(200000);

	//pthread_mutex_lock(&projector_info.mutex);
	//read_all_data_from_mcu();
	//sleep(2);
	pthread_mutex_unlock(&projector_info.mutex);
	
	report_fuction2();
	if(error_flag==1)
		{
			return -1;
		}
	return 1;
}
int cnfig_projector(char *data)
{
	char *buf1,*buf2;
	char *name,*value;
	int cutlen;
	int num=1;
	char *cut="B@_R@_X@_Y@_C@_U@_T@";
	unsigned char sendbuf[64];
	struct uci_context *context;


	unsigned char op[512];
	char ope[3]={0};
	int i=0;
	int length=0;
	int error_flag=0;

	
	cutlen=strlen(cut);
	buf1=strstr(data,cut);
	buf2=buf1+cutlen;
	while(1)
		{
			
			buf1=strstr(buf2,cut);
			if(buf1==NULL)
				break;
			buf1[0]='\0';
			name=buf2;
			buf2=buf1+cutlen;
			buf1=strstr(buf2,cut);
			if(buf1==NULL)
				break;
			buf1[0]='\0';
			value=buf2;
			buf2=buf1+cutlen;
			if(strcmp(name,"baudRate")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="projector",
						.option="baudrate",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					
					projector_info.baudrate=atoi(value);
					uci_free_context(context);
				}
			if(strcmp(name,"checkSum")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="projector",
						.option="check",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					projector_info.check=value[0];
					uci_free_context(context);
					
				}
			if(strcmp(name,"dataBits")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="projector",
						.option="databyte",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);

					projector_info.databyte=atoi(value);
					uci_free_context(context);
				
				}
			if(strcmp(name,"stopBits")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="projector",
						.option="stop",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);

					projector_info.stop=atoi(value);
					uci_free_context(context);
					
				}
			if(strcmp(name,"projectorOn")==0)
				{
					if(strlen(value)>500)
						{
							uart_read_config(0x02);
							uart_read_config(0x04);
							uart_read_config(0x05);
							uart_read_config(0x06);
							uart_read_config(0x0d);
							uart_read_config(0x0e);
							uart_read_config(0x0f);
							return -1;
						}
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="projector",
						.option="start_code",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					memset(projector_info.start_code,0,512);
					strcpy(projector_info.start_code,value);
					uci_free_context(context);
				}
			if(strcmp(name,"projectorOff")==0)
				{
					if(strlen(value)>500)
						{
							uart_read_config(0x02);
							uart_read_config(0x04);
							uart_read_config(0x05);
							uart_read_config(0x06);
							uart_read_config(0x0d);
							uart_read_config(0x0e);
							uart_read_config(0x0f);
							return -1;
						}
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="projector",
						.option="shutdown_code",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					memset(projector_info.shutdown_code,0,512);
					strcpy(projector_info.shutdown_code,value);
					uci_free_context(context);
				}
			if(strcmp(name,"projectorChannel")==0)
				{
					if(strlen(value)>500)
						{
							uart_read_config(0x02);
							uart_read_config(0x04);
							uart_read_config(0x05);
							uart_read_config(0x06);
							uart_read_config(0x0d);
							uart_read_config(0x0e);
							uart_read_config(0x0f);
							return -1;
						}
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="projector",
						.option="show_channel",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					memset(projector_info.show_channel,0,512);
					strcpy(projector_info.show_channel,value);
					uci_free_context(context);
				}
			if(strcmp(name,"projectorOpenDelay")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="projector",
						.option="start_delay",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					projector_info.start_delay=atoi(value);
					uci_free_context(context);
				}
			if(strcmp(name,"projectorCoolingDelay")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="projector",
						.option="shutdown_delay",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					projector_info.shutdown_delay=atoi(value);
					uci_free_context(context);
				}
			if(strcmp(name,"projectorOperationIntervalTime")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="projector",
						.option="interval",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					projector_info.interval=atoi(value);
					uci_free_context(context);
				}
		}


	
	memset(sendbuf,0,64);
	//sendbuf[0]=projector_info.baudrate;

	switch(projector_info.baudrate)
		{
			
			case 9600:
				sendbuf[0]=0;
				break;
			case 14400:
				sendbuf[0]=1;
				break;
			case 19200:
				sendbuf[0]=2;
				break;
			case 38400:
				sendbuf[0]=3;
				break;
			case 57600:
				sendbuf[0]=4;
				break;
			case 115200:
				sendbuf[0]=5;
				break;
			default:
				sendbuf[0]=5;
				break;
		}
	sendbuf[1]=projector_info.databyte;
	if(projector_info.check=='N')
		{
			sendbuf[2]=0;
		}
	else if(projector_info.check=='O')
		{
			sendbuf[2]=1;
		}
	else if(projector_info.check=='E')
		{
			sendbuf[2]=2;
		}
	sendbuf[3]=projector_info.stop-1;
	pthread_mutex_lock(&projector_info.mutex);
	while(1)
		{
			//pthread_mutex_lock(&projector_info.mutex);
			projector_info.send_return_status=0;
			mcu_config_projector_uart_parameter(sendbuf,4);
			usleep(200000);
			if(projector_info.send_return_status==1)
				{
					break;
				}
			else
				{
					error_flag=1;
					break;
				}
			projector_info.send_return_status=1;
			//pthread_mutex_unlock(&projector_info.mutex);
		}


	memset(op,0,512);
	length=strlen(projector_info.start_code);
	if((length==0)||((length%2)>0))
		{
			printf("errer start code\n");
		}
	else
		{
			for(i=0;i<(length/2);i++)
				{
					ope[0]=projector_info.start_code[i*2];
					ope[1]=projector_info.start_code[i*2+1];
					op[i]=String2hex(ope);
					//printf("send projector code  = %02x\n",op[i]);
				}
			while(1)
				{
					//pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_config_projector_start_code(op,length/2);
					usleep(200000);
					if(projector_info.send_return_status==1)
						{
							break;
						}
					else
						{
							error_flag=1;
							break;
						}
					projector_info.send_return_status=1;
					//pthread_mutex_unlock(&projector_info.mutex);
				}
		}

	memset(op,0,512);
	length=strlen(projector_info.shutdown_code);
	if((length==0)||((length%2)>0))
		{
			printf("errer start code\n");
		}
	else
		{
			for(i=0;i<(length/2);i++)
				{
					ope[0]=projector_info.shutdown_code[i*2];
					ope[1]=projector_info.shutdown_code[i*2+1];
					op[i]=String2hex(ope);
					//printf("send projector code  = %02x\n",op[i]);
				}
			while(1)
				{
					//pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_config_projector_shutdown_code(op,length/2);
					usleep(200000);
					if(projector_info.send_return_status==1)
						{
							break;
						}
					else
						{
							error_flag=1;
							break;
						}
					projector_info.send_return_status=1;
					//pthread_mutex_unlock(&projector_info.mutex);
				}
		}
	memset(op,0,512);
	length=strlen(projector_info.show_channel);
	if((length==0)||((length%2)>0))
		{
			printf("errer start code\n");
		}
	else
		{
			for(i=0;i<(length/2);i++)
				{
					ope[0]=projector_info.show_channel[i*2];
					ope[1]=projector_info.show_channel[i*2+1];
					op[i]=String2hex(ope);
					//printf("send projector code  = %02x\n",op[i]);
				}
			while(1)
				{
					//pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_config_projector_channel_code(op,length/2);
					usleep(200000);
					if(projector_info.send_return_status==1)
						{
							break;
						}
					else
						{
							error_flag=1;
							break;
						}
					projector_info.send_return_status=1;
					//pthread_mutex_unlock(&projector_info.mutex);
				}
		}
	
	while(1)
		{
			//pthread_mutex_lock(&projector_info.mutex);
			projector_info.send_return_status=0;
			mcu_config_projector_start_delay(projector_info.start_delay);
			usleep(200000);
			if(projector_info.send_return_status==1)
				{
					break;
				}
			else
				{
					error_flag=1;
					break;
				}
			projector_info.send_return_status=1;
			//pthread_mutex_unlock(&projector_info.mutex);
		}
	while(1)
		{
			//pthread_mutex_lock(&projector_info.mutex);
			projector_info.send_return_status=0;
			mcu_config_projector_stop_delay(projector_info.shutdown_delay);
			usleep(200000);
			if(projector_info.send_return_status==1)
				{
					break;
				}
			else
				{
					error_flag=1;
					break;
				}
			projector_info.send_return_status=1;
			//pthread_mutex_unlock(&projector_info.mutex);
		}
	while(1)
		{
			//pthread_mutex_lock(&projector_info.mutex);
			projector_info.send_return_status=0;
			mcu_config_projector_interval_delay(projector_info.interval);
			usleep(200000);
			if(projector_info.send_return_status==1)
				{
					break;
				}
			else
				{
					error_flag=1;
					break;
				}
			projector_info.send_return_status=1;
			//pthread_mutex_unlock(&projector_info.mutex);
		}
	//pthread_mutex_unlock(&projector_info.mutex);
	
	
	uart_read_config(0x02);
	uart_read_config(0x04);
	uart_read_config(0x05);
	uart_read_config(0x06);
	uart_read_config(0x0d);
	uart_read_config(0x0e);
	uart_read_config(0x0f);
	//pthread_mutex_lock(&projector_info.mutex);
	//usleep(200000);
	//read_all_data_from_mcu();
	//sleep(1);
	pthread_mutex_unlock(&projector_info.mutex);
	//set_serial();
	if(error_flag==1)
		{
			return -1;
		}
	return 1;
}

int cnfig_rfid(char *data)
{
	char *buf1,*buf2;
	char *name,*value;
	int cutlen;
	int num=1;
	unsigned char time=0;
	char *cut="B@_R@_X@_Y@_C@_U@_T@";
	struct uci_context *context;
	
	cutlen=strlen(cut);
	buf1=strstr(data,cut);
	buf2=buf1+cutlen;
	while(1)
		{
			
			buf1=strstr(buf2,cut);
			if(buf1==NULL)
				break;
			buf1[0]='\0';
			name=buf2;
			buf2=buf1+cutlen;
			buf1=strstr(buf2,cut);
			if(buf1==NULL)
				break;
			buf1[0]='\0';
			value=buf2;
			buf2=buf1+cutlen;
			if(strcmp(name,"RFIDWorkMode")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="rfid",
						.option="workmode",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);

					rfid_info.workmode=atoi(value);
					uci_free_context(context);
					//RFIDSection=rfid_info.section;
				}
			if(strcmp(name,"RFIDWorkTime")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="rfid",
						.option="opentime",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					rfid_info.opentime=atoi(value)*60;
					uci_free_context(context);
				}
		}


			
	time=rfid_info.opentime/60;
	//pthread_mutex_unlock(&projector_info.mutex);
	pthread_mutex_lock(&projector_info.mutex);
	uart_send_config(0x22,&time,1);
	uart_read_config(0x22);
	pthread_mutex_unlock(&projector_info.mutex);
	return 1;
}
int cnfig_volume(char *data)
{
	char *buf1,*buf2;
	char *name,*value;
	int cutlen;
	int num=1;
	char *cut="B@_R@_X@_Y@_C@_U@_T@";
	struct uci_context *context;
	int error_flag=0;
	
	cutlen=strlen(cut);
	buf1=strstr(data,cut);
	buf2=buf1+cutlen;
	while(1)
		{
			
			buf1=strstr(buf2,cut);
			if(buf1==NULL)
				break;
			buf1[0]='\0';
			name=buf2;
			buf2=buf1+cutlen;
			buf1=strstr(buf2,cut);
			if(buf1==NULL)
				break;
			buf1[0]='\0';
			value=buf2;
			buf2=buf1+cutlen;
			if(strcmp(name,"volumeHigh")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="volume",
						.option="high",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);

					volume_3105.high=atoi(value);
					uci_free_context(context);
					
				}
			if(strcmp(name,"volumeLow")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="volume",
						.option="low",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					volume_3105.low=atoi(value);
					uci_free_context(context);
				}
			if(strcmp(name,"volumeTotal")==0)
				{
					context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="volume",
						.option="volume",
						.value=value,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					volume_3105.volume=atoi(value);
					uci_free_context(context);
				}
		}
	pthread_mutex_lock(&projector_info.mutex);
	while(1)
		{
			//pthread_mutex_lock(&projector_info.mutex);
			projector_info.send_return_status=0;
			mcu_config_volume(volume_3105.volume);
			usleep(200000);
			if(projector_info.send_return_status==1)
				{
					break;
				}
			else
				{
					error_flag=1;
					break;
				}
			projector_info.send_return_status=1;
			//pthread_mutex_unlock(&projector_info.mutex);
		}
	pthread_mutex_unlock(&projector_info.mutex);
	if(error_flag==1)
		{
			return -1;
		}
	//pthread_mutex_lock(&projector_info.mutex);
	//read_all_data_from_mcu();
	//sleep(2);
	//pthread_mutex_unlock(&projector_info.mutex);
	return 1;
}


int projector_test_send_code(char *code)
{
	unsigned char op[54];
	char ope[3]={0};
	int i=0;
	int length=0;
	memset(op,0,18);
	length=strlen(code);
	if((length==0)||((length%2)>0))
		{
			return -1;
		}
	//tca_set_bit(4);

			for(i=0;i<(length/2);i++)
				{
					ope[0]=code[i*2];
					ope[1]=code[i*2+1];
					op[i]=String2hex(ope);
				}
			if(write(projector_info.serial_fd,op,(length/2))!=(length/2))
				{
					tcflush(projector_info.serial_fd, TCOFLUSH);
				}
			else
				{
					printf("test code send success\n");
				}
			
			//device_status.projector_flag=0;

		
	return 1;
}

int config_license(char *data)
{
	struct uci_context *context;
	int re;
	if(strlen(data)>500)
		{
			printf("error: qqlicense is too long\n");
			return -1;
		}
	if(qq_license_verification(data)==1)
		{
			printf("config_license qqlicense verficastion success\n");
			if(base_info.qqlicense[1023]==0)
				{
					if(1!=0)
						{
							memset(base_info.qqlicense,0,1024);
							strcpy(base_info.qqlicense,data);
							/*context=uci_alloc_context();
							struct uci_ptr ptr={
								.package="/usr/share/brxydata/brxy",
								.section="baseinfo",
								.option="qqlicense",
								.value=base_info.qqlicense,
								};
							uci_set(context,&ptr);
							uci_commit(context,&ptr.p,false);
							uci_free_context(context);*/
							
							base_info.qq_init_flag=1;
							//sem_post(&base_info.qq_init_sem_t);
							 pthread_mutex_lock(&projector_info.mutex);
							printf("now to cal send_lisence_to_mcu base_info.qqlicense[1023]==0\n");
							//re=send_lisence_to_mcu(base_info.qqlicense);
							re=send_qq_license_to_mcu_from_uart(base_info.qqlicense);
							 pthread_mutex_unlock(&projector_info.mutex);
							if(re<0)
								{
									printf("send_qq_license_to_mcu_from_uart false\n");
									return -1;
								}
							base_info.have_new_lisence=1;
						}
				}
			else
				{
					memset(base_info.qqlicense,0,1024);
					strcpy(base_info.qqlicense,data);
					/*context=uci_alloc_context();
					struct uci_ptr ptr={
						.package="/usr/share/brxydata/brxy",
						.section="baseinfo",
						.option="qqlicense",
						.value=base_info.qqlicense,
						};
					uci_set(context,&ptr);
					uci_commit(context,&ptr.p,false);
					uci_free_context(context);*/
					
					base_info.qq_init_flag=1;
					//sem_post(&base_info.qq_init_sem_t);
					
				  pthread_mutex_lock(&projector_info.mutex);
					printf("now to cal send_lisence_to_mcu base_info.qqlicense[1023]==0\n");
					//re=send_lisence_to_mcu(base_info.qqlicense);
					re=send_qq_license_to_mcu_from_uart(base_info.qqlicense);
					 pthread_mutex_unlock(&projector_info.mutex);
					 if(re<0)
						{
							printf("send_qq_license_to_mcu_from_uart false\n");
							return -1;
						}
					 base_info.have_new_lisence=1;
				}
			
		}
	else
		{
			printf("config_license qqlicense verficastion false\n");
			return -1;
		}
	pthread_mutex_lock(&projector_info.mutex);
	get_qq_license_from_uart();
	pthread_mutex_unlock(&projector_info.mutex);
	return 1;
	
}


int test_projector_code(char *data)
{	

	int baudrate=0;
	unsigned char checksun='O';
	int stops=0;
	int databits=0;
	unsigned char code[104];
	unsigned char codetype[36];
	unsigned char sendbuf[64];
	char *buf1,*buf2;
	char *name,*value;
	char *id[10]={0};
	char *id1[5]={0};
	char st[3]={0};
	int i;
	int cutlen;
	int num=1;
	char *cut="B@_R@_X@_Y@_C@_U@_T@";
	unsigned int section_back_up;
	int error_flag=0;




	unsigned char op[512];
	char ope[3]={0};
	int length=0;

	memset(code,0,104);
	memset(codetype,0,36);
	section_back_up=RFIDSection;
	cutlen=strlen(cut);
	buf1=strstr(data,cut);
	buf2=buf1+cutlen;
	while(1)
		{
			
			buf1=strstr(buf2,cut);
			if(buf1==NULL)
				break;
			buf1[0]='\0';
			name=buf2;
			buf2=buf1+cutlen;
			buf1=strstr(buf2,cut);
			if(buf1==NULL)
				break;
			buf1[0]='\0';
			value=buf2;
			buf2=buf1+cutlen;
			if(strcmp(name,"baudRate")==0)
				{
					baudrate=atoi(value);
					
				}
			if(strcmp(name,"codeType")==0)
				{
					strcpy(codetype,value);
					
				}
			if(strcmp(name,"checkSum")==0)
				{
					checksun=value;
				}
			if(strcmp(name,"dataBits")==0)
				{
					databits=atoi(value);
				}
			if(strcmp(name,"stopBits")==0)
				{
					stops=atoi(value);
				}
			if(strcmp(name,"code")==0)
				{
					strcpy(code,value);
				}
			
		}
	
	memset(sendbuf,0,64);
	switch(baudrate)
		{
			
			case 9600:
				sendbuf[0]=0;
				break;
			case 14400:
				sendbuf[0]=1;
				break;
			case 19200:
				sendbuf[0]=2;
				break;
			case 38400:
				sendbuf[0]=3;
				break;
			case 57600:
				sendbuf[0]=4;
				break;
			case 115200:
				sendbuf[0]=5;
				break;
			default:
				sendbuf[0]=5;
				break;
		}
	sendbuf[1]=databits;
	if(checksun=='N')
		{
			sendbuf[2]=0;
		}
	else if(checksun=='O')
		{
			sendbuf[2]=1;
		}
	else if(checksun=='E')
		{
			sendbuf[2]=2;
		}
	sendbuf[3]=stops-1;


	pthread_mutex_lock(&projector_info.mutex);
	while(1)
		{
			//pthread_mutex_lock(&projector_info.mutex);
			projector_info.send_return_status=0;
			mcu_config_projector_uart_parameter(sendbuf,4);
			usleep(200000);
			if(projector_info.send_return_status==1)
				{
					break;
				}
			else
				{
					error_flag=1;
					break;
				}
			projector_info.send_return_status=1;
			//pthread_mutex_unlock(&projector_info.mutex);
		}


	memset(op,0,512);
	length=strlen(code);
	if((length==0)||((length%2)>0))
		{
			printf("errer start code\n");
		}
	else
		{
			for(i=0;i<(length/2);i++)
				{
					ope[0]=code[i*2];
					ope[1]=code[i*2+1];
					op[i]=String2hex(ope);
					printf("send projector code  = %02x\n",op[i]);
				}
			while(1)
				{
					//pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_send_code_to_projector(op,length/2);
					usleep(200000);
					if(projector_info.send_return_status==1)
						{
							break;
						}
					else
						{
							error_flag=1;
							break;
						}
					projector_info.send_return_status=1;
					//pthread_mutex_unlock(&projector_info.mutex);
				}
		}

	//pthread_mutex_unlock(&projector_info.mutex);
	memset(sendbuf,0,32);
	switch(projector_info.baudrate)
		{
			
			case 9600:
				sendbuf[0]=0;
				break;
			case 14400:
				sendbuf[0]=1;
				break;
			case 19200:
				sendbuf[0]=2;
				break;
			case 38400:
				sendbuf[0]=3;
				break;
			case 57600:
				sendbuf[0]=4;
				break;
			case 115200:
				sendbuf[0]=5;
				break;
			default:
				sendbuf[0]=5;
				break;
		}
	sendbuf[1]=projector_info.databyte;
	if(projector_info.check=='N')
		{
			sendbuf[2]=0;
		}
	else if(projector_info.check=='O')
		{
			sendbuf[2]=1;
		}
	else if(projector_info.check=='E')
		{
			sendbuf[2]=2;
		}
	sendbuf[3]=projector_info.stop-1;




	
	while(1)
		{
			//pthread_mutex_lock(&projector_info.mutex);
			projector_info.send_return_status=0;
			mcu_config_projector_uart_parameter(sendbuf,4);
			usleep(200000);
			if(projector_info.send_return_status==1)
				{
					break;
				}
			else
				{
					error_flag=1;
					break;
				}
			projector_info.send_return_status=1;
			//pthread_mutex_unlock(&projector_info.mutex);
		}
	pthread_mutex_unlock(&projector_info.mutex);
	if(error_flag==1)
		{
			printf("test projector code false\n");
			return -1;
		}
	
	//sleep(projector_info.interval);
	//set_opt(projector_info.serial_fd,baudrate,databits,checksun,stops);
	//projector_test_send_code(code);
	//set_serial();
	return 1;

	
}

int write_to_rfid_card(char *data)
{
	char *buf1,*buf2;
	char *name,*value;
	char *id[10]={0};
	char *id1[5]={0};
	char st[3]={0};
	int i;
	int cutlen;
	int num=1;
	char *cut="B@_R@_X@_Y@_C@_U@_T@";
	unsigned int section_back_up;
	section_back_up=RFIDSection;
	cutlen=strlen(cut);
	buf1=strstr(data,cut);
	buf2=buf1+cutlen;
	while(1)
		{
			
			buf1=strstr(buf2,cut);
			if(buf1==NULL)
				break;
			buf1[0]='\0';
			name=buf2;
			buf2=buf1+cutlen;
			buf1=strstr(buf2,cut);
			if(buf1==NULL)
				break;
			buf1[0]='\0';
			value=buf2;
			buf2=buf1+cutlen;
			if(strcmp(name,"section")==0)
				{
					RFIDSection=atoi(value);
					
				}
			if(strcmp(name,"volumeLow")==0)
				{
					strcpy(id,value);
				}
			
		}

	for(i=0;i<5;i++)
		{
			st[0]=id[i*2];
			st[1]=id[i*2+1];
			id1[i]=String2hex(st);
		}
	readbuff[0]=5;
	for(i=0;i<5;i++)
		{
			readbuff[i+1]=id1[i];
		}
	if(RFID_user_write(1)==0)
		{
			return -1;
		}
	RFIDSection=section_back_up;
	return 1;
}

void *luci_socket_thread(void *ptr)
{
	char buf[BUFSIZ];
	char sendbuf[2048];
	int len_sockaddr_in,len;
	int socket_c;
	int re;
	char *deal_success="Success";
	char *deal_false="false";
	while(1)
		{

			memset(buf,0,BUFSIZ);
			listen(luci_socket_info.server_sockfd,5);
			len_sockaddr_in=sizeof(struct sockaddr_in);
			if((socket_c=accept(luci_socket_info.server_sockfd,(struct sockaddr *)&luci_socket_info.remote_addr,&len_sockaddr_in))<0)
				{
					printf("accept socket false\n");
					return;
				}
			if((len=recv(socket_c,buf,BUFSIZ,0))>0)
				{
					printf("now in while\n");
					if(strcmp(buf,"I need all the data from database")==0)
						{
							memset(sendbuf,0,2048);
							printf("get the  database request\n");
							get_all_data(sendbuf);
							send(socket_c,sendbuf, strlen(sendbuf),0);
							//close(socket_c);
							printf("send all config data success\n");
							
						}
					else if(strcmp(buf,"I need getDeviceConfig")==0)
						{
							memset(sendbuf,0,2048);
							printf("get the  database request\n");
							get_device_config(sendbuf);
							send(socket_c,sendbuf, strlen(sendbuf),0);
							//close(socket_c);
							printf("send all config data success\n");
							
						}
					else if(strcmp(buf,"I need getRS232Config")==0)
						{
							memset(sendbuf,0,2048);
							printf("get the  database request\n");
							get_rs232_config(sendbuf);
							send(socket_c,sendbuf, strlen(sendbuf),0);
							//close(socket_c);
							printf("send all config data success\n");
							
						}
					else if(strcmp(buf,"I need getRFIDConfig")==0)
						{
							memset(sendbuf,0,2048);
							printf("get the  database request\n");
							get_rfid_config(sendbuf);
							send(socket_c,sendbuf, strlen(sendbuf),0);
							//close(socket_c);
							printf("send all config data success\n");
							
						}
					else if(strcmp(buf,"I need the internetthings")==0)
						{
							memset(sendbuf,0,2048);
							printf("get the  internetthings request\n");
							get_the_internetthings(sendbuf);
							send(socket_c,sendbuf, strlen(sendbuf),0);
							//close(socket_c);
							printf("send internetthings success\n");
							
						}
					else if((buf[0]=='a')&&(buf[1]=='d')&&(buf[2]=='d')&&(buf[3]=='w')&&(buf[4]=='f')&&(buf[5]=='d'))
						{
							printf("add IOT device = %s\n",buf);
							if(base_info.qq_lisence_get_from_mcu_is_ok==1)
								{
									re=add_wifi_things(&buf[6]);
								}
							else
								{
									re==1;
								}
							if(re==0)
								{
									
									
									send(socket_c,deal_success, strlen(deal_success),0);//发送成功信息
									//close(socket_c);
								}
							else
								{
									
									printf("send_to_lua_IOT=%s\n",deal_false);
									send(socket_c,deal_false, strlen(deal_false),0);//发送失败信息
									//close(socket_c);
							
								}
							
						}
					else if((buf[0]=='d')&&(buf[1]=='e')&&(buf[2]=='l')&&(buf[3]=='w')&&(buf[4]=='f')&&(buf[5]=='d'))
						{
							printf("delete Internet Of Things ID=%s\n",buf);
							delete_ito_success_flag=0;
							if(base_info.qq_lisence_get_from_mcu_is_ok==1)
								{
									delet_wifi_things(&buf[6]);//找到数据库中相关数据并删除
								}
							else
								{
									delete_ito_success_flag=0;
								}
							if(delete_ito_success_flag){
								
								printf("send_to_lua_delete_IOT=%s\n",deal_success);
								send(socket_c,deal_success, strlen(deal_success),0);//发送成功信息
								//close(socket_c);
							}
							else{
								
								printf("send_to_lua_IOT=%s\n",deal_false);
								send(socket_c,deal_false, strlen(deal_false),0);//发送失败信息
								//close(socket_c);
							}
							
						}
					else if((buf[0]=='n')&&(buf[1]=='e')&&(buf[2]=='d')&&(buf[3]=='r')&&(buf[4]=='f')&&(buf[5]=='c'))
						{
							printf("%s\n",buf);
							memset(sendbuf,0,2048);
							pthread_mutex_lock(&rfid_card_uci_mutex);//上锁 
							get_rfid_card_id(sendbuf,buf[6]);
							pthread_mutex_unlock(&rfid_card_uci_mutex);//上锁 
							send(socket_c,sendbuf,strlen(sendbuf),0);
							printf("card id = %s\n",sendbuf);
							//close(socket_c);
							
						}
					else if((buf[0]=='a')&&(buf[1]=='d')&&(buf[2]=='d')&&(buf[3]=='r')&&(buf[4]=='f')&&(buf[5]=='c'))
						{
							printf("add card = %s\n",buf);
							if(base_info.qq_lisence_get_from_mcu_is_ok==1)
								{
									pthread_mutex_lock(&rfid_card_uci_mutex);//上锁 
									re=add_rfid_card_id(&buf[6]);
									pthread_mutex_unlock(&rfid_card_uci_mutex);//上锁 
									if(re>0)
										{
											printf("add rfid card success\n");
											send(socket_c,deal_success,strlen(deal_success),0);
										}
									else
										{
											printf("add rfid card false\n");
											send(socket_c,deal_false,strlen(deal_false),0);
										}
									//send(socket_c,deal_success,strlen(deal_success),0);
								}
							else
								{
									send(socket_c,deal_false,strlen(deal_false),0);
								}
							
							//close(socket_c);
							
							
						}
					else if((buf[0]=='d')&&(buf[1]=='e')&&(buf[2]=='l')&&(buf[3]=='r')&&(buf[4]=='f')&&(buf[5]=='c'))
						{
							printf("delete card = %s\n",buf);
							if(base_info.qq_lisence_get_from_mcu_is_ok==1)
								{
									pthread_mutex_lock(&rfid_card_uci_mutex);//上锁 
									delete_rfid_card_id(&buf[6]);
									pthread_mutex_unlock(&rfid_card_uci_mutex);//上锁 
									send(socket_c,deal_success,strlen(deal_success),0);
								}
							else
								{
									send(socket_c,deal_false,strlen(deal_false),0);
								}
							
							//close(socket_c);
							
						}
					else if((buf[0]=='c')&&(buf[1]=='f')&&(buf[2]=='g')&&(buf[3]=='b')&&(buf[4]=='s')&&(buf[5]=='i'))
						{
							printf("config base info =%s\n",buf);
							
							if(base_info.qq_lisence_get_from_mcu_is_ok==1)
								{
									if(cnfig_base_info(&buf[6])<0)
										{
											printf("config base info false\n");
											send(socket_c,deal_false,strlen(deal_false),0);
										}
									else
										{
											
											send(socket_c,deal_success,strlen(deal_success),0);
											printf("config base info success\n");
										}
									
								}
							else
								{
									send(socket_c,deal_false,strlen(deal_false),0);
								}
						
							//close(socket_c);
							
						}
					else if((buf[0]=='c')&&(buf[1]=='f')&&(buf[2]=='g')&&(buf[3]=='p')&&(buf[4]=='r')&&(buf[5]=='j'))
						{
							printf("config projector info =%s\n",buf);
							
							if(base_info.qq_lisence_get_from_mcu_is_ok==1)
								{
									//cnfig_projector(&buf[6]);
									//send(socket_c,deal_success,strlen(deal_success),0);
									if(cnfig_projector(&buf[6])<0)
										{
											printf("cnfig_projector false\n");
											send(socket_c,deal_false,strlen(deal_false),0);
										}
									else
										{
											
											send(socket_c,deal_success,strlen(deal_success),0);
											printf("cnfig_projector success\n");
										}
								}
							else
								{
									send(socket_c,deal_false,strlen(deal_false),0);
								}
							
							//close(socket_c);
							
						}
					else if((buf[0]=='c')&&(buf[1]=='f')&&(buf[2]=='g')&&(buf[3]=='r')&&(buf[4]=='f')&&(buf[5]=='i'))
						{
							printf("config rfid info =%s\n",buf);
							
							if(base_info.qq_lisence_get_from_mcu_is_ok==1)
								{
									cnfig_rfid(&buf[6]);
									send(socket_c,deal_success,strlen(deal_success),0);
									
								}
							else
								{
									send(socket_c,deal_false,strlen(deal_false),0);
								}
							
							//close(socket_c);
							
						}
					else if((buf[0]=='c')&&(buf[1]=='f')&&(buf[2]=='g')&&(buf[3]=='r')&&(buf[4]=='f')&&(buf[5]=='i'))
						{
							printf("config volume info =%s\n",buf);
							
							if(base_info.qq_lisence_get_from_mcu_is_ok==1)
								{
									//cnfig_volume(&buf[6]);
									//send(socket_c,deal_success,strlen(deal_success),0);
									if(cnfig_volume(&buf[6])<0)
										{
											printf("cnfig_volume false\n");
											send(socket_c,deal_false,strlen(deal_false),0);
										}
									else
										{
											
											send(socket_c,deal_success,strlen(deal_success),0);
											printf("cnfig_volume success\n");
										}
								}
							else
								{
									send(socket_c,deal_false,strlen(deal_false),0);
								}
							
							//close(socket_c);
							
						}
					else if((buf[0]=='t')&&(buf[1]=='e')&&(buf[2]=='s')&&(buf[3]=='t')&&(buf[4]=='p')&&(buf[5]=='r'))
						{
							printf("test projector code =%s\n",buf);
							
							if(base_info.qq_lisence_get_from_mcu_is_ok==1)
								{
									test_projector_code(&buf[6]);
								}
							
							
						}
					else if((buf[0]=='c')&&(buf[1]=='f')&&(buf[2]=='g')&&(buf[3]=='r')&&(buf[4]=='f')&&(buf[5]=='i'))
						{
							printf("write rfie card  =%s\n",buf);
							if(base_info.qq_lisence_get_from_mcu_is_ok==1)
								{
									pthread_mutex_lock(&i2c_mutex);
									re=write_to_rfid_card(&buf[6]);
									pthread_mutex_unlock(&i2c_mutex);
								}
							else
								{
									re=0;
								}
							
							
							if(re>0)
								{
									printf("write rfid card success\n");
									send(socket_c,deal_success,strlen(deal_success),0);
								}
							else
								{
									send(socket_c,deal_false,strlen(deal_false),0);
								}
						}	
					else if(strcmp(buf,"I need readCard")==0)
						{
							memset(sendbuf,0,2048);
							printf("need readCard request\n");
							read_rfid_id(sendbuf);
							printf("read card id = %s",sendbuf);
							send(socket_c,sendbuf, strlen(sendbuf),0);
						
							
							//close(socket_c);
							printf("send readCard  data success\n");
							
						}
					else if(strcmp(buf,"I need sn and license")==0)
						{
							memset(sendbuf,0,2048);
							printf("need sn and lisence\n");
							get_sn_license(sendbuf);
							send(socket_c,sendbuf,strlen(sendbuf),0);
							printf("send sn and lisence data success\n");
						}
					else if((buf[0]=='c')&&(buf[1]=='f')&&(buf[2]=='g')&&(buf[3]=='l')&&(buf[4]=='c')&&(buf[5]=='s'))
						{
							printf("config license =%s\n",buf);
							/*if(config_license(&buf[6])>0)
								{
									send(socket_c,deal_success,strlen(deal_success),0);
									printf("config qqlicense success\n");
								}
							else
								{
									send(socket_c,deal_false,strlen(deal_false),0);
									printf("config qqlicense false\n");
								}*/
							send(socket_c,deal_false,strlen(deal_false),0);
						}
					
				}
			close(socket_c);
		}
}









/*************************************************************************************
wifi物联网用到的函数和被其它线程调用的函数

*************************************************************************************/

static void wifi_rfid_data_return(char *ip)
{
	CURL *curl;
    CURLcode res;
	struct wifi_screen *a;
	char lamp_name[4];
	int i,j,p;
	int control;
	int turnon=1;
	int turnoff=0;
	//json_object **list_object;
	json_object *all;
	json_object *sub;
	int re;
	int nu;
	char data[512];
	char *data_a;
	char url[512];



					memset(data,0,512);
					memset(lamp_name,0,4);
					strcpy(lamp_name,"ep2");
					sub=json_object_new_object();
					all=json_object_new_object();
					//nu=a->lamp_number;
					/*for(i=0;i<nu;i++)
						{
							lamp_name[2]=i+49;
							json_object_object_add(sub,lamp_name,json_object_new_int(control));
						}*/
					json_object_object_add(sub,lamp_name,json_object_new_int(1));
					json_object_object_add(all,"rfid",sub);
					sprintf(data,"%s",json_object_to_json_string(all));
					json_object_put(all);
					json_object_put(sub);


					memset(url,0,512);
					strcpy(url,libcurl_select_header);
					strcat(url,ip);
					strcat(url,libcurl_select_end);
					//strcpy(data,"{\"swtich\":{\"ep1\":1,\"ep2\":1}}");
   					curl = curl_easy_init();

					//去除data中的空格------------------------------------------------------
					data_a=data;
					 for(j=p=0;data_a[p];p++)
					 	{
					 		if(data_a[p]!=' ')
					 			{
					 				data_a[j++]=data_a[p];
					 			}
					 	}
					 data_a[j]='\0';
					 //-----------------------------------------------------------------------


					struct curl_slist *headers=NULL;
					//struct curl_httppost *first=NULL;
					//struct curl_httppost *last=NULL;
					//curl_formadd(&first, &last,CURLFORM_CONTENTTYPE, "application/json", CURLFORM_END);
					headers = curl_slist_append(headers, "Content-Type:application/json");
					printf("++++++++++++++++++++++++++++data= %s\n",data);
					curl_easy_setopt(curl,CURLOPT_TIMEOUT,3);
					curl_easy_setopt(curl,CURLOPT_URL,url);
					curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST,"POST");
					curl_easy_setopt(curl, CURLOPT_POST,1L);
					curl_easy_setopt(curl, CURLOPT_HEADER,1L);
					curl_easy_setopt(curl, CURLOPT_HTTPHEADER,headers);
					curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
					curl_easy_setopt(curl, CURLOPT_POSTFIELDS,data);
					
					curl_easy_setopt(curl, CURLOPT_COOKIEFILE, "/usr/curlposttest.cookie");
					re=curl_easy_perform(curl);
					printf("%d\n",re);
					curl_slist_free_all(headers);
					//curl_formfree(first);
					curl_easy_cleanup(curl);


}

static void init_broadcast_socket(void)
{
	wifi_sock_fd=socket(AF_INET,SOCK_DGRAM,0);
	if(wifi_sock_fd<0)
		{
			perror("socket");
			exit(-1);
			
		}
	memset(&init_wifi_socket,0,sizeof(struct sockaddr_in));
	init_wifi_socket.sin_family=AF_INET;
	init_wifi_socket.sin_port=htons(1025);
	init_wifi_socket.sin_addr.s_addr=htonl(INADDR_ANY);
	/*if (bind(wifi_sock_fd,(struct sockaddr *)&init_wifi_socket,sizeof(struct sockaddr))<0)	
		{  
			perror("bind");  
			return 1;  
		} */
}
/*static void wifi_rfid_data_turn_on(char *data)
{
	int re=0;
	int time_use_able=0;
	time_use_able=get_sqlite3_TimeUseAble_data();
	re=get_sqlite3_RFID_ADMIN(data);
	if(re){
			if(time_use_able==3)
				{
			printf( "you can trun on Machine System\n");
			//serial_send_to_stm32_7(Turn_On_Machine_System);//发送开启系统命令到STM32
			turn_on_off_screen(1);
				}
			else
				{
					printf( "you have no permission to trun on Machine System\n");
				}
		}
//	else{
	//		serial_send_to_stm32_8(Unauthorized);//发送未授权通知
	//		printf( "get none cardId permissions=admin this card can't trun on Machine System\n");
	//	}
}*/



static void *recive_wifi_rfid(void)
{
	char bufre[256];
	int re;
	int len;
	char *mac;
	struct wifi_internetthings_mac *mac_struct;
	while(1)
		{
			if(broadcast_select_turn>0)
				break;
			else
				sleep(1);
		}
	
	
	memset(&wifi_client,0,sizeof(struct sockaddr_in));

	wifi_client.sin_family=AF_INET;
	wifi_client.sin_port=htons(1025);
	wifi_client.sin_addr.s_addr=htonl(INADDR_ANY);
	len=sizeof(wifi_client);
	while(1)
		{
			memset(bufre,0,256);
			re=recvfrom(wifi_sock_fd,(void*)bufre,256,0,(struct sockaddr *)&wifi_client,(socklen_t *)&len);
			if(re==-1)
				{
					printf("recive over time\n");
					return -1;
				}
			else
				{
					printf("get internetthings success\n");
					printf("bufre = %s\n",bufre);
					if(wifi_macheader.number>0)
						{
							if(strlen(bufre)>17)
								{
									deal_with_recive_from_broadcast(bufre,mac,mac_struct);
								}
							else
								{
									printf("recive data from broadcast not adjust\n");
								}
						}
				}
			//rebuf=bufre;
			printf("recive internetthings string is =\n");
		}
}


/*static void *recive_wifi_rfid(void)
{
	int socket_fd,port;
	char bufre[256];
	char *buf;
	char *type;
	char *mac;
	char *rfiddata;
	int count=0;
	char *source;
	int len,re;
	struct wifi_rfid *rfida;
	struct sockaddr_in server,addr;
	if((socket_fd=socket(AF_INET,SOCK_DGRAM,0))<0)
		{
			perror("fail t socket");
			exit(-1);
		}
	memset(&server,0,sizeof(struct sockaddr_in));
	memset(&addr,0,sizeof(struct sockaddr_in));
	server.sin_family = PF_INET;
	server.sin_port = htons(1050);
	server.sin_addr.s_addr=htonl(INADDR_ANY);
		
		//inet_addr("192.168.6.255");
	len=sizeof(addr);
	if(bind(socket_fd,(struct sockaddr *)&server,sizeof(struct sockaddr))<0)
		{
			perror("fail to blind to recive rfid");
			exit(-1);
		}
	while(1)
		{
			printf("now recive from rfid device\n");
			memset(bufre,0,256);
			count=0;
			re=recvfrom(socket_fd,(void*)bufre,256,0,(struct sockaddr *)&addr,(socklen_t *)&len);
			if(strlen(bufre)==0)
				{
					printf("rfid recive length = 0 ");
				}
			else
				{
					printf("recive bufre = %s\n",bufre);
					source=bufre;
					while(1)
						{
							buf=strsep(source," ");
							if(count==0)
								{
									type=buf;
									count++;
								}
							else if(count==1)
								{
									mac=buf;
									count++;
								}
							else if(count==2)
								{
									rfiddata=buf;
									count++;
								}
							if(buf==NULL)
								{
									break;
								}
							if(count==3)
								{
									break;
								}
							
						}
					if(wifi_rfidheader.number>0)
						{
							printf("there is rfid device\n");
							rfida=wifi_rfidheader.next;
							while(1)
								{
									if((strcmp(rfida->mac,mac)==0)&&(rfida->del==0))
										{
											//wifi_rfid_data_turn_on(rfiddata);
											break;
										}
									if(rfida->last==1)
										{
											
											break;
										}
									else
										{
											rfida=rfida->next;
										}
								}
						
						}
					
				}
		}
}*/
static void turn_on_off_screen(int st)
{
	CURL *curl;
    CURLcode res;
	struct wifi_screen *a;
	char lamp_name[4];
	int i,j,p;
	int control;
	int turnon=1;
	int turnoff=0;
	//json_object **list_object;
	json_object *all;
	json_object *sub;
	int re;
	int nu;
	char data[512];
	char *data_a;
	char url[512];
	if(st==1)
		{
			printf("now turn on screen\n");
			control=turnon;
		}
	else
		{
			printf("now turn off screen\n");
			control=turnoff;
		}


	if(wifi_screenheader.number>0)
		{
			a=wifi_screenheader.next;
			while(1)
				{
					if((a->status>0)&&(a->del==0))
						{
					memset(data,0,512);
					memset(lamp_name,0,4);
					strcpy(lamp_name,"ep1");
					sub=json_object_new_object();
					all=json_object_new_object();
					//nu=a->lamp_number;
					/*for(i=0;i<nu;i++)
						{
							lamp_name[2]=i+49;
							json_object_object_add(sub,lamp_name,json_object_new_int(control));
						}*/
					json_object_object_add(sub,lamp_name,json_object_new_int(control));
					json_object_object_add(all,"curtain",sub);
					sprintf(data,"%s",json_object_to_json_string(all));
					json_object_put(all);
					json_object_put(sub);


					memset(url,0,512);
					strcpy(url,libcurl_select_header);
					strcat(url,a->ip);
					strcat(url,libcurl_select_end);
					//strcpy(data,"{\"swtich\":{\"ep1\":1,\"ep2\":1}}");
   					curl = curl_easy_init();

					//去除data中的空格------------------------------------------------------
					data_a=data;
					 for(j=p=0;data_a[p];p++)
					 	{
					 		if(data_a[p]!=' ')
					 			{
					 				data_a[j++]=data_a[p];
					 			}
					 	}
					 data_a[j]='\0';
					 //-----------------------------------------------------------------------


					struct curl_slist *headers=NULL;
					//struct curl_httppost *first=NULL;
					//struct curl_httppost *last=NULL;
					//curl_formadd(&first, &last,CURLFORM_CONTENTTYPE, "application/json", CURLFORM_END);
					headers = curl_slist_append(headers, "Content-Type:application/json");
					printf("++++++++++++++++++++++++++++data= %s\n",data);
					curl_easy_setopt(curl,CURLOPT_TIMEOUT,3);
					curl_easy_setopt(curl,CURLOPT_URL,url);
					curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST,"POST");
					curl_easy_setopt(curl, CURLOPT_POST,1L);
					curl_easy_setopt(curl, CURLOPT_HEADER,1L);
					curl_easy_setopt(curl, CURLOPT_HTTPHEADER,headers);
					curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
					curl_easy_setopt(curl, CURLOPT_POSTFIELDS,data);
					
					curl_easy_setopt(curl, CURLOPT_COOKIEFILE, "/usr/curlposttest.cookie");
					re=curl_easy_perform(curl);
					printf("%d\n",re);
					curl_slist_free_all(headers);
					//curl_formfree(first);
					curl_easy_cleanup(curl);
						}
				
					if(a->last==1)
						break;
					else
						a=a->next;
				}

		}
}
static void set_wifi_device_wificonnect(char *getbuf)
{
	char buf[96];
	char *sep;
	char *source;
	char *ssid;
	char *password;
	char *cut="B@_R@_X@_Y@_C@_U@_T@";
	int cutlen;
	cutlen=strlen(cut);
	source=getbuf;
	
	sep=strstr(source,cut);
	sep[0]='\0';
	ssid=source;
	password=sep+cutlen;
	
}


static int add_wifi_things(char *thing)
{
	struct uci_context *context;
	struct wifi_internetthings_mac *maca;
	struct wifi_internetthings_mac *macb;
	struct wifi_switch *switcha;
	struct wifi_sensor *sensora;
	struct wifi_screen *screena;
	struct wifi_rfid *rfida;
	char buf[128]={0};
	char mac[24]={0};
	char name[104]={0};
	char *na;
	int i;
	int ret=0;
	memset(buf,0,128);
	memset(mac,0,24);
	memset(name,0,104);
	strcpy(buf,thing);
	for(i=0;i<17;i++)
		{
			mac[i]=buf[i];
		}
	na=&buf[17];
	strcpy(name,na);
	

	if(wifi_macheader.number==0)
		{
			ret=0;
		}
	else
		{
			macb=wifi_macheader.next;
			while(1)
				{
					if(strcmp(mac,macb->mac)==0)
						{
							if(macb->del==0)
								{
									ret=1;
								}
							else
								{
									
									memset(macb->name,0,104);
									strcpy(macb->name,name);
									macb->del=0;
									switch(macb->type)
										{
											case 1:
												printf("add switch\n");
												switcha=(struct wifi_switch *)macb->thing;
												memset(switcha->name,0,104);
												strcpy(switcha->name,name);
												switcha->del=0;
												wifi_switchheader.number++;
												wifi_macheader.number++;
												break;
											case 2:
												printf("add switch\n");
												sensora=(struct wifi_sensor *)macb->thing;
												memset(sensora->name,0,104);
												strcpy(sensora->name,name);

												sensora->del=0;
												wifi_sensorheader.number++;
												wifi_macheader.number++;
												break;
											case 3:
												printf("add switch\n");
												screena=(struct wifi_screen *)macb->thing;
												memset(screena->name,0,104);
												strcpy(screena->name,name);

												screena->del=0;
												wifi_screenheader.number++;
												wifi_macheader.number++;
												break;
											case 4:
												printf("add switch\n");
												rfida=(struct wifi_rfid *)macb->thing;
												memset(rfida->name,0,104);
												strcpy(rfida->name,name);

												rfida->del=0;
												wifi_rfidheader.number++;
													
												wifi_macheader.number++;
												break;
											default:
												break;
										}
										/*if(macb->type==1)
											{
												switcha=(struct wifi_switch *)macb->thing;
												switcha->del=0;
											}
										if(macb->type==2)
											{
												sensora=(struct wifi_sensor *)macb->thing;
												sensora->del=0;
											}
										if(macb->type==3)
											{
												
											}
										if(macb->type==4)
											{
												
											}*/
											
									ret=2;
								}


							
							printf("there is a device =%s aready\n",mac);
							break;
						}
					else
						{
							if(macb->last==1)
								{
									break;
								}
							macb=macb->next;
						}
				}
			
		}
	if(ret==0)
		{
			//maca=(struct wifi_internetthings_mac *)malloc(sizeof(struct wifi_internetthings_mac));
			while((maca=(struct wifi_internetthings_mac *)malloc(sizeof(struct wifi_internetthings_mac)))==NULL)
				{
					printf("malloc false\n");
					sleep(2);
				}
			memset(maca,0,sizeof(struct wifi_internetthings_mac));
			strcpy(maca->name,name);
			strcpy(maca->mac,mac);
			maca->aready_add=0;
			maca->status=0;
			maca->last=1;
			maca->del=0;
			
			if(wifi_macheader.number==0)
				{
					wifi_macheader.next=maca;
					wifi_macheader.number++;
				}
			else
				{
					macb=wifi_macheader.next;
					while(1)
						{
							if(macb->last==1)
								{
									macb->next=maca;
									wifi_macheader.number++;
									macb->last=0;
									printf("add things success\n");
									//macb->del=0;
									break;
								}
							else
								{
									macb=macb->next;
								}
						}
				}
		}
	
	printf("add buf =%s\n",thing);

	if((ret==0)||(ret==2))
		{
	context=uci_alloc_context();
	struct uci_ptr ptr={
			.package="/usr/share/brxydata/brxy",
			.section="Iot",
			.option="mac",
			.value=thing,
			};
	uci_add_list(context,&ptr);
	uci_commit(context,&ptr.p,false);
	uci_free_context(context);
		}
	
	add_ito_success_flag=1;
	if(ret==2)
		ret=0;
	return ret;
}



static void delet_wifi_things(char *thing)
{
	struct uci_context *context;
	struct wifi_internetthings_mac *maca;
	struct wifi_internetthings_mac *macb;
	char buf[128]={0};
	char buff[128]={0};
	char mac[24]={0};
	char name[104]={0};
	char *na;
	int i;
	int type=0;
	int ret=0;
	memset(buf,0,128);
	memset(buff,0,128);
	memset(mac,0,24);
	memset(name,0,104);
	strcpy(buf,thing);
	for(i=0;i<17;i++)
		{
			mac[i]=buf[i];
		}
	na=&buf[17];
	//strcpy(name,na);


	if(wifi_macheader.number>0)
		{
			macb=wifi_macheader.next;
			while(1)
				{
					if(strcmp(macb->mac,mac)==0)
						{
							macb->del=1;
							macb->status=0;
							type=macb->type;
							strcpy(name,macb->name);
							
							wifi_macheader.number--;
							printf("//////////////set del = 0");
							break;
						}
					else
						{
							if(macb->last==1)
								break;
							else
								{
									macb=macb->next;
								}
						}
				}
		}
	switch(type)
		{
			case 1:
				printf("now delete switch device %s",mac);
				struct wifi_switch *switcha;
				struct wifi_switch *switchb;
				if(wifi_switchheader.number>0)
					{
						switcha=wifi_switchheader.next;
						while(1)
							{
								if(strcmp(switcha->mac,mac)==0)
									{
										switcha->del=1;
										switcha->status=0;
										
										wifi_switchheader.number--;
										break;
									}
								else
									{
										if(switcha->last==1)
											break;
										else
											{
												switcha=switcha->next;
											}
									}
							}
							
					}
				break;
				case 2:
					printf("now delete sensor device %s",mac);
					struct wifi_sensor *sensora;
					struct wifi_sensor *sensorb;
					if(wifi_sensorheader.number>0)
						{
							sensora=wifi_sensorheader.next;
							while(1)
							{
								if(strcmp(sensora->mac,mac)==0)
									{
										sensora->del=1;
										sensora->status=0;
										
										wifi_sensorheader.number--;
										break;
									}
								else
									{
										if(sensora->last==1)
											break;
										else
											{
												sensora=sensora->next;
											}
									}
							}
								
						}
					break;

				case 3:
					printf("now delete screen device %s",mac);
					struct wifi_screen *screena;
					struct wifi_screen *screenb;
					if(wifi_screenheader.number>0)
						{
							screena=wifi_screenheader.next;
							while(1)
							{
								if(strcmp(screena->mac,mac)==0)
									{
										screena->del=1;
										screena->status=0;
										wifi_screenheader.number--;
										break;
									}
								else
									{
										if(screena->last==1)
											break;
										else
											{
												screena=screena->next;
											}
									}
							}
								
						}
					break;

				case 4:
					printf("now delete rfid device %s",mac);
					struct wifi_rfid *rfida;
					struct wifi_rfid *rfidb;
					if(wifi_rfidheader.number>0)
						{
							rfida=wifi_rfidheader.next;
							while(1)
							{
								if(strcmp(rfida->mac,mac)==0)
									{
										rfida->del=1;
										rfida->status=0;
										wifi_rfidheader.number--;
										break;
									}
								else
									{
										if(rfida->last==1)
											break;
										else
											{
												rfida=rfida->next;
											}
									}
							}
								
						}

				break;
				default:
				break;
				
		}
		
		

	strcpy(buff,mac);
	strcat(buff,name);
	context=uci_alloc_context();
	struct uci_ptr ptr={
		.package="/usr/share/brxydata/brxy",
		.section="Iot",
		.option="mac",
		.value=buff,
		};
	uci_del_list(context,&ptr);
	uci_commit(context,&ptr.p,false);
	uci_free_context(context);
	delete_ito_success_flag=1;
}


/*static void delet_wifi_things(char *thing)
{
	struct uci_context *context;
	context=uci_alloc_context();
	struct uci_ptr ptr={
		.package="/usr/share/brxydata/brxy",
		.section="Iot",
		.option="mac",
		.value=thing,
		};
	uci_del_list(context,&ptr);
	uci_commit(context,&ptr.p,false);
	uci_free_context(context);
	delete_ito_success_flag=1;
}*/
//开关等函数，1开灯，0关灯

void turn_on_off_wifi_switch1(int st,char *mac,int valume)
{
	CURL *curl;
    CURLcode res;
	struct wifi_switch *a;
	char lamp_name[4];
	int i,j,p;
	int control;
	int turnon=1;
	int turnoff=0;
	//json_object **list_object;
	json_object *all;
	json_object *sub;
	int re;
	int nu;
	char data[512];
	char *data_a;
	char url[512];
	if(st==1)
		{
			printf("now turn on lamp\n");
			control=turnon;
		}
	else
		{
			printf("now turn off lamp\n");
			control=turnoff;
		}


	if(wifi_switchheader.number>0)
		{
			a=wifi_switchheader.next;
			while(1)
				{
					if((a->status>0)&&(a->del==0)&&(strcmp(a->mac,mac)==0))
						{
					memset(data,0,512);
					memset(lamp_name,0,4);
					strcpy(lamp_name,"ep");
					sub=json_object_new_object();
					all=json_object_new_object();
					nu=a->lamp_number;
					/*for(i=0;i<nu;i++)
						{
							lamp_name[2]=i+49;
							json_object_object_add(sub,lamp_name,json_object_new_int(control));
						}*/
					lamp_name[2]=valume+49;
					json_object_object_add(sub,lamp_name,json_object_new_int(control));
					json_object_object_add(all,"switch",sub);
					sprintf(data,"%s",json_object_to_json_string(all));
					json_object_put(all);
					json_object_put(sub);


					memset(url,0,512);
					strcpy(url,libcurl_select_header);
					strcat(url,a->ip);
					strcat(url,libcurl_select_end);
					//strcpy(data,"{\"swtich\":{\"ep1\":1,\"ep2\":1}}");
   					curl = curl_easy_init();

					//去除data中的空格------------------------------------------------------
					data_a=data;
					 for(j=p=0;data_a[p];p++)
					 	{
					 		if(data_a[p]!=' ')
					 			{
					 				data_a[j++]=data_a[p];
					 			}
					 	}
					 data_a[j]='\0';
					 //-----------------------------------------------------------------------


					struct curl_slist *headers=NULL;
					//struct curl_httppost *first=NULL;
					//struct curl_httppost *last=NULL;
					//curl_formadd(&first, &last,CURLFORM_CONTENTTYPE, "application/json", CURLFORM_END);
					headers = curl_slist_append(headers, "Content-Type:application/json");
					printf("++++++++++++++++++++++++++++data= %s\n",data);
					curl_easy_setopt(curl,CURLOPT_TIMEOUT,3);
					curl_easy_setopt(curl,CURLOPT_URL,url);
					curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST,"POST");
					curl_easy_setopt(curl, CURLOPT_POST,1L);
					curl_easy_setopt(curl, CURLOPT_HEADER,1L);
					curl_easy_setopt(curl, CURLOPT_HTTPHEADER,headers);
					curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
					curl_easy_setopt(curl, CURLOPT_POSTFIELDS,data);
					
					curl_easy_setopt(curl, CURLOPT_COOKIEFILE, "/usr/curlposttest.cookie");
					re=curl_easy_perform(curl);
					printf("%d\n",re);
					curl_slist_free_all(headers);
					//curl_formfree(first);
					curl_easy_cleanup(curl);
						}
				
					if(a->last==1)
						break;
					else
						a=a->next;
				}

		}
}

static void turn_on_off_wifi_switch(int st)
{
	CURL *curl;
    CURLcode res;
	struct wifi_switch *a;
	char lamp_name[4];
	int i,j,p;
	int control;
	int turnon=1;
	int turnoff=0;
	//json_object **list_object;
	json_object *all;
	json_object *sub;
	int re;
	int nu;
	char data[512];
	char *data_a;
	char url[512];
	if(st==1)
		{
			printf("now turn on lamp\n");
			control=turnon;
		}
	else
		{
			printf("now turn off lamp\n");
			control=turnoff;
		}


	if(wifi_switchheader.number>0)
		{
			a=wifi_switchheader.next;
			while(1)
				{
					if((a->status>0)&&(a->del==0))
						{
					memset(data,0,512);
					memset(lamp_name,0,4);
					strcpy(lamp_name,"ep");
					sub=json_object_new_object();
					all=json_object_new_object();
					nu=a->lamp_number;
					for(i=0;i<nu;i++)
						{
							lamp_name[2]=i+49;
							json_object_object_add(sub,lamp_name,json_object_new_int(control));
						}
					json_object_object_add(all,"switch",sub);
					sprintf(data,"%s",json_object_to_json_string(all));
					json_object_put(all);
					json_object_put(sub);


					memset(url,0,512);
					strcpy(url,libcurl_select_header);
					strcat(url,a->ip);
					strcat(url,libcurl_select_end);
					//strcpy(data,"{\"swtich\":{\"ep1\":1,\"ep2\":1}}");
   					curl = curl_easy_init();

					//去除data中的空格------------------------------------------------------
					data_a=data;
					 for(j=p=0;data_a[p];p++)
					 	{
					 		if(data_a[p]!=' ')
					 			{
					 				data_a[j++]=data_a[p];
					 			}
					 	}
					 data_a[j]='\0';
					 //-----------------------------------------------------------------------


					struct curl_slist *headers=NULL;
					//struct curl_httppost *first=NULL;
					//struct curl_httppost *last=NULL;
					//curl_formadd(&first, &last,CURLFORM_CONTENTTYPE, "application/json", CURLFORM_END);
					headers = curl_slist_append(headers, "Content-Type:application/json");
					printf("++++++++++++++++++++++++++++data= %s\n",data);
					curl_easy_setopt(curl,CURLOPT_TIMEOUT,3);
					curl_easy_setopt(curl,CURLOPT_URL,url);
					curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST,"POST");
					curl_easy_setopt(curl, CURLOPT_POST,1L);
					curl_easy_setopt(curl, CURLOPT_HEADER,1L);
					curl_easy_setopt(curl, CURLOPT_HTTPHEADER,headers);
					curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
					curl_easy_setopt(curl, CURLOPT_POSTFIELDS,data);
					
					curl_easy_setopt(curl, CURLOPT_COOKIEFILE, "/usr/curlposttest.cookie");
					re=curl_easy_perform(curl);
					printf("%d\n",re);
					curl_slist_free_all(headers);
					//curl_formfree(first);
					curl_easy_cleanup(curl);
						}
				
					if(a->last==1)
						break;
					else
						a=a->next;
				}

		}
}






/********************************************************************************************
以下是查询wifi物联网设备状态所用函数


*********************************************************************************************/
static size_t libcurl_write_callback(void *ptr, size_t size, size_t nmemb, void *stream)
{
	char *getstr;
	//printf("-----------------libcurl write callback\n getstr= %s\n",getstr);
	getstr=stream;
	strcat(getstr,ptr);
	printf("-----------------libcurl write callback\n getstr= %s\n",getstr);
}

static void libcurl_select_switch(struct wifi_switch *sw)
{

	/*int global_libcurl_init_return=0;
	char libcurl_url[256];
	char *libcurl_select_header="http://";
	char *libcurl_select_end="/config?command=Device";
	char libcurl_select_recive[512];*/


	
	struct wifi_switch *a;
	CURL *curl;
    CURLcode res;
	int i,j;
	
	a=sw;
	memset(libcurl_url,0,256);
	strcpy(libcurl_url,libcurl_select_header);
	//pthread_mutex_lock(&internet_mutex);
	strcat(libcurl_url,a->ip);
	//pthread_mutex_unlock(&internet_mutex);
	//strcat(libcurl_url,a->ip);
	strcat(libcurl_url,libcurl_select_end);
	printf("---------------------use libcurl to send buf =%s\n",libcurl_url);
	printf("------------------------now use libcurl to select \n");
    curl = curl_easy_init();
	memset(libcurl_select_recive,0,512);
	if(a->status>0)
	a->status=a->status-1;
	
	if(curl)
		{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_URL,libcurl_url))
				{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_TIMEOUT,3))
				{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION,libcurl_write_callback))
				{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_WRITEDATA,libcurl_select_recive))
				{
			res = curl_easy_perform(curl);
			if(strlen(libcurl_select_recive)>0)
				{
					printf("--------------------use libcurl select success\n");
					printf("------------------------libcurl recive =%s\n",libcurl_select_recive);
					cJSON *data_cjson;
					cJSON *la_cjson;
					//cJSON *lamp1_cjson;
					//cJSON *lamp2_cjson;
					char ep[4]; 
					int num;
					char *la1;
					char *la2;
					ep[0]='e';
					ep[1]='p';
					ep[3]='\0';
					data_cjson=cJSON_Parse(libcurl_select_recive);
					printf("libcurl_select_recive = %s\n",libcurl_select_recive);
					la_cjson=cJSON_GetObjectItem(data_cjson,"switch");
					
					//pthread_mutex_lock(&internet_mutex);
					num=cJSON_GetObjectItem(la_cjson,"num")->valueint;
					//num=strtol(cJSON_GetObjectItem(la_cjson,"num")->valuestring,NULL,10);
					printf("num = %d\n",num);
					if(num>10)
						{
							num=10;
						}
					//a->lamp[0]=cJSON_GetObjectItem(la_cjson,"ep0")->valueint;
					for(j=0;j<num;j++)
						{
							ep[2]=j+49;
							printf("----------------ep = %s",ep);
							a->lamp[j]=cJSON_GetObjectItem(la_cjson,ep)->valueint;
							//a->lamp[j]=strtol(cJSON_GetObjectItem(la_cjson,ep)->valuestring,NULL,10);
						}
					//la1=cJSON_GetObjectItem(la_cjson,"ep1")->valuestring;
					//la2=cJSON_GetObjectItem(la_cjson,"ep2")->valuestring;
					a->status=2;
					a->lamp_number=num;
					
					//pthread_mutex_unlock(&internet_mutex);
					//a->lamp[0]=1;
					//a->lamp[1]=1;
					cJSON_Delete(data_cjson);
				}
			else
				printf("------------------use libcurl select false\n");
			
				}
				}
				}
				}
			curl_easy_cleanup(curl);
		}
	if(a->status==0)
		{
			for(i=0;i<10;i++)
				{
					a->lamp[i]=0;
				}
			a->mac_struct->status=0;
		}

	
}

static void libcurl_select_sensor(struct wifi_sensor *sw)
{
	struct wifi_sensor *a;
	CURL *curl;
    CURLcode res;
	int i;
	
	a=sw;
	memset(libcurl_url,0,256);
	strcpy(libcurl_url,libcurl_select_header);
	//pthread_mutex_lock(&internet_mutex);
	strcat(libcurl_url,a->ip);
	//pthread_mutex_unlock(&internet_mutex);
	//strcat(libcurl_url,a->ip);
	strcat(libcurl_url,libcurl_select_end);

	
    curl= curl_easy_init();
	memset(libcurl_select_recive,0,512);
	if(a->status>0)
	a->status=a->status-1;
	
	if(curl)
		{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_URL,libcurl_url))
				{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_TIMEOUT,3))
				{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION,libcurl_write_callback))
				{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_WRITEDATA,libcurl_select_recive))
				{
			res = curl_easy_perform(curl);
			if(strlen(libcurl_select_recive)>0)
				{
					cJSON *data_cjson;
					cJSON *la_cjson;
					//cJSON *lamp1_cjson;
					//cJSON *lamp2_cjson;
					/*char ep[4]; 
					int num;
					char *la1;
					char *la2;
					ep[0]='e';
					ep[1]='p';
					ep[3]='\0';*/
					char cc;
					data_cjson=cJSON_Parse(libcurl_select_recive);
					la_cjson=cJSON_GetObjectItem(data_cjson,"sensor");
					
					//pthread_mutex_lock(&internet_mutex);
					a->temperature=cJSON_GetObjectItem(la_cjson,"ep1")->valueint;
					//printf("%s\n",cJSON_GetObjectItem(la_cjson,"ep1")->valuestring);
					//printf("cJSON_GetObjectItem->valueint=%d\n",cJSON_GetObjectItem(la_cjson,"ep1")->valueint);
					//printf("a->temperature= %d\n",a->temperature);
					a->humidity=cJSON_GetObjectItem(la_cjson,"ep2")->valueint;
					a->illumination=cJSON_GetObjectItem(la_cjson,"ep3")->valueint;
					a->status=2;
					
					run_info.temperature=a->temperature;
					//printf("Temperature = %d\n",Temperature);
					run_info.humidity=a->humidity;
					run_info.illumination=a->illumination;
					//pthread_mutex_unlock(&internet_mutex);



					
					/*num=strtol(cJSON_GetObjectItem(la_cjson,"num")->valuestring,NULL,10);
					if(num>10)
						{
							num=10;
						}
					for(j=0;j<num;j++)
						{
							ep[2]=j+48;
							a->lamp[j]=strtol(cJSON_GetObjectItem(la_cjson,ep)->valuestring,NULL,10);
						}
					//la1=cJSON_GetObjectItem(la_cjson,"ep1")->valuestring;
					//la2=cJSON_GetObjectItem(la_cjson,"ep2")->valuestring;
					a->status=2;
					a->lamp_number=num;
					//a->lamp[0]=1;
					//a->lamp[1]=1;*/
					cJSON_Delete(data_cjson);
				}
				}
				}
				}
				}
			curl_easy_cleanup(curl);
			
		}
	if(a->status==0)
		{
			a->mac_struct->status=0;
		}
}

static void libcurl_select_screen(struct wifi_screen *sw)
{
	struct wifi_screen *a;
	CURL *curl;
    CURLcode res;
	int i;
	
	a=sw;
	memset(libcurl_url,0,256);
	strcpy(libcurl_url,libcurl_select_header);
	//pthread_mutex_lock(&internet_mutex);
	strcat(libcurl_url,a->ip);
	//pthread_mutex_unlock(&internet_mutex);
	//strcat(libcurl_url,a->ip);
	strcat(libcurl_url,libcurl_select_end);

	
    curl = curl_easy_init();
	memset(libcurl_select_recive,0,512);
	if(a->status>0)
	a->status=a->status-1;
	
	if(curl)
		{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_URL,libcurl_url))
				{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_TIMEOUT,3))
				{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION,libcurl_write_callback))
				{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_WRITEDATA,libcurl_select_recive))
				{
			res = curl_easy_perform(curl);
			if(strlen(libcurl_select_recive)>0)
				{
				
					//pthread_mutex_lock(&internet_mutex);
					a->status=2;
					
					//pthread_mutex_unlock(&internet_mutex);
				}
				}
				}
				}
				}
			curl_easy_cleanup(curl);
		}
	if(a->status==0)
		{
			a->mac_struct->status=0;
		}
}

static void libcurl_select_rfid(struct wifi_rfid *sw)
{
	struct wifi_screen *a;
	CURL *curl;
    CURLcode res;
	int i;
	
	a=sw;
	memset(libcurl_url,0,256);
	strcpy(libcurl_url,libcurl_select_header);
	
	//pthread_mutex_lock(&internet_mutex);
	strcat(libcurl_url,a->ip);
	//pthread_mutex_unlock(&internet_mutex);
	
	strcat(libcurl_url,libcurl_select_end);

	
    curl = curl_easy_init();
	memset(libcurl_select_recive,0,512);
	
	if(a->status>0)
	a->status=a->status-1;
	
	if(curl)
		{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_URL,libcurl_url))
				{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_TIMEOUT,3))
				{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION,libcurl_write_callback))
				{
			if(CURLE_OK==curl_easy_setopt(curl,CURLOPT_WRITEDATA,libcurl_select_recive))
				{
			res = curl_easy_perform(curl);
			if(strlen(libcurl_select_recive)>0)
				{
					
					//pthread_mutex_lock(&internet_mutex);
					a->status=2;
					
					//pthread_mutex_unlock(&internet_mutex);
				}
				}
				}
				}
				}

			curl_easy_cleanup(curl);
		}
	if(a->status==0)
		{
			
			a->mac_struct->status=0;
			
		}
}


static void select_wifi_internetthings_fuction(void)
{


	struct wifi_switch *switch_a;
	struct wifi_sensor *sensor_a;
	struct wifi_screen *screen_a;
	struct wifi_rfid *rfid_a;

	//switch

	if(wifi_switchheader.number==0)
		{
			printf("there is no switch\n");
		}
	else
		{
			printf("there is switch\n");
			switch_a=wifi_switchheader.next;
			while(1)
				{
					if(switch_a->del==0)
					libcurl_select_switch(switch_a);
					if(switch_a->last==1)
						break;
					else
						switch_a=switch_a->next;
				}
		}

	//sensor

	if(wifi_sensorheader.number==0)
		{
			printf("there is no sensor\n");
		}
	else
		{
			sensor_a=wifi_sensorheader.next;
			while(1)
				{
					if(sensor_a->del==0)
					libcurl_select_sensor(sensor_a);
					if(sensor_a->last==1)
						break;
					else
						sensor_a=sensor_a->next;
				}
		}


	//screen

	if(wifi_screenheader.number==0)
		{
			printf("there is no screen\n");
		}
	else
		{
			screen_a=wifi_screenheader.next;
			while(1)
				{
					if(screen_a->del==0)
					libcurl_select_screen(screen_a);
					if(screen_a->last==1)
						break;
					else
						screen_a=screen_a->next;
				}
		}

	//rfid

	if(wifi_rfidheader.number==0)
		{
			printf("there is no rfid\n");
		}
	else
		{
			rfid_a=wifi_rfidheader.next;
			while(1)
				{
					if(rfid_a->del==0)
						{
					libcurl_select_rfid(rfid_a);
						}
					if(rfid_a->last==1)
						break;
					else
						rfid_a=rfid_a->next;
					
				}
		}

}

/********************************************************************************
              查询线程



********************************************************************************/

static void *select_wifi_internetthings_status(void)
{
	while(1)
		{
			select_wifi_internetthings_fuction();
			sleep(5);
		}
}



static void libcurl_global_init(void)
{
	//int re;
	if(CURLE_OK!=curl_global_init(CURL_GLOBAL_ALL))
		{
			printf("curl global init false\n");
			exit(-1);
		}
	//printf("libcurl goibal init return = %d\n",re);
}




/************************************************************************************
广播接受数据后处理函数，建立物联网设备结构体并加入链表在此函数


传递的type确定类型 1为wifi_switchheader，2为wifi_sensorheader，3为wifi_screenheader
4为wifi_rfidheader，5为wifi_macheader


************************************************************************************/


static void deal_with_recive_from_broadcast(char * rebuf,char *oldmac1,struct wifi_internetthings_mac *mac_struct1)
{
	char buf[256];
	char *bu;
	char type[36];
	char remac[24];
	char reip[24];
	char *sepa;
	int i,j;
	struct wifi_internetthings_mac *mac_struct;
	struct wifi_rfid *rfida;
	char *oldmac;
	char *blank=" ";
	memset(buf,0,256);
	memset(type,0,36);
	memset(remac,0,24);
	memset(reip,0,24);
	strcpy(buf,rebuf);
	j=0;
	bu=buf;
	for(i=0;i<3;i++)
		{
			sepa=strsep(&bu,blank);
			if(j==0)
				{
					strcpy(type,sepa);
					printf("%s\n",type);
					j++;
				}
			else if(j==1)
				{
					strcpy(remac,sepa);
					printf("%s\n",remac);
					j++;
				}
			else if(j==2)
				{
					strcpy(reip,sepa);
					printf("%s\n",reip);
					j++;
				}
		}
	if(strcmp(type,"rfid-user")==0)
		{
			if(wifi_rfidheader.number>0)
				{
					rfida=wifi_rfidheader.next;
					while(1)
						{
					
							if(strcmp(rfida->mac,remac)==0)
								{
									wifi_rfid_data_return(rfida->ip);
									//wifi_rfid_data_turn_on(reip);
									
									break;
								}
							if(rfida->last==1)
								break;
							else
								rfida=rfida->next;
						}
				}
		}
	else
		{
			//pthread_mutex_lock(&internet_mutex);
		if(wifi_macheader.number>0)
			{
			mac_struct=wifi_macheader.next;
			while(1)
				{
	if(strcmp(remac,mac_struct->mac)==0)
		{
			if(strcmp(type,"switch")==0)
				{
					if(mac_struct->aready_add==1)
						{
							struct wifi_switch *b;
							b=(struct wifi_switch *)mac_struct->thing;
							strcpy(b->ip,reip);
							mac_struct->status=1;
						}
					else
						{
					struct wifi_switch *a;
					//a=(struct wifi_switch *)malloc(sizeof(struct wifi_switch));
					while((a=(struct wifi_switch *)malloc(sizeof(struct wifi_switch)))==NULL)
						{
							printf("malloc false\n");
							sleep(2);
						}
					memset(a,0,sizeof(struct wifi_switch));
					a->status=0;
					a->del=0;
					strcpy(a->ip,reip);
					strcpy(a->mac,remac);
					a->name_number=wifi_switchheader.number+1;
					//sprintf(a->name,"switch-%d",a->name_number);
					strcpy(a->name,mac_struct->name);
					a->last=1;
					a->mac_struct=mac_struct;
					a->mac_struct->status=1;
					
					mac_struct->type=1;
					mac_struct->aready_add=1;
					
					mac_struct->thing=(void *)a;
					
					add_to_header((void *)a,1);


					
						}
				}
			else if(strcmp(type,"sensor")==0)
				{
					if(mac_struct->aready_add==1)
						{
							struct wifi_sensor *b;
							b=(struct wifi_sensor *)mac_struct->thing;
							strcpy(b->ip,reip);
							mac_struct->status=1;
						}
					else
						{
					struct wifi_sensor *a;
					//a=(struct wifi_sensor *)malloc(sizeof(struct wifi_sensor));
					while((a=(struct wifi_sensor *)malloc(sizeof(struct wifi_sensor)))==NULL)
						{
							printf("malloc false\n");
							sleep(2);
						}
					memset(a,0,sizeof(struct wifi_sensor));
					a->status=0;
					a->del=0;
					strcpy(a->ip,reip);
					strcpy(a->mac,remac);
					a->name_number=wifi_sensorheader.number+1;
					//sprintf(a->name,"sensor-%d",a->name_number);
					strcpy(a->name,mac_struct->name);
					a->last=1;
					a->mac_struct=mac_struct;
					a->mac_struct->status=1;

					mac_struct->type=2;
					mac_struct->aready_add=1;
					mac_struct->thing=(void *)a;

					add_to_header((void *)a,2);
						}
				}
			else if(strcmp(type,"screen")==0)
				{
					if(mac_struct->aready_add==1)
						{
							struct wifi_screen *b;
							b=(struct wifi_screen *)mac_struct->thing;
							strcpy(b->ip,reip);
							mac_struct->status=1;
						}
					else
						{
					struct wifi_screen *a;
					//a=(struct wifi_screen *)malloc(sizeof(struct wifi_screen));
					while((a=(struct wifi_screen *)malloc(sizeof(struct wifi_screen)))==NULL)
						{
							printf("malloc false\n");
							sleep(2);
						}
					memset(a,0,sizeof(struct wifi_screen));
					a->status=0;
					a->del=0;
					strcpy(a->ip,reip);
					strcpy(a->mac,remac);
					a->name_number=wifi_screenheader.number+1;
					//sprintf(a->name,"sensor-%d",a->name_number);
					strcpy(a->name,mac_struct->name);
					a->last=1;
					a->mac_struct=mac_struct;
					a->mac_struct->status=1;
					
					mac_struct->type=3;
					mac_struct->aready_add=1;
					mac_struct->thing=(void *)a;

					add_to_header((void *)a,3);
						}
				}
			else if(strcmp(type,"rfid")==0)
				{
					if(mac_struct->aready_add==1)
						{
							struct wifi_rfid *b;
							b=(struct wifi_rfid *)mac_struct->thing;
							strcpy(b->ip,reip);
							mac_struct->status=1;
						}
					else
						{
					struct wifi_rfid *a;
					//a=(struct wifi_rfid *)malloc(sizeof(struct wifi_rfid));
					while((a=(struct wifi_rfid *)malloc(sizeof(struct wifi_rfid)))==NULL)
						{
							printf("malloc false\n");
							sleep(2);
						}
					memset(a,0,sizeof(struct wifi_rfid));
					a->status=0;
					a->del=0;
					strcpy(a->ip,reip);
					strcpy(a->mac,remac);
					a->name_number=wifi_rfidheader.number+1;
					//sprintf(a->name,"sensor-%d",a->name_number);
					strcpy(a->name,mac_struct->name);
					a->last=1;
					a->mac_struct=mac_struct;
					a->mac_struct->status=1;

					mac_struct->type=4;
					mac_struct->aready_add=1;
					mac_struct->thing=(void *)a;

					add_to_header((void *)a,4);
						}
				}
			break;
			
		}
				
	if(mac_struct->last==1)
		{
			break;
		}
	else
		{
			mac_struct=mac_struct->next;
		}
				}
			}
		}
	
	//pthread_mutex_unlock(&internet_mutex);
}

/*static void deal_with_recive_from_broadcast(char * rebuf,char *oldmac,struct wifi_internetthings_mac *mac_struct)
{
	char buf[256];
	char *bu;
	char type[36];
	char remac[24];
	char reip[24];
	char *sepa;
	int i,j;
	char *blank=" ";
	memset(buf,0,256);
	memset(type,0,36);
	memset(remac,0,24);
	memset(reip,0,24);
	strcpy(buf,rebuf);
	j=0;
	bu=buf;
	for(i=0;i<3;i++)
		{
			sepa=strsep(&bu,blank);
			if(j==0)
				{
					strcpy(type,sepa);
					printf("%s\n",type);
					j++;
				}
			else if(j==1)
				{
					strcpy(remac,sepa);
					printf("%s\n",remac);
					j++;
				}
			else if(j==2)
				{
					strcpy(reip,sepa);
					printf("%s\n",reip);
					j++;
				}
		}
	
	//pthread_mutex_lock(&internet_mutex);
	if(strcmp(remac,oldmac)==0)
		{
			if(strcmp(type,"switch")==0)
				{
					if(mac_struct->aready_add==1)
						{
							struct wifi_switch *b;
							b=(struct wifi_switch *)mac_struct->thing;
							strcpy(b->ip,reip);
						}
					else
						{
					struct wifi_switch *a;
					a=(struct wifi_switch *)malloc(sizeof(struct wifi_switch));
					memset(a,0,sizeof(struct wifi_switch));
					a->status=0;
					a->del=0;
					strcpy(a->ip,reip);
					strcpy(a->mac,remac);
					a->name_number=wifi_switchheader.number+1;
					//sprintf(a->name,"switch-%d",a->name_number);
					strcpy(a->name,mac_struct->name);
					a->last=1;
					a->mac_struct=mac_struct;
					a->mac_struct->status=1;
					
					mac_struct->type=1;
					mac_struct->aready_add=1;
					mac_struct->thing=(void *)a;
					
					add_to_header((void *)a,1);


					
						}
				}
			else if(strcmp(type,"sensor")==0)
				{
					if(mac_struct->aready_add==1)
						{
							struct wifi_sensor *b;
							b=(struct wifi_sensor *)mac_struct->thing;
							strcpy(b->ip,reip);
						}
					else
						{
					struct wifi_sensor *a;
					a=(struct wifi_sensor *)malloc(sizeof(struct wifi_sensor));
					memset(a,0,sizeof(struct wifi_sensor));
					a->status=0;
					a->del=0;
					strcpy(a->ip,reip);
					strcpy(a->mac,remac);
					a->name_number=wifi_sensorheader.number+1;
					//sprintf(a->name,"sensor-%d",a->name_number);
					strcpy(a->name,mac_struct->name);
					a->last=1;
					a->mac_struct=mac_struct;
					a->mac_struct->status=1;

					mac_struct->type=2;
					mac_struct->aready_add=1;
					mac_struct->thing=(void *)a;

					add_to_header((void *)a,2);
						}
				}
			else if(strcmp(type,"screen")==0)
				{
					if(mac_struct->aready_add==1)
						{
							struct wifi_screen *b;
							b=(struct wifi_screen *)mac_struct->thing;
							strcpy(b->ip,reip);
						}
					else
						{
					struct wifi_screen *a;
					a=(struct wifi_screen *)malloc(sizeof(struct wifi_screen));
					memset(a,0,sizeof(struct wifi_screen));
					a->status=0;
					a->del=0;
					strcpy(a->ip,reip);
					strcpy(a->mac,remac);
					a->name_number=wifi_screenheader.number+1;
					//sprintf(a->name,"sensor-%d",a->name_number);
					strcpy(a->name,mac_struct->name);
					a->last=1;
					a->mac_struct=mac_struct;
					a->mac_struct->status=1;
					
					mac_struct->type=3;
					mac_struct->aready_add=1;
					mac_struct->thing=(void *)a;

					add_to_header((void *)a,3);
						}
				}
			else if(strcmp(type,"rfid")==0)
				{
					if(mac_struct->aready_add==1)
						{
							struct wifi_rfid *b;
							b=(struct wifi_rfid *)mac_struct->thing;
							strcpy(b->ip,reip);
						}
					else
						{
					struct wifi_rfid *a;
					a=(struct wifi_rfid *)malloc(sizeof(struct wifi_rfid));
					memset(a,0,sizeof(struct wifi_rfid));
					a->status=0;
					a->del=0;
					strcpy(a->ip,reip);
					strcpy(a->mac,remac);
					a->name_number=wifi_rfidheader.number+1;
					//sprintf(a->name,"sensor-%d",a->name_number);
					strcpy(a->name,mac_struct->name);
					a->last=1;
					a->mac_struct=mac_struct;
					a->mac_struct->status=1;

					mac_struct->type=4;
					mac_struct->aready_add=1;
					mac_struct->thing=(void *)a;

					add_to_header((void *)a,4);
						}
				}
			
		}
	
	//pthread_mutex_unlock(&internet_mutex);
}*/

/**********************************************************************************
获取lan口广播ip函数，在加网线程中调用


**********************************************************************************/

static int get_lan_ip_to_broadcast(char *ip)
{
	struct uci_package *pkg=NULL;
	struct uci_element *element;
	struct uci_context *context=NULL;
	struct uci_section *section;
	struct uci_option *option;
	char *separator_previous;
	char *ser;
	//char *sep;
	char *ipend="255";
	char *point=".";
	char ipstore[24];
	char ip1[24];
	char ip2[24];
	char ip3[24];
	char ipnew[24];
	char *value;
	int i;
	int count=0;
	memset(ipstore,0,24);
	memset(ip1,0,24);
	memset(ip2,0,24);
	memset(ip3,0,24);
	memset(ipnew,0,24);
	printf("get broadcast ip\n");
	context=uci_alloc_context();
	if(UCI_OK!=uci_load(context,"/etc/config/network",&pkg))
		{
			printf("cannot load uci from network to get lan_ip");
			//uci_free_context(context);
			//context=NULL;
			goto cleanup;
		}
	
	if(NULL==(section=uci_lookup_section(context,pkg,"lan")))
		{
			printf("cannot get section from lan\n");
			goto err;
		}
	//option=uci_lookup_option(ctx, section, "mac")
	if(NULL==(value = uci_lookup_option_string(context, section, "ipaddr")))
		{
			printf("cannot get value from ipaddr\n");
			goto err;
		}
	strcpy(ipstore,value);
	printf("%s\n",ipstore);
	//sep=strdup(ipstore);
	ser=ipstore;
	while(1)
		{
			
			separator_previous=strsep(&ser,".");
			//ser=separator_previous;
			if(separator_previous==NULL)
				break;
			if(count==0)
				{
					strcpy(ip1,separator_previous);
					printf("%s\n",ip1);
					count++;
				}
			else if(count==1)
				{
					strcpy(ip2,separator_previous);
					printf("%s\n",ip2);
					count++;
				}
			else if(count==2)
				{
					strcpy(ip3,separator_previous);
					printf("%s\n",ip3);
					count++;
				}
			else
				break;
		}
		//for(i=0;i<3;i++)
		//	{
				//separator_previous=strstr(&sep,".");
			/*if(count==0)
				{
					strcpy(ip1,separator_previous);
					count++;
				}
			else if(count==1)
				{
					strcpy(ip2,separator_previous);
					count++;
				}
			else if(count==2)
				{
					strcpy(ip3,separator_previous);
					count++;
				}*/
		//	}
	//free(sep);
	strcpy(ip,ip1);
	strcat(ip,point);
	strcat(ip,ip2);
	strcat(ip,point);
	strcat(ip,ip3);
	strcat(ip,point);
	strcat(ip,ipend);

	uci_unload(context,pkg);
	uci_free_context(context);
	context=NULL;
	return 1;
err:
	uci_unload(context,pkg);
	uci_free_context(context);
	context=NULL;
	return -1;
cleanup:
	uci_free_context(context);
	context=NULL;
	return -1;
}

/*******************************************************************************
加网广播函数，只广播收发数据，调用deal_with_recive_from_broadcast函数进行接受的
数据处理


********************************************************************************/

static int broadcast_to_add(char *mac,char *lanip,struct wifi_internetthings_mac *mac_struct)
{
	char bufto[24];
	char *rebuf;
	//int len;
	int re;
	struct timeval tv;
	tv.tv_sec=3;
	tv.tv_usec=0;
	memset(bufto,0,24);
	//memset(bufre,0,256);
	strcpy(bufto,mac);
	char *tobuf=bufto;
	printf("%s\n",tobuf);
	
	
	memset(&wifi_serve,0,sizeof(struct sockaddr_in));
	

	wifi_serve.sin_family=AF_INET;
	wifi_serve.sin_port=htons(1025);
	wifi_serve.sin_addr.s_addr=inet_addr(lanip);

	
	int so_broadcast=1;
	int k=strlen(tobuf);
	printf("tobuf len = %d\n",k);
	printf("broadcast ip = %s\n",lanip);
	//len=sizeof(wifi_client);
	setsockopt(wifi_sock_fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast,sizeof(so_broadcast));
	//setsockopt(wifi_sock_fd,SOL_SOCKET,SO_RCVTIMEO,&tv, sizeof(tv));
	printf("now send MAC\n");
	sendto(wifi_sock_fd,tobuf,k,0,(struct sockaddr *)&wifi_serve,sizeof(wifi_serve));
	printf("send MAC success = %s\n",tobuf);
	broadcast_select_turn=1;
}




/*static int broadcast_to_add(char *mac,char *lanip,struct wifi_internetthings_mac *mac_struct)
{
	char bufto[24],bufre[256];
	char *rebuf;
	int len;
	int re;
	struct timeval tv;
	tv.tv_sec=3;
	tv.tv_usec=0;
	memset(bufto,0,24);
	memset(bufre,0,256);
	strcpy(bufto,mac);
	char *tobuf=bufto;
	printf("%s\n",tobuf);
	struct sockaddr_in client,serve;
	int sock_fd;
	sock_fd=socket(AF_INET,SOCK_DGRAM,0);
	if(sock_fd<0)
		{
			perror("socket");
			exit(-1);
			return -1;
		}
	memset(&serve,0,sizeof(struct sockaddr_in));
	memset(&client,0,sizeof(struct sockaddr_in));

	serve.sin_family=AF_INET;
	serve.sin_port=htons(1025);
	serve.sin_addr.s_addr=inet_addr(lanip);

	client.sin_family=AF_INET;
	client.sin_port=htons(1025);
	client.sin_addr.s_addr=htonl(INADDR_ANY);
	int so_broadcast=1;
	int k=strlen(tobuf);
	printf("tobuf len = %d\n",k);
	printf("broadcast ip = %s\n",lanip);
	len=sizeof(client);
	setsockopt(sock_fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast,sizeof(so_broadcast));
	setsockopt(sock_fd,SOL_SOCKET,SO_RCVTIMEO,&tv, sizeof(tv));
	printf("now send MAC\n");
	sendto(sock_fd,tobuf,k,0,(struct sockaddr *)&serve,sizeof(serve));
	printf("send MAC success\n");
	re=recvfrom(sock_fd,(void*)bufre,256,0,(struct sockaddr *)&client,(socklen_t *)&len);
	if(re==-1)
		{
			printf("recive over time\n");
			return -1;
		}
	else
		{
			printf("get internetthings success\n");
			printf("bufre = %s\n",bufre);
			deal_with_recive_from_broadcast(bufre,mac,mac_struct);
		}
	//rebuf=bufre;
	close(sock_fd);
	printf("recive internetthings string is =%s\n",rebuf);
}*/




/************************************************************************
依次从wifi_macheader中取出mac结构体，进行加网


************加网线程*************


*************************************************************************/
static void *add_internetthings_from_broadcast(void)
{
	struct wifi_internetthings_mac *submac;
	char lanip[24];
	printf("start\n");
	while(1)
		{
			memset(lanip,0,24);
			if(get_lan_ip_to_broadcast(lanip)<0)
				{
					printf("get lan ip false\n");
				}
			else
				{
			//pthread_mutex_lock(&mac_mutex);
					submac=wifi_macheader.next;
					if(wifi_macheader.number==0)
						{
							printf("there is no internetthings\n");
							
						}
					else
						{
							printf("now enter broadcast while\n");
							while(1)
								{
							
									if((submac->status==0)&&(submac->del==0))
										{
											broadcast_to_add(submac->mac,lanip,submac);
										}
									else
										{
											printf("all of the things is add aready\n");
										}
									if(submac->last==1)
										{
											break;
										}
									else
										{
											submac=submac->next;
										}
									printf("////////\n");
							
								}
						}
				}
			//pthread_mutex_unlock(&mac_mutex);
			sleep(5);
		}
}




//传递的type确定类型 1为wifi_switchheader，2为wifi_sensorheader，3为wifi_screenheader
//4为wifi_rfidheader，5为wifi_macheader
/************************************************************************************
将结构体加入链表头中函数  data要加的结构体，type类型。
************************************************************************************/
static void add_to_header(void *data,int type)
{
	switch(type)
		{
			//wifi_switchheader
			case 1:
				printf("get switch struct\n");
				struct wifi_switch *switcha;
				struct wifi_switch *switchb;
				switcha=(struct wifi_switch *)data;
				if(wifi_switchheader.number==0)
					{
						wifi_switchheader.next=switcha;
						wifi_switchheader.number++;
					}
				else
					{
						switchb=wifi_switchheader.next;
						while(1)
							{
								if(switchb->last==1)
									{
										switchb->next=switcha;
										wifi_switchheader.number++;
										switchb->last=0;
										break;
										
									}
								else
									{
										switchb=switchb->next;
									}
							}
					}
				break;
			//wifi_sensorheader
			case 2:
				printf("get sensor struct\n");
				struct wifi_sensor *sensora;
				struct wifi_sensor *sensorb;
				sensora=(struct wifi_sensor *)data;
				if(wifi_sensorheader.number==0)
					{
						wifi_sensorheader.next=sensora;
						wifi_sensorheader.number++;
					}
				else
					{
						sensorb=wifi_sensorheader.next;
						while(1)
							{
								if(sensorb->last==1)
									{
										sensorb->next=sensora;
										sensorb->last=0;
										wifi_sensorheader.number++;
										break;
										
									}
								else
									{
										sensorb=sensorb->next;
									}
							}
					}
				break;
			//wifi_screenheader
			case 3:
				printf("get screen struct\n");
				struct wifi_screen *screena;
				struct wifi_screen *screenb;
				screena=(struct wifi_screen *)data;
				if(wifi_screenheader.number==0)
					{
						wifi_screenheader.next=screena;
						wifi_screenheader.number++;
					}
				else
					{
						screenb=wifi_screenheader.next;
						while(1)
							{
								if(screenb->last==1)
									{
										screenb->next=screena;
										screenb->last=0;
										wifi_screenheader.number++;
										break;
										
									}
								else
									{
										screenb=screenb->next;
									}
							}
					}
				break;
			//wifi_rfidheader
			case 4:
				printf("get rfid struct\n");
				struct wifi_rfid *rfida;
				struct wifi_rfid *rfidb;
				rfida=(struct wifi_rfid *)data;
				if(wifi_rfidheader.number==0)
					{
						wifi_rfidheader.next=rfida;
						wifi_rfidheader.number++;
					}
				else
					{
						rfidb=wifi_rfidheader.next;
						while(1)
							{
								if(rfidb->last==1)
									{
										rfidb->next=rfida;
										rfidb->last=0;
										wifi_rfidheader.number++;
										break;
										
									}
								else
									{
										rfidb=rfidb->next;
									}
							}
					}
				break;	
			//wifi_macheader
			case 5:
				printf("get mac struct\n");
				struct wifi_internetthings_mac *maca;
				struct wifi_internetthings_mac *macb;
				maca=(struct wifi_internetthings_mac *)data;
				if(wifi_macheader.number==0)
					{
						wifi_macheader.next=maca;
						wifi_macheader.number++;
					}
				else
					{
						macb=wifi_macheader.next;
						while(1)
							{
								if(macb->last==1)
									{
										macb->next=maca;
										macb->last=0;
										wifi_macheader.number++;
										break;
										
									}
								else
									{
										macb=macb->next;
									}
							}
					}
				break;
			default:
				break;
		}
}


static void init_wifi_internettings_header(void)
{
	wifi_switchheader.number=0;
	wifi_sensorheader.number=0;
	wifi_screenheader.number=0;
	wifi_rfidheader.number=0;
	wifi_macheader.number=0;
}
/*****************************************************************************
从uci中获取mac并加入链表中

*****************************************************************************/
static int get_internetthings_mac_from_uci(void)
{
	struct uci_package *pkg = NULL;
    struct uci_element *ee;
	struct uci_section *section;
	struct uci_option *o;
	char va[128];
	char mac[24];
	char name[104];
	char *na;
	char *value;
	int i;
	int j;
	struct uci_context * ctx = NULL;
    ctx = uci_alloc_context(); // 申请一个UCI上下文.
	//uci_set_savedir(ctx, "/tmp/");
	//uci_set_confdir(ctx, "/usr/");
	//printf("set ctx dir later\n");
    if (UCI_OK != uci_load(ctx, "/usr/share/brxydata/brxy", &pkg))
	{
		printf("uci load false\n");
        goto cleanup; //如果打开UCI文件失败,则跳到末尾 清理 UCI 上下文
       // exit(-1);
	}
	if(NULL==(section=uci_lookup_section(ctx,pkg,"Iot")))
		{
			printf("get Iot false\n");
			goto err;
		}
	
	
	/*if(NULL==(o= uci_lookup_option(ctx, section, "mac")))
		{
			printf("get mac false\n");
			goto err;
		}*/
	o= uci_lookup_option(ctx, section, "mac");
	if ((NULL != o) && (UCI_TYPE_LIST == o->type)) //o存在 且 类型是 UCI_TYPE_LIST则可以继续.
	{
		struct uci_element *e;
		uci_foreach_element(&o->v.list, e)
		{


			memset(va,0,128);
			memset(name,0,104);
			memset(mac,0,24);
			value=e->name;
			printf("///////////////////////%s\n",value);
			strcpy(va,value);
			for(j=0;j<17;j++)
				{
					mac[j]=va[j];
				}
			na=&va[17];
			strcpy(name,na);

			
			printf("mac = %s\nname = %s\n",mac,name);
			struct wifi_internetthings_mac *a;
			//a=(struct wifi_internetthings_mac *)malloc(sizeof(struct wifi_internetthings_mac));
			while((a=(struct wifi_internetthings_mac *)malloc(sizeof(struct wifi_internetthings_mac)))==NULL)
				{
					printf("malloc false\n");
					sleep(2);
				}
			memset(a->mac,0,24);
			memset(a->name,0,104);
			a->del=0;
			a->last=1;
			a->status=0;//0表示不在线，1表示在线，具体要加网置1，查询置0
			strcpy(a->mac,mac);
			strcpy(a->name,name);
			a->aready_add=0;
			add_to_header((void *)a,5);

			
		}
	}


    //遍历UCI的每一个节
  /*  uci_foreach_element(&pkg->sections, e)
    {
        struct uci_section *s = uci_to_section(e);
        // 将一个 element 转换为 section类型, 如果节点有名字,则 s->anonymous 为 false.
        // 此时通过 s->e->name 来获取.
        // 此时 您可以通过 uci_lookup_option()来获取 当前节下的一个值.
        if (NULL != (value = uci_lookup_option_string(ctx, s, "ipaddr")))
        {
            ip = strdup(value) //如果您想持有该变量值，一定要拷贝一份。当 pkg销毁后value的内存会被释放。
        }
        // 如果您不确定是 string类型 可以先使用 uci_lookup_option() 函数得到Option 然后再判断.
        // Option 的类型有 UCI_TYPE_STRING 和 UCI_TYPE_LIST 两种.


    }*/
    uci_unload(ctx, pkg); // 释放 pkg 
    uci_free_context(ctx);
	return 1;
err:
	uci_unload(ctx, pkg); // 释放 pkg 
    uci_free_context(ctx);
	return -1;
cleanup:
    uci_free_context(ctx);
    ctx = NULL;
	return -1;
}


/*******************************************************************************

以上是wifi物联网函数
********************************************************************************/



/*******************************************************************************
以下是qq上报数据和接收数据处理函数


*********************************************************************************/

/**
 * 登录完成的通知，errcode为0表示登录成功，其余请参考全局的错误码表
 */
void on_login_complete(int errcode) {
    //printf("on_login_complete | code[%d]\n", errcode);
}

/**
 * 在线状态变化通知， 状态（status）取值为 11 表示 在线， 取值为 21 表示  离线
 * old是前一个状态，new是变化后的状态（当前）
 */
void on_online_status(int old, int new) {
	//printf("online status: %s\n", 11 == new ? "true" : "false"); 
}


// 绑定列表变化通知 pBinderList是最新的绑定者列表
void on_binder_list_change(int error_code, tx_binder_info * pBinderList, int nCount)
{

	/*if (err_null != error_code)
	{
		printf("on_binder_list_change failed, errcode:%d\n", error_code);
		return;
	}

	printf("on_binder_list_change, %d binder: \n", nCount);
	int i = 0;
	for (i = 0; i < nCount; ++i )
	{
		printf("binder uin[%llu], nick_name[%s]\n", pBinderList[i].uin, pBinderList[i].nick_name);
	}*/
}




/**
 * 辅助函数：SDK的log输出回调
 * SDK内部调用改log输出函数，有助于开发者调试程序
 * 回调函数参数说明：
 *         level   log级别 取值有 0 严重错误；1 错误；2 警告；3 提示；4 调试
 *         module  模块
 *         line    行号
 *         message log内容
 */
void log_func(int level, const char* module, int line, const char* message)
{
	printf("%s\n", message);
}


/**
 * SDK初始化
 * 例如：
 * （1）填写设备基本信息
 * （2）打算监听哪些事件，事件监听的原理实际上就是设置各类消息的回调函数，
 * 	例如设置Datapoint消息通知回调：
 * 	开发者应该定义如下的 my_on_receive_data_point 函数，将其赋值tx_data_point_notify对象中对应的函数指针，并初始化：
 *
 * 			tx_data_point_notify msgOnRecv= {0};
 * 			msgOnRecv.on_receive_data_point = my_on_receive_data_point;
 * 			tx_init_data_point(&msgOnRecv);
 *
 * 	那么当SDK内部的一个线程收到对方发过来的DataPoint消息后（通过服务器转发），将同步调用 msgOnRecv.on_receive_data_point
 */
bool initDevice() {
	// 读取 license
	unsigned char license[1024] = {0};
	int nLicenseSize = 0;
	printf("now initdevice //////////////////////////////////////////////////////\n");





	
	/*if (!readBufferFromFile("/usr/share/brxydata/licence.sign.file.txt", license, sizeof(license), &nLicenseSize)) {
		printf("[error]get license from file failed...\n");
		return false;
	}*/
	memmove(license,base_info.qqlicense,1024);

	

	// 读取guid
	unsigned char guid[32] = {0};
	int nGUIDSize = 0;
	/*if(!readBufferFromFile("/usr/share/brxydata/Guid_file.txt", guid, sizeof(guid), &nGUIDSize)) {
		printf("[error]get guid from file failed...\n");
		return false;
	}*/
	memmove(guid,base_info.sn,32);
	//读取服务器公钥
    char svrPubkey[256] = {0};
    int nPubkeySize = 0;
    if (!readBufferFromFile("/usr/share/brxydata/1700002626.pem", svrPubkey, sizeof(svrPubkey), &nPubkeySize))
    {
        printf("[error]get svrPubkey from file failed...\n");
        return NULL;
    }

    // 设备的基本信息
    tx_device_info info = {0};
    info.os_platform            = "Linux";
    info.device_name            = "DTS-K3A-MINI";
    info.device_serial_number   = guid;
    info.device_license         = license;
    info.product_version        = 101;  //软件版本号，可以做成一个变量，读取固定值，升级后需要改变
    info.network_type			= network_type_wifi;
    info.product_id             = 1700002626;  //产品id
    info.test_mode              = 0;  //0,接入正式环境   1登录到测试环境
    info.server_pub_key         = svrPubkey;  //服务器公钥
    info.run_mode               = 0; //正常模式  1, //低功耗模式

	// 设备登录、在线状态、消息等相关的事件通知
	// 注意事项：
	// 如下的这些notify回调函数，都是来自硬件SDK内部的一个线程，所以在这些回调函数内部的代码一定要注意线程安全问题
	// 比如在on_login_complete操作某个全局变量时，一定要考虑是不是您自己的线程也有可能操作这个变量
    tx_device_notify notify      = {0};
    notify.on_login_complete     = on_login_complete;
    notify.on_online_status      = on_online_status;
    notify.on_binder_list_change = on_binder_list_change;

    // SDK初始化目录，写入配置、Log输出等信息
    // 为了了解设备的运行状况，存在上传异常错误日志 到 服务器的必要 ,其中的各个capicity字段都是用于限定目录大小所用的
    tx_init_path init_path = {0};

    // system_path：SDK会在该目录下写入保证正常运行必需的配置信息,system_path 是最重要的一个路径，必须要保证SDK对此目录的可读写权限
    // system_path_capicity：是允许SDK在该目录下最多写入多少字节的数据（最小大小：10K，建议大小：100K）
    init_path.system_path = "/usr/share/brxydata";
    init_path.system_path_capicity = 100 * 1024;

    // app_path：用于保存运行中产生的log或者crash堆栈
    // app_path_capicity：同上，（最小大小：300K，建议大小：1M）
    init_path.app_path = "/usr/share/brxydata";
    init_path.app_path_capicity = 1024 * 1024;

    // temp_path：可能会在该目录下写入临时文件
    // temp_path_capicity：这个参数实际没有用的，可以忽略
    init_path.temp_path = "/usr/share/brxydata";
    init_path.temp_path_capicity = 10 * 1024;

    // 设置log输出函数，如果不想打印log，则无需设置。
    // 建议开发在开发调试阶段开启log，在产品发布的时候禁用log。
    tx_set_log_func(log_func);



	//挂接datapoint的处理函数
    tx_data_point_notify dataPointNotify= {0};
    dataPointNotify.on_receive_data_point = my_on_receive_data_point;
    tx_init_data_point(&dataPointNotify);





	//ota升级的事件通知
	tx_ota_notify ota_notify = {0};
	ota_notify.on_new_pkg_come 		= cb_on_new_pkg_come;
	ota_notify.on_download_progress = cb_on_download_progress;
	ota_notify.on_download_complete = cb_on_download_complete;
	ota_notify.on_update_confirm 	= cb_on_update_confirm;
	/**
	* 初始化OTA模块
	* param: replace_timeout 您希望手机APP最多等待多少时间提示升级超时，您需要保证绝大多数情况下：
	*文件替换 + 设备重启的时间 < replace_timeout, 时间单位:秒
	* param: target_pathname 您希望我们把升级包下载到哪个目录下的哪个文件，需要填写带文件名的完整路径
	*/
	tx_init_ota(&ota_notify, 10*60, "/tmp/update_pkg.tar");


    // 初始化SDK，若初始化成功，则内部会启动一个线程去执行相关逻辑，该线程会持续运行，直到收到 exit 调用
	/*int ret = tx_init_device(&info, &notify, &init_path);
	if (err_null == ret) {
		printf(" >>> tx_init_device success\n");
	}
	else {
		printf(" >>> tx_init_device failed [%d]\n", ret);
		sleep(10000);
		return false;
	}*/
	while(tx_init_device(&info, &notify, &init_path)!=err_null)
		{
			sleep(4);
		}
	
	return true;
}








/**
 * 博瑞星云数字教学一体机
 *
 * 业务处理
 *
 *2016-01-13
 */







void on_receive_controll_dts(unsigned long long from_id, tx_data_point  datapoint){
	//解析json字符串(此处省略冗长的解析代码)
	 char* json = datapoint.value;
	 printf("on_receive_controll_dts, json:%s\n", json);
	 //log_func(4, "business", 23, json);
	// Document document;
	 //document.Parse(json);
	 //TODO 控制设备 包含设备类型 操作码  操作值
	char *devicetype;
	char *operation;
	char *mac;
	char *value;
	char *return_success="{\"result\":\"true\", \"msg\":\"success\"}";
	char *return_failed="{\"result\":\"false\",\"msg\":\"failed\"}";
	int ret = 0;

	cJSON *alldata=cJSON_Parse(json);
	devicetype=cJSON_GetObjectItem(alldata,"deviceType")->valuestring;

	if(strcmp(devicetype,"DTS_HOST")==0)
		{
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(strcmp(operation,"START")==0)
				{
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_open_system();
					usleep(200000);
					read_status_from_mcu();
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					//return;
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_shutdown_system();
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					//return;
						
				}
		}
	else if(strcmp(devicetype,"DTS_PC")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			
			if(strcmp(operation,"START")==0)
				{
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_open_pc();
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					//return;
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_shutdown_pc();
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					//return;
						
				}
		}
	else if(strcmp(devicetype,"DTS_DISPLAYER")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			
			if(strcmp(operation,"START")==0)
				{
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_open_projector();
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					//return;
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_shutdown_projector();
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					//return;
				}
		}
	else if(strcmp(devicetype,"DTS_CHANNEL")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(1==0)
				{
					printf("dts_host is shutdown,so can not control dts_channel\n");
					tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_host is shutdown,so can not control dts_channel\"}" , datapoint.seq, ret, 0, 0);
					//return;
				}
			else //if(strcmp(operation,"START")==0)
				{
					
					//ep_change();
					if(strcmp(operation,"START")==0)
						{
							pthread_mutex_lock(&projector_info.mutex);
							projector_info.send_return_status=0;
							mcu_change_channel(2);
							usleep(200000);
							if(projector_info.send_return_status!=1)
								{
							
									tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
								}
							else
								{
							
									tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
								}
							pthread_mutex_unlock(&projector_info.mutex);
							//return;
						}
					if(strcmp(operation,"SHUTDOWN")==0)
						{
							pthread_mutex_lock(&projector_info.mutex);
							projector_info.send_return_status=0;
							mcu_change_channel(1);
							usleep(200000);
							if(projector_info.send_return_status!=1)
								{
							
									tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
								}
							else
								{
							
									tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
								}
							pthread_mutex_unlock(&projector_info.mutex);
							//return;
						}
					
					

					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
				}
				
		}
	else if(strcmp(devicetype,"DTS_BOOTH")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(1==0)
				{
					printf("dts_host is shutdown,so can not control dts_booth\n");
					tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_host is shutdown,so can not control dts_booth\"}" , datapoint.seq, ret, 0, 0);
					//return;
				}
			else if(strcmp(operation,"START")==0)
				{
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_open_stand();
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					//return;
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_shutdown_stand();
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					//return;
				}
		}
	else if(strcmp(devicetype,"DTS_SCREEN")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(1==0)
				{
					printf("dts_screen is shutdown,so can not control dts_screen dts_booth\n");
					tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_screen is shutdown,so can not control dts_screen \"}" , datapoint.seq, ret, 0, 0);
					//return;
				}
			else if(strcmp(operation,"START")==0)
				{
					
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_screen_control(1);
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					//return;
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_screen_control(0);
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					//return;
				}
		}
	else if(strcmp(devicetype,"DTS_VOLUME")==0)
		{
			int vo=0;
			int vo1=0;
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			value=cJSON_GetObjectItem(alldata,"operationValue")->valuestring;
			vo=atoi(value);
			vo1=vo/2;
			if((vo1==0)&&(vo==0))
				{
					volume_3105.volume=0;
				}
			else if((vo1>0)&&(vo1<9))
				{
					volume_3105.volume=vo1;
				}
			pthread_mutex_lock(&projector_info.mutex);
			projector_info.send_return_status=0;
			mcu_config_volume(volume_3105.volume);
			usleep(200000);
			if(projector_info.send_return_status!=1)
				{
							
					tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
				}
			else
				{
							
					tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
				}
			pthread_mutex_unlock(&projector_info.mutex);
			//return;
		}
	else if(strcmp(devicetype,"DTS_SWITCH")==0)
		{
			int valume;
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			mac=cJSON_GetObjectItem(alldata,"deviceID")->valuestring;
			value=cJSON_GetObjectItem(alldata,"operationValue")->valuestring;
			valume=atoi(value);
			if(strcmp(operation,"START")==0)
				{
					turn_on_off_wifi_switch1(1,mac,valume);
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					turn_on_off_wifi_switch1(0,mac,valume);
				}
			
			tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
			return;
		}
	else
		{
			tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"invalidity control order\"}" , datapoint.seq, ret, 0, 0);
		}
	cJSON_Delete(alldata);
	return;


	 //指令处理结果回传到手机(注意seq一定要配对，不要随便乱填哦)
	// int ret = 0;
	 // 应答收到的datapoint
	 // [in]    from_client:     datapoint信令来自某个绑定者，       必须和已收到的datapoint时的from_client一致，否则会被过滤
	 // [in]    id:              应答的datapoint对应的ID，          必须和已收到的datapoint的ID一致，否则会被过滤
	 // [in]    value:           应答的datapoint的自定义数据
	 // [in]    seq:             应答的datapoint对应的seq，         必须和已收到的datapoint的seq一致，否则会被过滤
	 // [out]   unsigned int ret_code
	 // [out]   cookie:          返回调用cookie
	 // [in]    ret_callback:    发送结果callback

}

void on_receive_controll_dts_old(unsigned long long from_id, tx_data_point  datapoint){
	//解析json字符串(此处省略冗长的解析代码)
	 char* json = datapoint.value;
	 printf("on_receive_controll_dts, json:%s\n", json);
	 //log_func(4, "business", 23, json);
	// Document document;
	 //document.Parse(json);
	 //TODO 控制设备 包含设备类型 操作码  操作值
	char *devicetype;
	char *operation;
	char *mac;
	char *value;
	char *return_success="{\"result\":\"true\", \"msg\":\"success\"}";
	char *return_failed="{\"result\":\"false\",\"msg\":\"failed\"}";
	int ret = 0;

	cJSON *alldata=cJSON_Parse(json);
	devicetype=cJSON_GetObjectItem(alldata,"deviceType")->valuestring;

	if(strcmp(devicetype,"DTS_HOST")==0)
		{
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(strcmp(operation,"START")==0)
				{
					if(device_status.k4==1)
						{
							printf("dts aready started\n");
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
					else
						{
							open_machine();
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					if(device_status.k4==0)
						{
							
							printf("dts aready shutdown\n");
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready shutdown\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
					else
						{
							close_machine();
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
						
				}
		}
	else if(strcmp(devicetype,"DTS_PC")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(device_status.k4==0)
				{
					printf("dts_host is shutdown,so can not control pc\n");
					tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_host is shutdown,so can not control pc\"}" , datapoint.seq, ret, 0, 0);
					return;
				}
			else if(strcmp(operation,"START")==0)
				{
					if(device_status.pc==1)
						{
							printf("pc aready started\n");
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"pc aready dtarted\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
					else
						{

							pc_open();

							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					if(device_status.pc==0)
						{
							
							printf("pc aready shutdown\n");
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"pc aready shutdown\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
					else
						{

							pc_close();

							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
						
				}
		}
	else if(strcmp(devicetype,"DTS_DISPLAYER")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(device_status.k4==0)
				{
					printf("dts_host is shutdown,so can not control dts_displayer\n");
					tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_host is shutdown,so can not control dts_displayer\"}" , datapoint.seq, ret, 0, 0);
					return;
				}
			else if(strcmp(operation,"START")==0)
				{
					if(device_status.projector==1)
						{
							printf("dts_displayer aready started\n");
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_displayer aready dtarted\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
					else
						{

							projector_open();

							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					if(device_status.projector==0)
						{
							
							printf("dts_displayer aready shutdown\n");
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_displayer aready shutdown\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
					else
						{

							projector_close();

							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
						
				}
		}
	else if(strcmp(devicetype,"DTS_CHANNEL")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(device_status.k4==0)
				{
					printf("dts_host is shutdown,so can not control dts_channel\n");
					tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_host is shutdown,so can not control dts_channel\"}" , datapoint.seq, ret, 0, 0);
					return;
				}
			else //if(strcmp(operation,"START")==0)
				{
					
					//ep_change();
					if(strcmp(operation,"START")==0)
						{
							if(device_status.ep==1)
								{
									printf("dts_stand aready started\n");
									tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_booth aready dtarted\"}" , datapoint.seq, ret, 0, 0);
									return;
								}
							else
								{
									ep_switch_to_out();
									tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
									return;
								}
						}
					if(strcmp(operation,"SHUTDOWN")==0)
						{
							if(device_status.ep==0)
								{
									printf("dts_stand aready shutdown\n");
									tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_booth aready dtarted\"}" , datapoint.seq, ret, 0, 0);
									return;
								}
							else
								{
									ep_switch_to_in();
									tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
									return;
								}
						}
					
					

					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
				}
				
		}
	else if(strcmp(devicetype,"DTS_BOOTH")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(device_status.k4==0)
				{
					printf("dts_host is shutdown,so can not control dts_booth\n");
					tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_host is shutdown,so can not control dts_booth\"}" , datapoint.seq, ret, 0, 0);
					return;
				}
			else if(strcmp(operation,"START")==0)
				{
					if(device_status.stand_lock==1)
						{
							
							printf("dts_booth aready started\n");
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_booth aready dtarted\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
					else
						{

							
							//stand_close();
							stand_open();

							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					if(device_status.stand_lock==0)
						{
							
							printf("dts_booth aready shutdown\n");
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_booth aready shutdown\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
					else
						{

							//stand_open();
							stand_close();
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
						
				}
		}
	else if(strcmp(devicetype,"DTS_SCREEN")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(device_status.k4==0)
				{
					printf("dts_screen is shutdown,so can not control dts_screen dts_booth\n");
					tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_screen is shutdown,so can not control dts_screen \"}" , datapoint.seq, ret, 0, 0);
					return;
				}
			else if(strcmp(operation,"START")==0)
				{
					/*if(info.dts_screen==1)
						{
							printf("dts_screen aready started\n");
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_screen aready dtarted\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
					else
						{

							info.dts_screen=1;

							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							return;
						}*/
					turn_on_off_screen(1);
					tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
						
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					/*if(info.dts_screen==0)
						{
							
							printf("dts_screen aready shutdown\n");
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_screen aready shutdown\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
					else
						{

							info.dts_screen=0;

							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							return;
						}*/
					turn_on_off_screen(0);
					tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
				}
		}
	else if(strcmp(devicetype,"DTS_VOLUME")==0)
		{
			int vo=0;
			int vo1=0;
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			value=cJSON_GetObjectItem(alldata,"operationValue")->valuestring;
			vo=atoi(value);
			vo1=vo/2;
			if((vo1==0)&&(vo==0))
				{
					volume_3105.volume=0;
				}
			else if((vo1>0)&&(vo1<9))
				{
					volume_3105.volume=vo1;
				}
			pthread_mutex_lock(&i2c_mutex);
			i2c_write(i2c_address_wm,0x1c,wm8776_volume_table[volume_3105.volume]);
			i2c_write(i2c_address_wm,0x15,0x8f);//打开通道 不静音
			pthread_mutex_unlock(&i2c_mutex);

			
			printf("adjust valume to %d\n",vo);
			tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
			return;
		}
	else if(strcmp(devicetype,"DTS_SWITCH")==0)
		{
			int valume;
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			mac=cJSON_GetObjectItem(alldata,"deviceID")->valuestring;
			value=cJSON_GetObjectItem(alldata,"operationValue")->valuestring;
			valume=atoi(value);
			if(strcmp(operation,"START")==0)
				{
					turn_on_off_wifi_switch1(1,mac,valume);
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					turn_on_off_wifi_switch1(0,mac,valume);
				}
			
			tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
			return;
		}
	else
		{
			tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"invalidity control order\"}" , datapoint.seq, ret, 0, 0);
		}
	return;


	 //指令处理结果回传到手机(注意seq一定要配对，不要随便乱填哦)
	// int ret = 0;
	 // 应答收到的datapoint
	 // [in]    from_client:     datapoint信令来自某个绑定者，       必须和已收到的datapoint时的from_client一致，否则会被过滤
	 // [in]    id:              应答的datapoint对应的ID，          必须和已收到的datapoint的ID一致，否则会被过滤
	 // [in]    value:           应答的datapoint的自定义数据
	 // [in]    seq:             应答的datapoint对应的seq，         必须和已收到的datapoint的seq一致，否则会被过滤
	 // [out]   unsigned int ret_code
	 // [out]   cookie:          返回调用cookie
	 // [in]    ret_callback:    发送结果callback

}

/*
function DeviceType(){
    this.dtsHost = "DTS_HOST";  //教学一体机主机
    this.dtsPC = "DTS_PC";  //内置电脑
    this.dtsDisplayer = "DTS_DISPLAYER";  //外显
    this.dtsSwitch = "DTS_SWITCH";  //开关
    this.dtsVolume = "DTS_VOLUME";  //音量
    this.dtsRfid = "DTS_RFID";  //RFID刷卡器
    this.dtsSensor = "DTS_SENSOR";  //温度湿度亮度传感器
    this.dtsChannel = "DTS_CHANNEL";  //显示通道
    this.dtsBooth = "DTS_BOOTH";  //展台
    this.dtsScreen = "DTS_SCREEN"; //荧幕控制器
}
function Operation(){
    this.start = "START";  //打开设备
    this.shutdown = "SHUTDOWN"; //关闭设备
    this.restart = "RESTART"; //重启设备
    this.adjust = "ADJUST"; //设备在打开状态下 调节设备
    this.read = "READ";  //设备在打开状态下 读取值
    this.enable = "ENABLE";  //启用设备
    this.disable = "DISABLE";  // 禁用设备
}
function DeviceStatus(){
    this.standby = "STANDBY";//待机状态
    this.working = "WORKING";//运行状态
    this.offLine = "OFFLINE";//离线状态
}
deviceType  operationCode operationValue 取值范围如上
{
    "deviceType":"DTS_VOLUME",
    "operationCode":"ADJUST",
    "operationValue":"12"
}
{
    "deviceType":"DTS_SWITCH",
    "operationCode":"START/SHUTDOWN",
    "mac":"11:22:33:44:55:66",
    "operationValue":"1/2 代表面板上的第一个或者第二个开关"
}
{
    "result":"true/false",
    "msg":"成功/失败"
}


*/




void on_receive_query_all_status(unsigned long long from_id, tx_data_point  datapoint){
	//解析json字符串(此处省略冗长的解析代码)
	 char* json = datapoint.value;
	 //TODO 查询设备和外设状态并返回 ret的json数据
	 printf("on_receive_query_all_status, json:%s\n", json);

	char info_to_report[2048];
	json_object *screen=json_object_new_object();
	json_object *switchn=json_object_new_array();
	json_object *sensor=json_object_new_object();
	json_object *rfid=json_object_new_object();

	json_object *host=json_object_new_object();
	json_object *pc=json_object_new_object();
	json_object *display=json_object_new_object();
	json_object *booth=json_object_new_object();	

	json_object *mainjson=json_object_new_object();
	json_object *mainjson1=json_object_new_object();
	//host
	pthread_mutex_lock(&nopoll_info.mutex);
	char *host_status;
	if(device_status.k4==1)
		{
			host_status="WORKING";
		}
	else
		{
			host_status="STANDBY";
		}

	json_object_object_add(host,"status",json_object_new_string(host_status));


	//pc
	char *pc_status;
	if(device_status.pc==1)
		{
			pc_status="WORKING";
		}
	else
		{
			pc_status="STANDBY";
		}
	

	json_object_object_add(pc,"identify",json_object_new_string("pc"));
	json_object_object_add(pc,"status",json_object_new_string(pc_status));


	//booth
	char *booth_status;
	if(device_status.stand_lock==1)
		{
			booth_status="WORKING";
		}
	else
		{
			booth_status="STANDBY";
		}
	

	json_object_object_add(booth,"identify",json_object_new_string("booth"));
	json_object_object_add(booth,"status",json_object_new_string(booth_status));


	//display
	char *display_status;
	if(device_status.projector==1)
		{
			display_status="WORKING";
		}
	else
		{
			display_status="STANDBY";
		}
	

	json_object_object_add(display,"identify",json_object_new_string("dtsDisplayer"));
	json_object_object_add(display,"status",json_object_new_string(display_status));









/**screen***/

	/*json_object *status_screen=json_object_new_object();
	json_object *scree=json_object_new_array();
	json_object **screen_list;
	///////////////////////////////////////////////////////////////////////
	if(wifi_screenheader.number>0)
			{
				int i=0;
				printf("screen is used\n");
				struct wifi_screen *a;
				a=wifi_screenheader.next;
				char *status;
				screen_list=(json_object *)malloc(wifi_screenheader.number*sizeof(json_object *));
				for(i=0;i<wifi_screenheader.number;i++)
					{
						screen_list[i]=json_object_new_object();
						json_object_object_add(screen_list[i],"deviceIdentify",json_object_new_string(a->name));
						if(a->status>0)
							{
								status="WORKING";
							}
						if(a->status==0)
							{
								status="OFFLINE";
							}
						json_object_object_add(screen_list[i],"deviceStatus",json_object_new_string(status));
	
						if(a->del==0)
						json_object_array_add(scree,screen_list[i]);
	
						if(i!=(wifi_screenheader.number-1))
							{
								a=a->next;
							}
					}
			json_object_object_add(screen1, "deviceType",json_object_new_string("DTS_SCREEN"));
			json_object_object_add(screen1,"status",scree);
				}*/




	














































	int have_sensor=0;
	//sensor
	if(wifi_sensorheader.number>0)
		{
	struct wifi_sensor *sensor_a;
	sensor_a=wifi_sensorheader.next;
	while(1)
		{
			if(sensor_a->del==1)
				{
					if(sensor_a->last==1)
						{
							have_sensor=0;
							break;
						}
					else
						{
							sensor_a=sensor_a->next;
							continue;
						}
				}
			else
				{
					have_sensor=1;
					break;
				}
		}
	char *sensor_status;
	if(have_sensor==1)
		{
	if(sensor_a->status>=1)
		{
			display_status="WORKING";
		}
	else
		{
			display_status="OFFLINE";
		}

	json_object_object_add(sensor,"identify",json_object_new_string("Sensor"));
	json_object_object_add(sensor,"status",json_object_new_string(display_status));
	json_object_object_add(sensor,"temperature",json_object_new_int(sensor_a->temperature));
	json_object_object_add(sensor,"humidness",json_object_new_int(sensor_a->humidity));
	json_object_object_add(sensor,"illumination",json_object_new_int(sensor_a->illumination));
		}
	}

	//rfid
	int have_rfid=0;
	if(wifi_rfidheader.number>0)
		{
	struct wifi_rfid *rfid_a;
	rfid_a=wifi_rfidheader.next;
	while(1)
		{
			if(rfid_a->del==1)
				{
					if(rfid_a->last==1)
						{
							have_rfid=0;
							break;
						}
					else
						{
							rfid_a=rfid_a->next;
							continue;
						}
				}
			else
				{
					have_rfid=1;
					break;
				}
		}
	char *rfid_status;
	if(have_rfid==1)
		{
	if(rfid_a->status>=1)
		{
			rfid_status="WORKING";
		}
	else
		{
			rfid_status="OFFLINE";
		}
	//json_object *rfid1=json_object_new_object();
	json_object_object_add(rfid,"identify",json_object_new_string("RFID"));
	json_object_object_add(rfid,"status",json_object_new_string(rfid_status));
	//json_object_array_add(rfid,rfid1);
		}
		}

	//switch




	struct switch_json_header switch_jsonheader;
	switch_jsonheader.number=0;
	switch_jsonheader.next=NULL;
	if(wifi_switchheader.number>0)
		{
			struct wifi_switch *switch_a;
			struct switch_json *switch_json_a;
			struct switch_json *switch_json_b;
			int lamp_nu=0;
			
			char lamp_numb[5];
			
			switch_a=wifi_switchheader.next;
			while(1)
				{
					
					if(switch_a->del==1)
						{
							switch_a=switch_a->next;
							continue;
						}
					else
						{
							//switch_json_a=(struct switch_json *)malloc(sizeof(struct switch_json));
							while((switch_json_a=(struct switch_json *)malloc(sizeof(struct switch_json)))==NULL)
								{
									printf("malloc false\n");
									sleep(2);
								}
							switch_json_a->lamp_number=switch_a->lamp_number;
							switch_json_a->last=1;
							switch_json_a->switch_array=json_object_new_array();
							switch_json_a->switch_switch=json_object_new_object();
							//switch_json_a->lamps=(json_object **)malloc(switch_json_a->lamp_number*sizeof(json_object *));
							while((switch_json_a->lamps=(json_object **)malloc(switch_json_a->lamp_number*sizeof(json_object *)))==NULL)
								{
									printf("malloc false\n");
									sleep(2);
								}
							for(lamp_nu=0;lamp_nu<switch_json_a->lamp_number;lamp_nu++)
								{
									switch_json_a->lamps[lamp_nu]=json_object_new_object();
									memset(lamp_numb,0,5);
									sprintf(lamp_numb,"%d",(lamp_nu+1));
									json_object_object_add(switch_json_a->lamps[lamp_nu],"identify",json_object_new_string(lamp_numb));
									json_object_object_add(switch_json_a->lamps[lamp_nu],"status",json_object_new_string(switch_a->lamp[lamp_nu]));
									json_object_array_add(switch_json_a->switch_array,switch_json_a->lamps[lamp_nu]);
								}

							
							json_object_object_add(switch_json_a->switch_switch,"name",json_object_new_string(switch_a->name));
							json_object_object_add(switch_json_a->switch_switch,"mac",json_object_new_string(switch_a->mac));
							json_object_object_add(switch_json_a->switch_switch,"switch",switch_json_a->switch_array);
							json_object_array_add(switchn,switch_json_a->switch_switch);


							if(switch_jsonheader.number==0)
								{
									switch_jsonheader.next=switch_json_a;
									switch_json_b=switch_json_a;
									switch_jsonheader.number++;
								}
							else
								{
									switch_json_b->next=switch_json_a;
									switch_json_b->last=0;
									switch_json_b=switch_json_a;
									switch_jsonheader.number++;
								}
							if(switch_a->last==1)
								{
									break;
								}
							else
								{
									switch_a=switch_a->next;
								}
						}
				}
		}

	

	/*json_object *switch_array=json_object_new_array();
	json_object *switch_array1=json_object_new_array();
	json_object *switch_array2=json_object_new_array();

	json_object *switch1=json_object_new_object();
	json_object *lamp1_1=json_object_new_object();
	json_object *lamp1_2=json_object_new_object();

	json_object *switch2=json_object_new_object();
	json_object *lamp2_1=json_object_new_object();
	json_object *lamp2_2=json_object_new_object();

	char *lamp1_1_status,*lamp1_2_status,*lamp2_1_status,*lamp2_2_status;
	if(info.switch1_lamp1==1)
		{
			lamp1_1_status="WORKING";
		}
	else
		{
			lamp1_1_status="STANDBY";
		}
	
	if(info.switch1_lamp2==1)
		{
			lamp1_2_status="WORKING";
		}
	else
		{
			lamp1_2_status="STANDBY";
		}
	
	if(info.switch2_lamp1==1)
		{
			lamp2_1_status="WORKING";
		}
	else
		{
			lamp2_1_status="STANDBY";
		}
	
	if(info.switch2_lamp2==1)
		{
			lamp2_2_status="WORKING";
		}
	else
		{
			lamp2_2_status="STANDBY";
		}
	json_object_object_add(lamp1_1,"identify",json_object_new_string("1"));
	json_object_object_add(lamp1_1,"status",json_object_new_string(lamp1_1_status));
	json_object_object_add(lamp1_2,"identify",json_object_new_string("2"));
	json_object_object_add(lamp1_2,"status",json_object_new_string(lamp1_2_status));

	json_object_object_add(lamp2_1,"identify",json_object_new_string("1"));
	json_object_object_add(lamp2_1,"status",json_object_new_string(lamp2_1_status));
	json_object_object_add(lamp2_2,"identify",json_object_new_string("2"));
	json_object_object_add(lamp2_2,"status",json_object_new_string(lamp2_2_status));

	json_object_array_add(switch_array1,lamp1_1);
	json_object_array_add(switch_array1,lamp1_2);

	json_object_array_add(switch_array2,lamp2_1);
	json_object_array_add(switch_array2,lamp2_2);

	json_object_object_add(switch1,"name",json_object_new_string("switch1"));
	json_object_object_add(switch1,"mac",json_object_new_string(info.switch1_mac));
	json_object_object_add(switch1,"switch",switch_array1);

	json_object_object_add(switch2,"name",json_object_new_string("switch2"));
	json_object_object_add(switch2,"mac",json_object_new_string(info.switch2_mac));
	json_object_object_add(switch2,"switch",switch_array2);

	json_object_array_add(switchn,switch1);
	json_object_array_add(switchn,switch2);*/
	
 	int have_screen=0;
	//screen
	if(wifi_screenheader.number>0)
		{
	struct wifi_screen *screen_a;
	screen_a=wifi_screenheader.next;
	while(1)
		{
			if(screen_a->del==1)
				{
					if(screen_a->last==1)
						{
							have_screen=0;
							break;
						}
					else
						{
							screen_a=screen_a->next;
							continue;
						}
				}
			else
				{
					have_screen=1;
					break;
				}
		}
	
		
	char *screen_status;
	
	if(have_screen==1)
		{
	if(screen_a->status>=1)
		{
			screen_status="WORKING";
		}
	else
		{
			screen_status="OFFLINE";
		}
	json_object_object_add(screen,"identify",json_object_new_string("screen"));
	json_object_object_add(screen,"status",json_object_new_string(screen_status));
		}
		}

	//sunmary
	
	json_object_object_add(mainjson1,"result",json_object_new_int(1));
	json_object_object_add(mainjson1,"function",json_object_new_string("reportstatus"));
	json_object_object_add(mainjson1,"msg",json_object_new_string("chenggong"));
	json_object_object_add(mainjson1,"dtsHost",host);
	json_object_object_add(mainjson1,"dtsPC",pc);
	json_object_object_add(mainjson1,"dtsVolume",json_object_new_int(volume_3105.volume*2));
	json_object_object_add(mainjson1,"dtsChannel",json_object_new_int(device_status.ep-1));
	json_object_object_add(mainjson1,"dtsBooth",booth);
	json_object_object_add(mainjson1,"dtsDisplayer",display);
	if(have_sensor==1)
		{
	json_object_object_add(mainjson1,"dtsSensor",sensor);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsSensor",json_object_new_string("null"));
		}
	if(have_rfid==1)
		{
	json_object_object_add(mainjson1,"dtsRfid",rfid);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsRfid",json_object_new_string("null"));
		}
	if(switch_jsonheader.number>0)
		{
	json_object_object_add(mainjson1,"dtsSwitch",switchn);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsSwitch",json_object_new_string("null"));
		}
	if(have_screen==1)
		{
	json_object_object_add(mainjson1,"dtsScreen",screen);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsScreen",json_object_new_string("null"));
		}

	json_object_object_add(mainjson,"dtsinfo",mainjson1);
	memset(info_to_report,0,2048);
	sprintf(info_to_report,"%s",json_object_to_json_string(mainjson));

	int lent;
	lent=strlen(info_to_report);
	int i=0;
	int j=0;
	printf("dtsinfo = %s\n%d\n",info_to_report,lent);

	/*while(info_to_report[i]!='\0')
		{
			if(info_to_report[i]!=' ')
				{
					info_to_report[j]=info_to_report[i];
					j++;
					i++;
				}
			else
				{
					i++;
				}
		}
	info_to_report[j]='\0';
	lent=strlen(info_to_report);
	printf("dtsinfo = %s\n%d\n",info_to_report,lent);*/
	
	
	json_object_put(mainjson);
	json_object_put(mainjson1);
	json_object_put(host);
	json_object_put(pc);
	json_object_put(booth);
	json_object_put(display);
	json_object_put(sensor);
	json_object_put(rfid);
	json_object_put(switchn);
	json_object_put(screen);
	if(switch_jsonheader.number>0)
		{
			struct switch_json *a;
			int a1=0;
			while(1)
				{	
					a=switch_jsonheader.next;
					json_object_put(a->switch_switch);
					json_object_put(a->switch_array);
					for(a1=0;a1<a->lamp_number;a1++)
						{
							json_object_put(a->lamps[a1]);
						}
					free(a->lamps);
					if(a->last!=1)
						{
							switch_jsonheader.next=a->next;
							free(a);
						}
					else
						{
							free(a);
							break;
						}
					
				}
		}
	pthread_mutex_unlock(&nopoll_info.mutex);
	//json_object_put(rfid1);
	/*json_object_put(switch_array);
	json_object_put(switch_array1);
	json_object_put(switch_array2);
	json_object_put(switch1);
	json_object_put(lamp1_1);
	json_object_put(lamp1_2);
	json_object_put(switch2);
	json_object_put(lamp2_1);
	json_object_put(lamp2_2);*/
	 //指令处理结果回传到手机(注意seq一定要配对，不要随便乱填哦)
	 tx_ack_data_point(from_id, datapoint.id, info_to_report, datapoint.seq, 0, 0, 0);

}
/*
{
    "dtsinfo":{
        "result":true,
        "msg":"chenggong",
        "dtsHost":{
            "status":"STANDBY"
        },
        "dtsPC":{
            "identify":"pc",
            "status":"STANDBY"
        },
        "dtsVolume":14,
        "dtsChannel":0,
        "dtsBooth":{
            "identify":"booth",
            "status":"WORKING"
        },
        "dtsDisplayer":{
            "identify":"dtsDisplayer",
            "status":"OFFLINE"
        },
        "dtsSensor":{
            "identify":"Sensor",
            "status":"OFFLINE",
            "temperature":20,
            "humidness":70,
            "illumination":500
        },
        "dtsRfid":[
            {
                "identify":"RFID",
                "status":"OFFLINE"
            }
        ],
        "dtsSwitch":[
            {
                "name":"switch1",
                "mac":"11:22:33:44:55:66",
                "switch":[
                    {
                        "identify":"1",
                        "status":"OFFLINE"
                    },
                    {
                        "identify":"2",
                        "status":"OFFLINE"
                    }
                ]
            },
            {
                "name":"switch2",
                "mac":"11:22:33:44:55:88",
                "switch":[
                    {
                        "identify":"1",
                        "status":"OFFLINE"
                    },
                    {
                        "identify":"2",
                        "status":"OFFLINE"
                    }
                ]
            }
        ],
        "dtsScreen":{
            "identify":"dtsScreen",
            "status":"OFFLINE"
        }
    }
}

*/



//主动上报数据
void actice_report_data(){
	// 上报datapoint数据
	// [in]     id:             上报的datapoint ID
	// [in]     value:          上报的datapoint value
	// [out]    cookie:         返回调用cookie
	// [in]     ret_callback:   发送结果callback
	tx_report_data_point(100002223, "{result:success}", 0, 0);


}


// 处理设备关心的控制端指令
void my_on_receive_data_point(unsigned long long from_id, tx_data_point * data_points, int data_points_count)
{
  int i = 0;
  while (i < data_points_count) {
      if (data_points[i].id== 100002224) {
    	  //获取主机和外设的状态（内置PC、外显、展台、传感器、RFID、荧幕控制器、开关）
          on_receive_query_all_status(from_id, data_points[i]);
      }else if (data_points[i].id== 100002223) {
    	  //控制一体机设备、PC、展台、外显、通道切换、音量控制、开关
          on_receive_controll_dts(from_id, data_points[i]);
      }
      ++i;
  }
}

//读取文件中的内容
bool readBufferFromFile(char *pPath, unsigned char *pBuffer, int nInSize, int *pSizeUsed) {
	if (!pPath || !pBuffer) {
		return false;
	}

	int uLen = 0;
	FILE * file = fopen(pPath, "rb");
	if (!file) {
	    return false;
	}

	fseek(file, 0L, SEEK_END);
	uLen = ftell(file);
	fseek(file, 0L, SEEK_SET);

	if (0 == uLen || nInSize < uLen) {
		printf("invalide file or buffer size is too small...\n");
        fclose(file);
		return false;
	}

	*pSizeUsed = fread(pBuffer, 1, uLen, file);
	// bugfix: 0x0a is a lineend char, no use.
	if (pBuffer[uLen-1] == 0x0a)
	{
		*pSizeUsed = uLen - 1;
		pBuffer[uLen-1] = '\0';
	}

	printf("len:%d, ulen:%d\n",uLen, *pSizeUsed);
	fclose(file);
	return true;
}





/**
 * 有设备可用的新固件版本，手机会将查询到的固件包信息通知给设备
 * param: pkg_size 新的升级固件包的大小，单位为字节
 * param：title + desc 升级描述信息，如果您的智能设备没有显示屏，可以忽略
 * param: target_version 目标版本的版本号
 * return: 如果返回0，sdk将会开始启动升级包下载
 * 如果返回1 会提示用户设备端拒绝升级（一般是磁盘剩余空间问题）
 */
int cb_on_new_pkg_come(unsigned long long from, unsigned long long pkg_size, const char * title, const char * desc, unsigned int target_version)
{
    //todo
    return 0;
}

/**
 * 设备下载升级文件，并且实时地将进度通知给手机QQ，单位为字节
 * param：download_size 当前已经下载到的文件大小
 * param: total_size文件总计大小，您可以用  download_size/total_size 来计算百分比
 */
void cb_on_download_progress(unsigned long long download_size, unsigned long long total_size)
{
    //todo
}

/**
 * param: ret_code  0表示下载成功，其它表示失败
 * 0   成功
 * 2   未知错误
 * 3   当前请求需要用户验证401
 * 4   下载文件写失败，没有写权限/空间不足/文件路径不正确
 * 5   网络异常
 * 7   升级文件包不存在404
 * 8   服务器当前无法处理请求503
 * 9   下载被手q用户中止
 * 10  参数错误，url不合法
 * 11  升级包md5值校验失败，下载可能被劫持
 */
void cb_on_download_complete(int ret_code)
{
    //todo
}

/**
 * 手机等设备将文件下载完成后，会有一个最后的用户确认才会开始更新固件，因为替换文件可能需要一些时间，
 * 之后也有可能要重启设备，一个手机端的确认界面能给用户以心理上的等待预期。
 *
 * 所以您需要在收到这个通知以后，再开始启动固件升级
 */
void cb_on_update_confirm()
{
    //todo 在这里执行升级操作



	/**
	*  设备完成升级，需要给手机一个升级结果的反馈，告知手机是否升级成功了,否则手q会在一段时间
	*            之后告知用户升级超时，所以请务必实现此接口（如果设备重启，需要在设备上线之后调用）
	*  param: ret_code  0表示成功；1表示失败
	*  param: err_msg   升级失败的描述文案，升级失败时填写
	*/
	tx_ack_ota_result(0, "success");


}


void *qq_thread(void)
{
	//if ( !initDevice() ) {
	//	return -1;
	//}
	
	// 你可以在做其他相关的事情
	// ...

	/*char input[100];
	while (scanf("%s", input)) {
		if ( !strcmp(input, "quit") ) {
			tx_exit_device();
			break;
		}
		sleep(1);
	}*/
	qq_init_decision();
	
	return 0;
}











/****************************************************************************
上电初始化数据
取出brxy中的数据，存入结构体中；
uci.h头文件应用


******************************************************************************/
static int init_uci(void)
{

	struct uci_package *pkg = NULL;
	struct uci_section *section;
	struct uci_option *option;
	struct uci_context *ctx = NULL;
	char *uci_name,*uci_id,*uci_poe,*uci_openlimit,*uci_pc_automatic,*uci_qqlicense,*uci_serverip,*mediaa,*mediab;
	char *uci_baudrate,*uci_databyte,*uci_chech,*uci_stop;
	char *uci_start_code,*uci_shutdown_code,*uci_show_channel;
	char *uci_start_delay,*uci_shutdown_delay,*uci_interval;
	char *uci_high,*uci_low,*uci_volume;
	char *uci_section1,*uci_opentime,*uci_workmode;
	if(NULL==(ctx=uci_alloc_context()))
		{
			printf("get ctx for uci false\n");
			return -1;
		}
	
	if (UCI_OK!= uci_load(ctx, brxy, &pkg))
	{
		printf("init uci load false\n");
        goto loaderr;
	}

	//初始化base_info
	if(NULL==(section=uci_lookup_section(ctx,pkg,"baseinfo")))
		{
			printf("get baseinfo false\n");
			goto err;
		}
	else
		{
			if(NULL==(uci_name=uci_lookup_option_string(ctx,section,"name")))
				{
					printf("get name false\n");
					goto err;
				}
			else
				{
					if(strcmp(uci_name,"9999999999-9999999999")==0)
						{
							strcpy(base_info.name,base_info.id);
						}
					else
						{
							
							strcpy(base_info.name,uci_name);
						}
					
				}
			if(NULL==(uci_poe=uci_lookup_option_string(ctx,section,"poe")))
				{
					printf("get poe false\n");
					goto err;
				}
			else
				{
					base_info.poe=atoi(uci_poe);
				}
			if(NULL==(uci_qqlicense =uci_lookup_option_string(ctx,section,"qqlicense")))
				{
					printf("get qqlicense false\n");
					goto err;
				}
			else
				{
					strcpy(base_info.qqlicense,uci_qqlicense);
					
				}
			if(NULL==(uci_openlimit=uci_lookup_option_string(ctx,section,"openlimit")))
				{
					printf("get openlimit false\n");
					goto err;
				}
			else
				{
					base_info.openlimit=atoi(uci_openlimit);
				}
			if(NULL==(uci_pc_automatic=uci_lookup_option_string(ctx,section,"pc_automatic")))
				{
					printf("get openlimit false\n");
					goto err;
				}
			else
				{
					base_info.pc_automatic=atoi(uci_pc_automatic);
				}
			if(NULL==(uci_serverip=uci_lookup_option_string(ctx,section,"serverip")))
				{
					printf("get serverip false\n");
					goto err;
				}
			else
				{
					
					strcpy(base_info.ip,uci_serverip);
				}
			if(NULL==(mediaa=uci_lookup_option_string(ctx,section,"mediaa")))
				{
					printf("get mediaa false\n");
					goto err;
				}
			else
				{
					
					strcpy(base_info.mediaa,mediaa);
				}
			if(NULL==(mediab=uci_lookup_option_string(ctx,section,"mediab")))
				{
					printf("get mediab false\n");
					goto err;
				}
			else
				{
					
					strcpy(base_info.mediab,mediab);
				}
		}
	//初始化projector
	if(NULL==(section=uci_lookup_section(ctx,pkg,"projector")))
		{
			printf("get projector false\n");
			goto err;
		}
	else
		{
			//串口设置参数
			//波特率
			if(NULL==(uci_baudrate=uci_lookup_option_string(ctx,section,"baudrate")))
				{
					printf("get baudrate false\n");
					goto err;
				}
			else
				{
					projector_info.baudrate=atoi(uci_baudrate);
				}
			//字节数
			if(NULL==(uci_databyte=uci_lookup_option_string(ctx,section,"databyte")))
				{
					printf("get databyte false\n");
					goto err;
				}
			else
				{
					projector_info.databyte=atoi(uci_databyte);
				}
			//校验
			if(NULL==(uci_chech=uci_lookup_option_string(ctx,section,"check")))
				{
					printf("get check false\n");
					goto err;
				}
			else
				{
					projector_info.check=uci_chech[0];
				}
			//停止位
			if(NULL==(uci_stop=uci_lookup_option_string(ctx,section,"stop")))
				{
					printf("get stop false\n");
					goto err;
				}
			else
				{
					projector_info.stop=atoi(uci_stop);
				}
			//投影仪码参数
			//开机码
			if(NULL==(uci_start_code=uci_lookup_option_string(ctx,section,"start_code")))
				{
					printf("get uci_start_code false\n");
					goto err;
				}
			else
				{
					strcpy(projector_info.start_code,uci_start_code);
					
				}
			//关机吗
			if(NULL==(uci_shutdown_code=uci_lookup_option_string(ctx,section,"shutdown_code")))
				{
					printf("get uci_shutdown_code false\n");
					goto err;
				}
			else
				{
					strcpy(projector_info.shutdown_code,uci_shutdown_code);
					
				}
			//通道切换码
			if(NULL==(uci_show_channel=uci_lookup_option_string(ctx,section,"show_channel")))
				{
					printf("get uci_show_channel false\n");
					goto err;
				}
			else
				{
					strcpy(projector_info.show_channel,uci_show_channel);
					
				}
			//投影仪延迟时间参数
			//开机延迟时间
			if(NULL==(uci_start_delay=uci_lookup_option_string(ctx,section,"start_delay")))
				{
					printf("get uci_start_delay false\n");
					goto err;
				}
			else
				{
					projector_info.start_delay=atoi(uci_start_delay);
				}
			//关机延迟时间
			if(NULL==(uci_shutdown_delay=uci_lookup_option_string(ctx,section,"shutdown_delay")))
				{
					printf("get uci_shutdown_delay false\n");
					goto err;
				}
			else
				{
					projector_info.shutdown_delay=atoi(uci_shutdown_delay);
				}
			//间隔时间
			if(NULL==(uci_interval=uci_lookup_option_string(ctx,section,"interval")))
				{
					printf("get uci_interval false\n");
					goto err;
				}
			else
				{
					projector_info.interval=atoi(uci_interval);
				}
		}
	//声音初始化
	if(NULL==(section=uci_lookup_section(ctx,pkg,"volume")))
		{
			printf("get volume false\n");
			goto err;
		}
	else
		{
			//高音
			if(NULL==(uci_high=uci_lookup_option_string(ctx,section,"high")))
				{
					printf("get uci_high false\n");
					goto err;
				}
			else
				{
					volume_3105.high=atoi(uci_high);
				}
			//低音
			if(NULL==(uci_low=uci_lookup_option_string(ctx,section,"low")))
				{
					printf("get uci_low false\n");
					goto err;
				}
			else
				{
					volume_3105.low=atoi(uci_low);
				}
			//音量
			if(NULL==(uci_volume=uci_lookup_option_string(ctx,section,"volume")))
				{
					printf("get uci_volume false\n");
					goto err;
				}
			else
				{
					volume_3105.volume=atoi(uci_volume);
				}
		}
	if(NULL==(section=uci_lookup_section(ctx,pkg,"rfid")))
		{
			printf("get volume false\n");
			goto err;
		}
	else
		{
			if(NULL==(uci_section1=uci_lookup_option_string(ctx,section,"section")))
				{
					printf("get uci_section false\n");
					goto err;
				}
			else
				{
					rfid_info.section=atoi(uci_section1);
					RFIDSection=rfid_info.section;
				}
			if(NULL==(uci_opentime=uci_lookup_option_string(ctx,section,"opentime")))
				{
					printf("get uci_opentime false\n");
					goto err;
				}
			else
				{
					rfid_info.opentime=atoi(uci_opentime)*60;
				}
			if(NULL==(uci_workmode=uci_lookup_option_string(ctx,section,"workmode")))
				{
					printf("get uci_workmode false\n");
					goto err;
				}
			else
				{
					rfid_info.workmode=atoi(uci_workmode);
				}
		}
good:
	uci_unload(ctx,pkg);
	uci_free_context(ctx);
	ctx=NULL;
	return 1;
loaderr:
	uci_free_context(ctx);
	ctx=NULL;
	return -1;
err:
	uci_unload(ctx,pkg);
	uci_free_context(ctx);
	ctx=NULL;
	return -1;	
}

/****************************************************************************
获取mac地址



******************************************************************************/

static int Get_DTS2B_Mac(char * mac, int len_limit)
{

	
	int fd;
	int re;
	unsigned char buf[6];
	unsigned char buff[12];
		
	unsigned int a;
	unsigned int b;
		
	memset(buf,0,6);
	memset(buff,0,12);
	printf("memset success\n");
	fd=open("/dev/mtd2",O_RDONLY);
	if(fd<0)
	{
		printf("cannot open mtd2\n");
		return -1;
	}
	lseek(fd,4,SEEK_SET);
	printf("lseek success\n");
	read(fd,buf,6);
	sprintf(buff,"%02x%02x%02x%02x%02x%02x",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
	printf("buff = %s\n",buff);
	strcpy(mac,buff);
	printf("mac =%s\n",mac);
	close(fd);
	return 1;




	
	/*struct ifreq ifreq;
    int sock;
    if ((sock = socket (AF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror ("socket");
        return -1;
    }
    strcpy (ifreq.ifr_name, "eth0");    //Currently, only get eth0

    if (ioctl (sock, SIOCGIFHWADDR, &ifreq) < 0)
    {
        perror ("ioctl");
        return -1;
    }
    
   // return snprintf (mac, len_limit, "%02x:%02x:%02x:%02x:%02x:%02x", (unsigned char) ifreq.ifr_hwaddr.sa_data[0], (unsigned char) ifreq.ifr_hwaddr.sa_data[1], (unsigned char) ifreq.ifr_hwaddr.sa_data[2], (unsigned char) ifreq.ifr_hwaddr.sa_data[3], (unsigned char) ifreq.ifr_hwaddr.sa_data[4], (unsigned char) ifreq.ifr_hwaddr.sa_data[5]);
	return snprintf (mac, len_limit, "%02x%02x%02x%02x%02x%02x", (unsigned char) ifreq.ifr_hwaddr.sa_data[0], (unsigned char) ifreq.ifr_hwaddr.sa_data[1], (unsigned char) ifreq.ifr_hwaddr.sa_data[2], (unsigned char) ifreq.ifr_hwaddr.sa_data[3], (unsigned char) ifreq.ifr_hwaddr.sa_data[4], (unsigned char) ifreq.ifr_hwaddr.sa_data[5]);*/
}
/****************************************************************************
最初的初始化，在brxy文件初始化之前调用



******************************************************************************/
static int init_first(void)
{
	char *mac;
	unsigned char rfid_card_id[4]={0x00,0x00,0x00,0x00};
	unsigned char rfid_card_projector_id[16]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a};
	unsigned char key_a[6]={0x06,0x0f,0x0c,0x02,0x03,0x09};
	unsigned char control_data[16]={0x06,0x0f,0x0c,0x02,0x03,0x09,0x00,0x00,0x00,0x00,0x06,0x0f,0x0c,0x02,0x03,0x09};

	int i=0;
	for(i=0;i<10;i++)
		{
			rfid_info.projector[i]=0xff;
		}
	rfid_info.section=1;
	rfid_info.opentime=40*60;
	rfid_info.workmode=0;
	memmove(rfid_info.rfid_card_id,rfid_card_id,4);
	memmove(rfid_info.rfid_card_projector_id,rfid_card_projector_id,10);
	memmove(rfid_info.key_a,key_a,6);
	memmove(rfid_info.control_data,control_data,10);
	rfid_info.sector=1;
	rfid_info.number_local=0;
	rfid_info.number_net=0;

	power_control_status.microphone_broadcast=0;
	power_control_status.schedule_broadcast=0;
	power_control_status.system_status=0;

	
	nopoll_info.request_rfidinfo=0;
	nopoll_info.nopoll_isok_flag=0;
	nopoll_info.connect_report_flag=0;
	nopoll_info.nopoll_create_first=0;
	nopoll_info.count_of_failure=0;
	nopoll_info.nopoll_create_ctx_first=0;

	memset(base_info.ip,0,36);
	memset(base_info.name,0,128);
	memset(base_info.id,0,36);
	memset(base_info.qqlicense,0,1024);
	memset(base_info.sn,0,17);
	memset(base_info.sn_mcu,0,40);
	memset(base_info.mediaa,0,36);
	memset(base_info.mediab,0,36);
	base_info.poe=1;
	base_info.version="k4-0.0.0";
	base_info.openlimit=0;
	base_info.pc_automatic=1;
	base_info.qq_init_flag=0;
	base_info.qq_lisence_get_from_mcu_is_ok=0;
	base_info.qq_init_success=0;
	base_info.have_new_lisence=0;
	
	device_status.k4=0;
	device_status.pc=0;
	device_status.projector=0;
	device_status.ep=1;
	device_status.stand_lock=0;
	

	run_info.temperature=0;
	run_info.illumination=0;
	run_info.humidity=0;


	projector_info.baudrate=115200;
	projector_info.databyte=8;
	projector_info.check='N';
	projector_info.stop=1;
	memset(projector_info.start_code,0,512);
	memset(projector_info.shutdown_code,0,512);
	memset(projector_info.show_channel,0,512);
	projector_info.start_delay=20;
	projector_info.shutdown_delay=20;
	projector_info.interval=20;
	projector_info.start_wait=0;
	projector_info.shutdown_wait=0;
	projector_info.interval_wait=0;
	projector_info.send_return_status=0;
	projector_info.get_qq_license_busy=0;
	projector_info.get_qq_sn_return_flag=0;
	projector_info.get_qq_sn_busy=0;
	

	volume_3105.high=8;
	volume_3105.low=0;
	volume_3105.volume=5;
	volume_3105.mute=0;
	volume_3105.volume_default=5;

	mac=base_info.id;
	if(Get_DTS2B_Mac(mac,12)>0)
		{
			printf("mac = %s\n",mac);
			
		}
	else
		{
			printf("get mac false\n");
			return -1;
		}
	strcpy(base_info.sn,base_info.id);
	strcat(base_info.sn,"0000");
	return 1;
	
	
}



/****************************************************************************
rfid读取线程函数



******************************************************************************/

void *rfid_read_thread(void * ptr)
{
	int ret=0;
	int ret_decision;
	unsigned char an[10];
	while(1)
		{	
			ret=0;
			memset(an,0,10);
			memset(rfid_info.rfid_card_id,0,4);
			memset(rfid_info.rfid_card_projector_id,0,10);
			if((rfid_info.rfid_flag==0)&&(device_status.k4==0)&&(device_status.machine_flag==0))
				{
					//rfid_info.rfid_flag=1;
					pthread_mutex_lock(&i2c_mutex);
					ret=rfid_read();
					pthread_mutex_unlock(&i2c_mutex);
					if(ret==1)
						{
							memmove(an,rfid_info.rfid_card_id,4);
							pthread_mutex_lock(&rfid_card_uci_mutex);//上锁 
							ret_decision=rfid_decision(an);
							pthread_mutex_unlock(&rfid_card_uci_mutex);//上锁 
							if(ret_decision==2)
								{
									
									open_machine();
									rfid_info.flag=0;
								}
							else
								{
									if((rfid_info.rfid_flag==0)&&(device_status.k4==0)&&(device_status.machine_flag==0))
										{
											pthread_mutex_lock(&i2c_mutex);
											ret=RFID_user_read();
											pthread_mutex_unlock(&i2c_mutex);
											if(ret==1)
												{
													if((rfid_info.rfid_card_projector_id[0]==rfid_info.rfid_card_projector_id_read[0])&&(rfid_info.rfid_card_projector_id[1]==rfid_info.rfid_card_projector_id_read[1])&&(rfid_info.rfid_card_projector_id[2]==rfid_info.rfid_card_projector_id_read[2])&&(rfid_info.rfid_card_projector_id[2]==rfid_info.rfid_card_projector_id_read[3])&&(rfid_info.rfid_card_projector_id[4]==rfid_info.rfid_card_projector_id_read[4])&&(rfid_info.rfid_card_projector_id[5]==rfid_info.rfid_card_projector_id_read[5])&&(rfid_info.rfid_card_projector_id[6]==rfid_info.rfid_card_projector_id_read[6])&&(rfid_info.rfid_card_projector_id[7]==rfid_info.rfid_card_projector_id_read[7])&&(rfid_info.rfid_card_projector_id[8]==rfid_info.rfid_card_projector_id_read[8])&&(rfid_info.rfid_card_projector_id[9]==rfid_info.rfid_card_projector_id_read[9]))
														{
															open_machine();
														}
												}
										}
								}
						}
				}
			else if((rfid_info.rfid_flag==0)&&(device_status.k4==1)&&(device_status.machine_flag==0))
				{
					pthread_mutex_lock(&i2c_mutex);
					ret=rfid_read();
					pthread_mutex_unlock(&i2c_mutex);
					if(ret==1)
						{
							memmove(an,rfid_info.rfid_card_id,4);
							pthread_mutex_lock(&rfid_card_uci_mutex);//上锁 
							ret_decision=rfid_decision(an);
							pthread_mutex_unlock(&rfid_card_uci_mutex);//上锁 
							if(ret_decision==2)
								{
									rfid_info.flag=0;
								}
						}
				}
			sleep(1);
		}
}






int init_rfid_uci(void)
{
	struct uci_package *pkg=NULL;
	struct uci_section *section,*section_net;
	struct uci_cntext *ctx;
	struct uci_option *o,*o_net;

	struct k4_rfid_card_id *a;
	struct k4_rfid_card_id *b;
	char *rout;
	char *id;
	char id1[156];
	unsigned char id2[4];
	char st[3]={0};
	int i;
	rout=rfid_info.projector;
	if(NULL==(ctx=uci_alloc_context()))
		{
			printf("get ctx for uci false\n");
			return -1;
		}
	if (UCI_OK!= uci_load(ctx, "/usr/share/brxydata/card", &pkg))
		{
			printf("init uci load false\n");
      	  goto loaderr;
		}
	if(NULL==(section=uci_lookup_section(ctx,pkg,"card")))
		{
			printf("get card section false\n");
			goto err;
		}
	if(NULL==(o=uci_lookup_option(ctx, section, "id")))
		{
			printf("get id option false\n");
			goto err;
		}
	if(NULL==(section_net=uci_lookup_section(ctx,pkg,"net_card")))
		{
			printf("get net_card section false\n");
			goto err;
		}
	if(NULL==(o_net=uci_lookup_option(ctx, section_net, "id")))
		{
			printf("get net id option false\n");
			goto err;
		}
	printf("get option finish\n");
	//sleep(1);
	if ((NULL != o) && (UCI_TYPE_LIST == o->type)) //o存在 且 类型是 UCI_TYPE_LIST则可以继续.
	{
		struct uci_element *e;
		
		uci_foreach_element(&o->v.list, e)
		{

			id=e->name;
			if(strcmp(id,"00000000brxynotuse")!=0)
				{
			//printf("rfid id =%s\n",id);
			memset(id1,0,44);
			strcpy(id1,id);
			for(i=0;i<4;i++)
				{
					st[0]=id1[i*2];
					st[1]=id1[i*2+1];
					id2[i]=String2hex(st);
				}
			//a=(struct k4_rfid_card_id *)malloc(sizeof(struct k4_rfid_card_id));
			while((a=(struct k4_rfid_card_id *)malloc(sizeof(struct k4_rfid_card_id)))==NULL)
				{
					printf("malloc false\n");
					sleep(2);
				}
			a->last=1;
			a->del=0;
			memmove(a->id,id2,4);
			a->id[5]=0;
			memset(a->name,0,128);
			
			memmove(a->id1,id,8);
			a->id1[8]=0;
			strcpy(a->name,&id1[8]);
			
			if(rfid_info.number_local==0)
				{
					rfid_info.next_local=a;
					rfid_info.number_local++;
				}
			else
				{
					b=rfid_info.next_local;
					while(1)
						{
							if(b->last==1)
								{
									b->next=a;
									b->last=0;
									rfid_info.number_local++;
									break;
								}
							b=b->next;
						}
				}
				}
			
		}
	}	
	else
		{
			printf("get card local false\n");
			
		}
	printf("get card local finish\n");
	//sleep(1);
	if ((NULL != o_net) && (UCI_TYPE_LIST == o_net->type)) //o存在 且 类型是 UCI_TYPE_LIST则可以继续.
	{
		struct uci_element *e_net;
		uci_foreach_element(&o_net->v.list, e_net)
		{

			id=e_net->name;
			if(strcmp(id,"00000000brxynotuse")!=0)
				{
			//printf("rfid id =%s\n",id);
			memset(id1,0,156);
			strcpy(id1,id);
			for(i=0;i<5;i++)
				{
					st[0]=id1[i*2];
					st[1]=id1[i*2+1];
					id2[i]=String2hex(st);
				}
			//a=(struct k4_rfid_card_id *)malloc(sizeof(struct k4_rfid_card_id));
			while((a=(struct k4_rfid_card_id *)malloc(sizeof(struct k4_rfid_card_id)))==NULL)
				{
					printf("malloc false\n");
					sleep(2);
				}
			a->last=1;
			a->del=0;
			memmove(a->id,id2,4);
			a->id[5]=0;
			memset(a->name,0,128);
			a->id1[8]=0;
			memmove(a->id1,id,8);
			strcpy(a->name,&id1[8]);
			
			if(rfid_info.number_net==0)
				{
					rfid_info.next_net=a;
					rfid_info.number_net++;
				}
			else
				{
					b=rfid_info.next_net;
					while(1)
						{
							if(b->last==1)
								{
									b->next=a;
									b->last=0;
									rfid_info.number_net++;
									break;
								}
							b=b->next;
						}
				}
				}
			
		}
	}	
	printf("get net card finish\n");
	//sleep(1);
good:
	//printf("1\n");
	//sleep(1);
	//uci_unload(ctx,pkg);
	//printf("2\n");
	//sleep(1);
	uci_free_context(ctx);
	//printf("3\n");
	//sleep(1);
	ctx=NULL;
	return 1;
loaderr:
	uci_free_context(ctx);
	ctx=NULL;
	return -1;
err:
	uci_unload(ctx,pkg);
	uci_free_context(ctx);
	ctx=NULL;
	return -1;	
}

/****************************************************************************
关系统倒计时线程



******************************************************************************/
void *close_machine_thread(void)
{
	//int flag=0;
	//rfid_info.flag=0;
	while(1)
		{
			if(device_status.k4==0)
				{
					sleep(10);
					
				}
			else
				{
					if(rfid_info.flag==0)
						{
							rfid_info.flag=1;
							if(rfid_info.opentime==0)
								{
									rfid_info.use_mode=0;
								}
							else
								{
									rfid_info.use_mode=1;
								}
							rfid_info.use_time=rfid_info.opentime/60;
						}
					if(rfid_info.flag==1)
						{
							if(rfid_info.use_mode==1)
								{
									rfid_info.use_time--;
									if(rfid_info.use_time==5)
										{
											printf("only 5 minites to close machine\n");
										}
									if(rfid_info.use_time==0)
										{
											close_machine();
										}
									sleep(60);
								}
							else
								{
									sleep(60);
								}
						}
					else
						{
							sleep(10);
						}
				}
		}
}

/****************************************************************************
开机前闪烁


******************************************************************************/
int init_led_struct(void)
{
	int ret=0;
	if((ret=sem_init(&led_glint.led_standy_sem_t,0,1))!=0)
		{
			perror("init semphare key_button_info.wake_key_deal fail");
			return -1;
		}
	return 1;
}

void *machine_standy_thread(void)
{
	unsigned char to=0;
	int k;
	while(1)
		{
			sem_wait(&led_glint.led_standy_sem_t);
			if(led_glint.led_standy_flag==1)
				{
					k=0;
					while(1)
						{
							
							if(k==0)
								{
									k=1;
									pthread_mutex_lock(&i2c_mutex);
									//tca_set_bit(2);
									if(i2c_read(i2c_address_tca,tca_info.port0_out,&to)<0)
										{
											printf("can not read tca\n");
											break;
										}
									to|=(1<<tca_info.mation_status);
									to|=(1<<tca_info.ep_status);
									to|=(1<<tca_info.projector_status);
									//tca_set_bit(2);
									//tca_set_bit(3);
									//tca_set_bit(7);
									if(i2c_write(i2c_address_tca,tca_info.port0_out,to)<0)
										{
											printf("can not write to tca\n");
											break;
										}
									pthread_mutex_unlock(&i2c_mutex);
								}
							else
								{
									k=0;
									pthread_mutex_lock(&i2c_mutex);
									//clean_tca_bit(2);
									if(i2c_read(i2c_address_tca,tca_info.port0_out,&to)<0)
										{
											printf("can not read tca\n");
											break;
										}
									to&=(~(1<<tca_info.mation_status));
									to&=(~(1<<tca_info.ep_status));
									to&=(~(1<<tca_info.projector_status));
									
									if(i2c_write(i2c_address_tca,tca_info.port0_out,to)<0)
										{
											printf("can not write to tca\n");
											break;
										}
									pthread_mutex_unlock(&i2c_mutex);
								}
							if(led_glint.led_standy_flag==2)
								{
									led_glint.led_standy_flag=3;
									break;
								}
							sleep(1);
						}
				}
			else if(led_glint.led_standy_flag==0)
				{
					k=0;
					while(1)
						{
							
							if(k==0)
								{
									k=1;
									pthread_mutex_lock(&i2c_mutex);
									tca_set_bit(2);
									usleep(10000);
									/*if(i2c_read(i2c_address_tca,tca_info.port0_out,&to)<0)
										{
											printf("can not read tca\n");
											break;
										}
									to|=(1<<tca_info.mation_status);
									to|=(1<<tca_info.ep_status);
									to|=(1<<tca_info.projector_status);
									//tca_set_bit(2);
									//tca_set_bit(3);
									//tca_set_bit(7);
									if(i2c_write(i2c_address_tca,tca_info.port0_out,to)<0)
										{
											printf("can not write to tca\n");
											break;
										}*/
									pthread_mutex_unlock(&i2c_mutex);
								}
							else
								{
									k=0;
									pthread_mutex_lock(&i2c_mutex);
									clean_tca_bit(2);
									usleep(10000);
									/*if(i2c_read(i2c_address_tca,tca_info.port0_out,&to)<0)
										{
											printf("can not read tca\n");
											break;
										}
									to&=(~(1<<tca_info.mation_status));
									to&=(~(1<<tca_info.ep_status));
									to&=(~(1<<tca_info.projector_status));
									
									if(i2c_write(i2c_address_tca,tca_info.port0_out,to)<0)
										{
											printf("can not write to tca\n");
											break;
										}*/
									pthread_mutex_unlock(&i2c_mutex);
								}
							if(led_glint.led_standy_flag==2)
								{
									led_glint.led_standy_flag=3;
									break;
								}
							sleep(1);
						}
				}
		}
}


/****************************************************************************
生成qqlicense



******************************************************************************/
/*int qq_sn_to_license(void)
{
	int nRet = -1;
	unsigned int nLicencelen   = 0;
	char szLicenceBuffer[512] = {0};
	char sn[36]={0};
	char *szPrivateKeyPath="/usr/share/brxydata/ec_key.pem";
	char *szBufferData;
	int nBufferData;
	int nLicenceOutlen      = 0;
	int nLicenceBase16len   = 1024;
	if(strcmp(base_info.qqlicense,"000")==0)
		{

			memmove(sn,base_info.sn,17);
			printf("base_info = %s\n",base_info.sn);
			nBufferData=strlen(sn);
			szBufferData=sn;
			nRet = ECDSASignToBuffer(szPrivateKeyPath,szBufferData,nBufferData,szLicenceBuffer,&nLicencelen);
			if(nRet==1)
				{
					base16Encode((unsigned char *)szLicenceBuffer,nLicencelen,(unsigned char *)base_info.qqlicense,nLicenceBase16len,&nLicenceOutlen);
					return 1;
				}
			else
				{
					return -1;
				}
			FILE *file = fopen("/usr/share/brxydata/licence.sign.file.txt","wb");
			fwrite(base_info.qqlicense,nLicenceOutlen,1,file);
			fclose(file);
			
		}
	else
		{
			return 1;
		}


	return -1;
}*/

/****************************************************************************
初始化qq设备



******************************************************************************/
int qq_init_decision(void)
{
	int flag=0;
	if(strcmp(base_info.qqlicense,"000")==0)
		{
			base_info.qq_init_flag=0;
		}
	else
		{
			base_info.qq_init_flag=1;
		}
	while(1)
		{
			if(base_info.qq_init_flag==0)
				{
					sem_wait(&base_info.qq_init_sem_t);
				}
			if(base_info.qq_init_flag==1)
				{
					initDevice();
					base_info.qq_init_flag=2;
					base_info.qq_lisence_get_from_mcu_is_ok=1;
					sem_wait(&base_info.qq_init_sem_t);
					tx_exit_device();
				}
			printf("sem_wait to base_info.qq_init_sem_t exit\n");
			sleep(1);
		}
}


/****************************************************************************
nopoll 用到的函数



******************************************************************************/



noPollCtx * create_ctx (void) {
	
	/* create a context */
	noPollCtx * ctx = nopoll_ctx_new ();
	nopoll_log_enable (ctx,debug);
	nopoll_log_color_enable (ctx,debug);
	return ctx;
}






void analyze_of_control_device(char *msg)
{

	char* json;
	 printf("in analyze_of_control_device \n");
	 //log_func(4, "business", 23, json);
	// Document document;
	 //document.Parse(json);
	 //TODO 控制设备 包含设备类型 操作码  操作值
	char *devicetype;
	char *operation;
	char *mac;
	char *value;
	char *return_success="{\"result\":\"true\", \"msg\":\"success\"}";
	char *return_failed="{\"result\":\"false\",\"msg\":\"failed\"}";
	int ret = 0;
	json=msg;

	cJSON *alldata=cJSON_Parse(json);
	devicetype=cJSON_GetObjectItem(alldata,"deviceType")->valuestring;

	if(strcmp(devicetype,"DTS_HOST")==0)
		{
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(strcmp(operation,"START")==0)
				{
					printf("analyze_of_control_device   starting into lock\n");
					pthread_mutex_lock(&projector_info.mutex);
					printf("analyze_of_control_device    in lock\n");
					projector_info.send_return_status=0;
					mcu_open_system();
					usleep(200000);
					//read_status_from_mcu();
				//	usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					printf("analyze_of_control_device   out of lock\n");
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_shutdown_system();
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					
						
				}
		}
	else if(strcmp(devicetype,"DTS_PC")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			
			if(strcmp(operation,"START")==0)
				{
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_open_pc();
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_shutdown_pc();
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					
						
				}
		}
	else if(strcmp(devicetype,"DTS_DISPLAYER")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			
			if(strcmp(operation,"START")==0)
				{
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_open_projector();
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							//
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_shutdown_projector();
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					
				}
		}
	else if(strcmp(devicetype,"DTS_CHANNEL")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(1==0)
				{
					printf("dts_host is shutdown,so can not control dts_channel\n");
					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_host is shutdown,so can not control dts_channel\"}" , datapoint.seq, ret, 0, 0);
					
				}
			else //if(strcmp(operation,"START")==0)
				{
					
					//ep_change();
					if(strcmp(operation,"START")==0)
						{
							pthread_mutex_lock(&projector_info.mutex);
							projector_info.send_return_status=0;
							mcu_change_channel(2);
							usleep(200000);
							if(projector_info.send_return_status!=1)
								{
							
									//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
								}
							else
								{
							
									//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
								}
							pthread_mutex_unlock(&projector_info.mutex);
							
						}
					if(strcmp(operation,"SHUTDOWN")==0)
						{
							pthread_mutex_lock(&projector_info.mutex);
							projector_info.send_return_status=0;
							mcu_change_channel(1);
							usleep(200000);
							if(projector_info.send_return_status!=1)
								{
							
									//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
								}
							else
								{
							
									//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
								}
							pthread_mutex_unlock(&projector_info.mutex);
							
						}
					
					

					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
				}
				
		}
	else if(strcmp(devicetype,"DTS_BOOTH")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(1==0)
				{
					printf("dts_host is shutdown,so can not control dts_booth\n");
					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_host is shutdown,so can not control dts_booth\"}" , datapoint.seq, ret, 0, 0);
					
				}
			else if(strcmp(operation,"START")==0)
				{
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_open_stand();
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_shutdown_stand();
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					
				}
		}
	else if(strcmp(devicetype,"DTS_SCREEN")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(1==0)
				{
					printf("dts_screen is shutdown,so can not control dts_screen dts_booth\n");
					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_screen is shutdown,so can not control dts_screen \"}" , datapoint.seq, ret, 0, 0);
					
				}
			else if(strcmp(operation,"START")==0)
				{
					
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_screen_control(1);
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					pthread_mutex_lock(&projector_info.mutex);
					projector_info.send_return_status=0;
					mcu_screen_control(0);
					usleep(200000);
					if(projector_info.send_return_status!=1)
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
						}
					else
						{
							
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
						}
					pthread_mutex_unlock(&projector_info.mutex);
					
				}
		}
	else if(strcmp(devicetype,"DTS_VOLUME")==0)
		{
			int vo=0;
			int vo1=0;
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			value=cJSON_GetObjectItem(alldata,"operationValue")->valuestring;
			vo=atoi(value);
			vo1=vo/2;
			if((vo1==0)&&(vo==0))
				{
					volume_3105.volume=0;
				}
			else if((vo1>0)&&(vo1<9))
				{
					volume_3105.volume=vo1;
				}
			pthread_mutex_lock(&projector_info.mutex);
			projector_info.send_return_status=0;
			mcu_config_volume(volume_3105.volume);
			usleep(200000);
			if(projector_info.send_return_status!=1)
				{
							
					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							
				}
			else
				{
							
					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							
				}
			pthread_mutex_unlock(&projector_info.mutex);
			
		}
	else if(strcmp(devicetype,"DTS_SWITCH")==0)
		{
			int valume;
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			mac=cJSON_GetObjectItem(alldata,"deviceID")->valuestring;
			value=cJSON_GetObjectItem(alldata,"operationValue")->valuestring;
			valume=atoi(value);
			if(strcmp(operation,"START")==0)
				{
					turn_on_off_wifi_switch1(1,mac,valume);
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					turn_on_off_wifi_switch1(0,mac,valume);
				}
			
			//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
			
		}
	else
		{
			//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"invalidity control order\"}" , datapoint.seq, ret, 0, 0);
		}
	cJSON_Delete(alldata);
	return;
}


void analyze_of_control_device_old(char *msg)
{

	char* json;
	 printf("on_receive_controll_dts, json:%s\n", json);
	 //log_func(4, "business", 23, json);
	// Document document;
	 //document.Parse(json);
	 //TODO 控制设备 包含设备类型 操作码  操作值
	char *devicetype;
	char *operation;
	char *mac;
	char *value;
	char *return_success="{\"result\":\"true\", \"msg\":\"success\"}";
	char *return_failed="{\"result\":\"false\",\"msg\":\"failed\"}";
	int ret = 0;
	json=msg;

	cJSON *alldata=cJSON_Parse(json);
	devicetype=cJSON_GetObjectItem(alldata,"deviceType")->valuestring;

	if(strcmp(devicetype,"DTS_HOST")==0)
		{
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(strcmp(operation,"START")==0)
				{
					if(device_status.k4==1)
						{
							printf("dts aready started\n");
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready started\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
					else
						{
							open_machine();
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					if(device_status.k4==0)
						{
							
							printf("dts aready shutdown\n");
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts aready shutdown\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
					else
						{
							close_machine();
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
						
				}
		}
	else if(strcmp(devicetype,"DTS_PC")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(device_status.k4==0)
				{
					printf("dts_host is shutdown,so can not control pc\n");
					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_host is shutdown,so can not control pc\"}" , datapoint.seq, ret, 0, 0);
					//return;
				}
			else if(strcmp(operation,"START")==0)
				{
					if(device_status.pc==1)
						{
							printf("pc aready started\n");
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"pc aready dtarted\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
					else
						{

							pc_open();

							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					if(device_status.pc==0)
						{
							
							printf("pc aready shutdown\n");
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"pc aready shutdown\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
					else
						{

							pc_close();

							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
						
				}
		}
	else if(strcmp(devicetype,"DTS_DISPLAYER")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(device_status.k4==0)
				{
					printf("dts_host is shutdown,so can not control dts_displayer\n");
					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_host is shutdown,so can not control dts_displayer\"}" , datapoint.seq, ret, 0, 0);
					//return;
				}
			else if(strcmp(operation,"START")==0)
				{
					if(device_status.projector==1)
						{
							printf("dts_displayer aready started\n");
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_displayer aready dtarted\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
					else
						{

							projector_open();

							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					if(device_status.projector==0)
						{
							
							printf("dts_displayer aready shutdown\n");
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_displayer aready shutdown\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
					else
						{

							projector_close();

							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
						
				}
		}
	else if(strcmp(devicetype,"DTS_CHANNEL")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(device_status.k4==0)
				{
					printf("dts_host is shutdown,so can not control dts_channel\n");
					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_host is shutdown,so can not control dts_channel\"}" , datapoint.seq, ret, 0, 0);
					//return;
				}
			else //if(strcmp(operation,"START")==0)
				{
					if(strcmp(operation,"START")==0)
						{
							if(device_status.ep==1)
								{
									printf("dts_stand aready started\n");
									//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_booth aready dtarted\"}" , datapoint.seq, ret, 0, 0);
									return;
								}
							else
								{
									ep_switch_to_out();
									//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
									return;
								}
						}
					if(strcmp(operation,"SHUTDOWN")==0)
						{
							if(device_status.ep==0)
								{
									printf("dts_stand aready shutdown\n");
									//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_booth aready dtarted\"}" , datapoint.seq, ret, 0, 0);
									return;
								}
							else
								{
									ep_switch_to_in();
									//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
									return;
								}
						}
					//ep_change();
					

					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
				}
				
		}
	else if(strcmp(devicetype,"DTS_BOOTH")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(device_status.k4==0)
				{
					printf("dts_host is shutdown,so can not control dts_booth\n");
					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_host is shutdown,so can not control dts_booth\"}" , datapoint.seq, ret, 0, 0);
					//return;
				}
			else if(strcmp(operation,"START")==0)
				{
					if(device_status.stand_lock==1)
						{
							
							printf("dts_booth aready started\n");
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_booth aready dtarted\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
					else
						{

							
							//stand_close();
							stand_open();

							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					if(device_status.stand_lock==0)
						{
							
							printf("dts_booth aready shutdown\n");
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_booth aready shutdown\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
					else
						{

							//stand_open();
							stand_close();
							//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							//return;
						}
						
				}
		}
	else if(strcmp(devicetype,"DTS_SCREEN")==0)
		{
		
			
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			if(device_status.k4==0)
				{
					printf("dts_screen is shutdown,so can not control dts_screen dts_booth\n");
					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_screen is shutdown,so can not control dts_screen \"}" , datapoint.seq, ret, 0, 0);
					//return;
				}
			else if(strcmp(operation,"START")==0)
				{
					/*if(info.dts_screen==1)
						{
							printf("dts_screen aready started\n");
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_screen aready dtarted\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
					else
						{

							info.dts_screen=1;

							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							return;
						}*/
					turn_on_off_screen(1);
					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
						
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					/*if(info.dts_screen==0)
						{
							
							printf("dts_screen aready shutdown\n");
							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"dts_screen aready shutdown\"}" , datapoint.seq, ret, 0, 0);
							return;
						}
					else
						{

							info.dts_screen=0;

							tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
							return;
						}*/
					turn_on_off_screen(0);
					//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
				}
		}
	else if(strcmp(devicetype,"DTS_VOLUME")==0)
		{
			int vo=0;
			int vo1=0;
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			value=cJSON_GetObjectItem(alldata,"operationValue")->valuestring;
			vo=atoi(value);
			vo1=vo/2;
			if((vo1==0)&&(vo==0))
				{
					volume_3105.volume=0;
				}
			else if((vo1>0)&&(vo1<9))
				{
					volume_3105.volume=vo1;
				}
			pthread_mutex_lock(&i2c_mutex);
			i2c_write(i2c_address_wm,0x1c,wm8776_volume_table[volume_3105.volume]);
			i2c_write(i2c_address_wm,0x15,0x8f);//打开通道 不静音
			pthread_mutex_unlock(&i2c_mutex);

			
			printf("adjust valume to %d\n",vo);
			//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
			//return;
		}
	else if(strcmp(devicetype,"DTS_SWITCH")==0)
		{
			int valume;
			operation=cJSON_GetObjectItem(alldata,"operationCode")->valuestring;
			mac=cJSON_GetObjectItem(alldata,"deviceID")->valuestring;
			value=cJSON_GetObjectItem(alldata,"operationValue")->valuestring;
			valume=atoi(value);
			if(strcmp(operation,"START")==0)
				{
					turn_on_off_wifi_switch1(1,mac,valume);
				}
			else if(strcmp(operation,"SHUTDOWN")==0)
				{
					turn_on_off_wifi_switch1(0,mac,valume);
				}
			
			//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"true\", \"msg\":\"success\"}" , datapoint.seq, ret, 0, 0);
			//return;
		}
	else
		{
			//tx_ack_data_point(from_id, datapoint.id,"{\"result\":\"false\", \"msg\":\"invalidity control order\"}" , datapoint.seq, ret, 0, 0);
		}
	cJSON_Delete(alldata);
	return;
}


int save_rfid_info_to_uci(char *rfidid,char *rfidname)
{
	char *buf1,*buf2;

	char *cardid,*cardname;
	struct k4_rfid_card_id *a;
	struct k4_rfid_card_id *b;

	int i;
	int num=1;
	char change[9]={0};
	char change1[3]={0};
	char change2[4]={0};
	char save_uci[156]={0};

	cardid=rfidid;
	cardname=rfidname;


	if(num==1)
		{
			strcpy(change,cardid);
			for(i=0;i<4;i++)
				{
					change1[0]=change[i*2];
					change1[1]=change[i*2+1];
					change2[i]=String2hex(change1);
				}

			
			//a=(struct k4_rfid_card_id *)malloc(sizeof(struct k4_rfid_card_id));
			while((a=(struct k4_rfid_card_id *)malloc(sizeof(struct k4_rfid_card_id)))==NULL)
				{
					printf("malloc false\n");
					sleep(2);
				}
			a->del=0;
			a->last=1;
			memset(a->id,0,4);
			memset(a->name,0,128);
			memmove(a->id,change2,4);
			strcpy(a->name,cardname);

			a->last=1;
			a->del=0;
			memmove(a->id,change2,4);
			a->id[5]=0;
			memset(a->name,0,128);
			memmove(a->id1,change,8);
			a->id1[8]=0;
			strcpy(a->name,cardname);
			if(rfid_info.number_net==0)
				{
					rfid_info.next_net=a;
					rfid_info.number_net++;
				}
			else
				{
					b=rfid_info.next_net;
					while(1)
						{
							if((b->del==0)&&(b->id[0]==a->id[0])&&(b->id[1]==a->id[1])&&(b->id[2]==a->id[2])&&(b->id[3]==a->id[3]))
								{
									free(a);
									memset(b->name,0,128);
									strcpy(b->name,a->name);
									b->del=0;
									return -1;
								}
							else if((b->del==1)&&(b->id[0]==a->id[0])&&(b->id[1]==a->id[1])&&(b->id[2]==a->id[2])&&(b->id[3]==a->id[3]))
								{
									
									memset(b->name,0,128);
									strcpy(b->name,a->name);
									b->del=0;
									memset(save_uci,0,156);
									strcpy(save_uci,a->id1);
									strcat(save_uci,cardname);
									printf("save rfid from server = %s\n",save_uci);
									struct uci_context *context;
									context=uci_alloc_context();
									struct uci_ptr ptr={
											.package="/usr/share/brxydata/card",
											.section="net_card",
											.option="id",
											.value=save_uci,
											};
									uci_add_list(context,&ptr);
									uci_commit(context,&ptr.p,false);
									uci_free_context(context);
									free(a);
									return -1;
								}
							if(b->last==1)
								{
									memset(save_uci,0,156);
									strcpy(save_uci,a->id1);
									strcat(save_uci,cardname);
									printf("save rfid from server = %s\n",save_uci);
									struct uci_context *context;
									context=uci_alloc_context();
									struct uci_ptr ptr={
											.package="/usr/share/brxydata/card",
											.section="net_card",
											.option="id",
											.value=save_uci,
											};
									uci_add_list(context,&ptr);
									uci_commit(context,&ptr.p,false);
									uci_free_context(context);
									b->next=a;
									b->last=0;
									rfid_info.number_net++;
									break;
								}
							b=b->next;
						}
				}
			
		}
	return 1;
}


int delete_rfid_info_to_uci(char *rfidid,char *rfidname)
{
	char *buf1,*buf2;

	char *cardid,*cardname;
	struct k4_rfid_card_id *a;
	struct k4_rfid_card_id *b;

	int i;
	int num=1;
	char change[9]={0};
	char change1[3]={0};
	char change2[4]={0};
	char save_uci[156]={0};

	cardid=rfidid;
	cardname=rfidname;
	//printf("del cardname = %s\n",cardname);
	strcpy(change,cardid);
	for(i=0;i<4;i++)
		{
			change1[0]=change[i*2];
			change1[1]=change[i*2+1];
			change2[i]=String2hex(change1);
		}
	if(rfid_info.number_net==0)
		{
			return -1;
		}
	else
		{
			b=rfid_info.next_net;
			while(1)
				{
					if((b->id[0]==change2[0])&&(b->id[1]==change2[1])&&(b->id[2]==change2[2])&&(b->id[3]==change2[3]))
						{
							if(b->del==0)
								{
									b->del=1;
									memset(save_uci,0,156);
									strcpy(save_uci,change);
									strcat(save_uci,cardname);
									//printf("del rfid = %s\n",save_uci);
									struct uci_context *context;
									context=uci_alloc_context();
									struct uci_ptr ptr={
											.package="/usr/share/brxydata/card",
											.section="net_card",
											.option="id",
											.value=save_uci,
											};
									uci_del_list(context,&ptr);
									uci_commit(context,&ptr.p,false);
									uci_free_context(context);
								}
							else
								{
									return -1;
								}
						}
					if(b->last==1)
						{
							return -1;
						}
					b=b->next;
				}
		}
	

	return 1;
	
}


int analyze_of_config_rfid(char *data)
{
	cJSON *alldata;
	cJSON *rfidinfo;
	cJSON *one_rfid;
	char config_rfid_return[256]={0};
	int number_of_rfid;

	char *rfidid;
	char *rfidname;
	char *rfidoperation;
	int j;

	alldata=cJSON_Parse(data);
	if(alldata!=NULL)
		{
			rfidinfo=cJSON_GetObjectItem(alldata,"array");
			if(rfidinfo!=NULL)
				{
					number_of_rfid=cJSON_GetArraySize(rfidinfo);
					for(j=0;j<number_of_rfid;j++)
						{
							one_rfid=cJSON_GetArrayItem(rfidinfo,j);
							if(one_rfid!=NULL)
								{
									if((rfidid=cJSON_GetObjectItem(one_rfid,"rfidid")->valuestring)!=NULL)
										{
											if((rfidname=cJSON_GetObjectItem(one_rfid,"rfidname")->valuestring)!=NULL)
												{
													printf("deal server rfid --rfidname = %s\n",rfidname);
													if((rfidoperation=cJSON_GetObjectItem(one_rfid,"operation")->valuestring)!=NULL)
														{
															if(strcmp(rfidoperation,"no")==0)
																{
																	save_rfid_info_to_uci(rfidid,rfidname);
																	
																	sprintf(config_rfid_return,"{\"function\":\"configrfidreturn\", \"status\":\"no\", \"rfidid\":\"%s\", \"mac\":\"%s\"}",rfidid,base_info.id);
																	nopoll_conn_send_text(nopoll_info.conn,config_rfid_return,strlen(config_rfid_return));
																	
																}
															else if(strcmp(rfidoperation,"del")==0)
																{
																	delete_rfid_info_to_uci(rfidid,rfidname);
																	
																	sprintf(config_rfid_return,"{\"function\":\"configrfidreturn\", \"status\":\"del\", \"rfidid\":\"%s\", \"mac\":\"%s\"}",rfidid,base_info.id);
																	nopoll_conn_send_text(nopoll_info.conn,config_rfid_return,strlen(config_rfid_return));
																		
																}
														}
													
													
												}
										}
								}
							
						}
					
				}

			
		}
	return 1;
	
	
}

int analyze_of_broadcast_play_mp3(char *data)
{
	cJSON *alldata;
	char *send_schedule_text="{\"function\":\"schedule\", \"operation\":\"schedule_play_return\"}";
	char *send_schedule_text_finish="{\"function\":\"schedule\", \"operation\":\"schedule_stop_return\"}";
	int send_schedule_text_length;
	int send_schedule_text_length_finish;
	char *send_broadcast_text="{\"function\":\"broadcastmp3\", \"operation\":\"broadcastmp3_play_return\"}";
	char *send_broadcast_text_finish="{\"function\":\"broadcastmp3\", \"operation\":\"broadcastmp3_stop_return\"}";
	int send_broadcast_text_length;
	int send_broadcast_text_length_finish;
	//a=(struct client_struct *)ptr;
	send_schedule_text_length=strlen(send_schedule_text);
	send_broadcast_text_length=strlen(send_broadcast_text);
	send_schedule_text_length_finish=strlen(send_schedule_text_finish);
	send_broadcast_text_length_finish=strlen(send_broadcast_text_finish);
	char *operation;
	alldata=cJSON_Parse(data);
	operation=cJSON_GetObjectItem(alldata,"operation")->valuestring;
	if(strcmp(operation,"broadcastmp3_play")==0)
		{
			//if(broadcast_play_mp3_info.is_working==0)
				//{
					broadcast_play_mp3_info.is_working=1;
					//sem_post(&broadcast_play_mp3_info.broadcast_play_mp3_wait_server);
					
				//}
			if(nopoll_conn_is_ready(nopoll_info.conn))
				{
									
					if(nopoll_conn_send_text(nopoll_info.conn,send_broadcast_text,send_broadcast_text_length)!=send_broadcast_text_length)
						{
							printf("broadcast play return false\n");
											
						}
										
				}
		}
	if(strcmp(operation,"broadcastmp3_stop")==0)
		{
			//if(broadcast_play_mp3_info.is_working==1)
				//{
					broadcast_play_mp3_info.is_working=0;
					//sem_post(&broadcast_play_mp3_info.broadcast_play_mp3_wait_server);
					
				//}
			if(nopoll_conn_is_ready(nopoll_info.conn))
				{
									
					if(nopoll_conn_send_text(nopoll_info.conn,send_broadcast_text_finish,send_broadcast_text_length_finish)!=send_broadcast_text_length_finish)
						{
							printf("broadcast play return false\n");
											
						}
										
				}
		}

	return 1;
}

int analyze_of_schedule_play(char *data)
{
	cJSON *alldata;
	char *send_schedule_text="{\"function\":\"schedule\", \"operation\":\"schedule_play_return\"}";
	char *send_schedule_text_finish="{\"function\":\"schedule\", \"operation\":\"schedule_stop_return\"}";
	int send_schedule_text_length;
	int send_schedule_text_length_finish;
	char *send_broadcast_text="{\"function\":\"broadcastmp3\", \"operation\":\"broadcastmp3_play_return\"}";
	char *send_broadcast_text_finish="{\"function\":\"broadcastmp3\", \"operation\":\"broadcastmp3_stop_return\"}";
	int send_broadcast_text_length;
	int send_broadcast_text_length_finish;
	//a=(struct client_struct *)ptr;
	send_schedule_text_length=strlen(send_schedule_text);
	send_broadcast_text_length=strlen(send_broadcast_text);
	send_schedule_text_length_finish=strlen(send_schedule_text_finish);
	send_broadcast_text_length_finish=strlen(send_broadcast_text_finish);
	char *operation;
	alldata=cJSON_Parse(data);
	operation=cJSON_GetObjectItem(alldata,"operation")->valuestring;
	if(strcmp(operation,"schedule_play")==0)
		{
			//if(schema_play_info.is_working==0)
				//{
					schema_play_info.is_working=1;
					//sem_post(&schema_play_info.schedule_play_wait_server);
					
				//}
			if(nopoll_conn_is_ready(nopoll_info.conn))
				{
									
					if(nopoll_conn_send_text(nopoll_info.conn,send_schedule_text,send_schedule_text_length)!=send_schedule_text_length)
						{
							printf("schedule play return false\n");
											
						}
										
				}
		}
	if(strcmp(operation,"schedule_stop")==0)
		{
			//if(schema_play_info.is_working==1)
				//{
					schema_play_info.is_working=0;
					//sem_post(&schema_play_info.schedule_play_wait_server);
					
				//}
			if(nopoll_conn_is_ready(nopoll_info.conn))
				{
									
					if(nopoll_conn_send_text(nopoll_info.conn,send_schedule_text_finish,send_schedule_text_length_finish)!=send_schedule_text_length_finish)
						{
							printf("schedule play return false\n");
											
						}
										
				}
		}

	return 1;
}


void mleaf_cb_recv_handler(noPollCtx *ctx, noPollConn *conn, noPollMsg *msg, noPollPtr user_data)
{
	char *buffer;
	char *functionstring;
	
	cJSON *dtsinfo;
	buffer=(char*)nopoll_msg_get_payload(msg);
	printf("get from server buffer = %s\n",buffer);
	if(strlen(buffer)>0)
		{
			dtsinfo=cJSON_Parse(buffer);
			if(dtsinfo==NULL)
				{
					nopoll_msg_unref (msg);
					return;
				}
			functionstring=cJSON_GetObjectItem(dtsinfo,"function")->valuestring;
			if(strcmp(functionstring,"controlldevice")==0)
				{
					analyze_of_control_device(buffer);
					
				}
			else if(strcmp(functionstring,"configrfid")==0)
				{
					analyze_of_config_rfid(buffer);
				}
			else if(strcmp(functionstring,"broadcastmp3")==0)
				{
					analyze_of_broadcast_play_mp3(buffer);
				}
			else if(strcmp(functionstring,"schedule")==0)
				{
					analyze_of_schedule_play(buffer);
				}
			else if(strcmp(functionstring,"reportreturn")==0)
				{
					nopoll_info.count_of_failure=0;
				}
			
			cJSON_Delete(dtsinfo);
		}
	nopoll_msg_unref (msg);
	return;
}





void nopoll_client_close_handle(noPollCtx * ctx, noPollConn * conn, noPollPtr user_data)
{
	
	//nopoll_conn_close(conn);
	schema_play_info.is_working=0;
	broadcast_play_mp3_info.is_working=0;
	nopoll_loop_stop(nopoll_info.ctx);
	return;
}

/*void * creat_nopoll_connection_old(void)
{
	char *port="8743";
	char ip[36]={0};
	char ipaddr[36]={0};
	int success=0;
	int i=0;
	char *sendtext="hello!";
	int length;
	ipaddr[0]='/';
	length=strlen(sendtext);
	strcpy(&ipaddr[1],base_info.id);
	while((nopoll_info.ctx=create_ctx())==NULL)
		{
			printf("create ctx false now into while\n");
			sleep(5);
		}
	
	
	while(1)
		{
			memset(ip,0,36);
			strcpy(ip,base_info.ip);
			nopoll_info.count_of_failure=0;
			if(nopoll_info.nopoll_create_first!=0)
				{
					nopoll_conn_close(nopoll_info.conn);//关闭conn
				}
			
			if((nopoll_info.opts=nopoll_conn_opts_new())==NULL)
				{
					printf("create opts false now into while\n");
					sleep(5);
				}
			else
				{
					if(nopoll_conn_opts_set_ssl_certs(nopoll_info.opts,"/usr/share/brxydata/dts2b-cert.pem","/usr/share/brxydata/dts2b-key-nopwd.pem",NULL,"/usr/share/brxydata/brxy-inner-ca-cert.pem")==nopoll_false)
						{
							printf("nopoll_conn_opts_set_ssl_certs false\n");
							sleep(5);
						}
					else
						{
							if((nopoll_info.conn=nopoll_conn_tls_new (nopoll_info.ctx, nopoll_info.opts, ip, port, NULL, ipaddr, NULL, NULL))==NULL)
								{
									printf("creat conn false now into while\n");
									sleep(5);
								}
							else
								{
									nopoll_info.nopoll_create_first=1;//表示创建了conn 需要关闭conn
									if(!nopoll_conn_is_ok(nopoll_info.conn))
										{
											printf("conn is not ok\n");
											sleep(5);
										}
									else
										{
											while(1)
												{
													i++;
													if(nopoll_conn_is_ready(nopoll_info.conn))
														{
									
															if(nopoll_conn_send_text(nopoll_info.conn, sendtext,length)==length)
																{
																	success=1;
																	break;
																}
									
														}
													sleep(2);
													if(i>15)
														{
															i=0;
															break;
									
														}
							
												}
										}
									if(success==1)
										{
											success=0;
											nopoll_conn_set_on_msg(nopoll_info.conn, mleaf_cb_recv_handler,NULL);
											nopoll_conn_set_on_close(nopoll_info.conn,nopoll_client_close_handle,NULL);
											nopoll_info.nopoll_isok_flag=1;
											printf("now nopoll_loop_wait\n");
											nopoll_loop_wait(nopoll_info.ctx,0);
											nopoll_info.nopoll_isok_flag=0;
											nopoll_info.request_rfidinfo=0;
					
										}
									
										}
						}
					nopoll_conn_opts_free(nopoll_info.opts);
				}
				sleep(5);
		}
	return;

}*/

void * creat_nopoll_connection(void)
{
	char *port="8743";
	char ip[36]={0};
	char ipaddr[36]={0};
	int success=0;
	int i=0;
	char *sendtext="hello!";
	int length;
	ipaddr[0]='/';
	length=strlen(sendtext);
	strcpy(&ipaddr[1],base_info.id);
	
	
	
	
	while(1)
		{
			memset(ip,0,36);
			strcpy(ip,base_info.ip);
			if(nopoll_info.nopoll_create_ctx_first!=0)
				{
					nopoll_ctx_unref (nopoll_info.ctx);
				}
			printf("============================now start to  create ctx\n");
			while((nopoll_info.ctx=create_ctx())==NULL)
				{
					printf("create ctx false now into while\n");
					sleep(5);
				}
			printf("============================ create ctx success\n");
			nopoll_info.nopoll_create_ctx_first=1;
			nopoll_info.count_of_failure=0;
			if(nopoll_info.nopoll_create_first!=0)
				{
					nopoll_conn_close(nopoll_info.conn);//关闭conn
				}
			
			if(1==0)
				{
					printf("create opts false now into while\n");
					sleep(5);
				}
			else
				{
					if(1==0)
						{
							printf("nopoll_conn_opts_set_ssl_certs false\n");
							sleep(5);
						}
					else
						{
							printf("============================ now to create conn\n");
							if(( nopoll_info.conn= nopoll_conn_new (nopoll_info.ctx, ip, port, NULL, ipaddr, NULL, NULL)) ==NULL)
								{
									printf("creat conn false now into while\n");
									sleep(5);
								}
							else
								{
									printf("============================  create conn success\n");
									nopoll_info.nopoll_create_first=1;//表示创建了conn 需要关闭conn
									if(!nopoll_conn_is_ok(nopoll_info.conn))
										{
											printf("==================================conn is not ok\n");
											sleep(5);
										}
									else
										{
											while(1)
												{
													i++;
													if(nopoll_conn_is_ready(nopoll_info.conn))
														{
															printf("====================conn is ready\n");
															if(nopoll_conn_send_text(nopoll_info.conn, sendtext,length)==length)
																{
																	success=1;
																	break;
																}
									
														}
													sleep(2);
													if(i>15)
														{
															i=0;
															break;
									
														}
							
												}
										}
									if(success==1)
										{
											printf("====================conn is ready\n");
											success=0;
											nopoll_conn_set_on_msg(nopoll_info.conn, mleaf_cb_recv_handler,NULL);
											nopoll_conn_set_on_close(nopoll_info.conn,nopoll_client_close_handle,NULL);
											nopoll_info.nopoll_isok_flag=1;
											printf("now nopoll_loop_wait\n");
											nopoll_info.connect_report_flag=1;
											nopoll_loop_wait(nopoll_info.ctx,0);
											
											nopoll_info.nopoll_isok_flag=0;
											nopoll_info.request_rfidinfo=0;
					
										}
									
										}
						}
					//nopoll_conn_opts_free(nopoll_info.opts);
				}
				sleep(5);
		}
	return;

}


int report_fuction(void)
{
	char info_to_report[2048];
	json_object *screen=json_object_new_object();
	json_object *switchn=json_object_new_array();
	json_object *sensor=json_object_new_object();
	json_object *rfid=json_object_new_object();

	json_object *host=json_object_new_object();
	json_object *pc=json_object_new_object();
	json_object *display=json_object_new_object();
	json_object *booth=json_object_new_object();	

	json_object *mainjson=json_object_new_object();
	json_object *mainjson1=json_object_new_object();
	//host
	printf("555555555555555555	0\n");

	pthread_mutex_lock(&projector_info.mutex);
	printf("555555555555555555  1\n");
	read_all_data_from_mcu();
	//sleep(2);
	pthread_mutex_unlock(&projector_info.mutex);
	printf("555555555555555555  2\n");





	pthread_mutex_lock(&nopoll_info.mutex);
	printf("555555555555555555  3\n");

	
	char *host_status;
	if(device_status.k4==1)
		{
			host_status="WORKING";
		}
	else
		{
			host_status="STANDBY";
		}

	json_object_object_add(host,"status",json_object_new_string(host_status));


	//pc
	char *pc_status;
	if(device_status.pc==1)
		{
			pc_status="WORKING";
		}
	else
		{
			pc_status="STANDBY";
		}
	

	json_object_object_add(pc,"identify",json_object_new_string("pc"));
	json_object_object_add(pc,"status",json_object_new_string(pc_status));


	//booth
	char *booth_status;
	if(device_status.stand_lock==1)
		{
			booth_status="WORKING";
		}
	else
		{
			booth_status="STANDBY";
		}
	

	json_object_object_add(booth,"identify",json_object_new_string("booth"));
	json_object_object_add(booth,"status",json_object_new_string(booth_status));


	//display
	char *display_status;
	if(device_status.projector==1)
		{
			display_status="WORKING";
		}
	else
		{
			display_status="STANDBY";
		}
	

	json_object_object_add(display,"identify",json_object_new_string("dtsDisplayer"));
	json_object_object_add(display,"status",json_object_new_string(display_status));









/**screen***/

	/*json_object *status_screen=json_object_new_object();
	json_object *scree=json_object_new_array();
	json_object **screen_list;
	///////////////////////////////////////////////////////////////////////
	if(wifi_screenheader.number>0)
			{
				int i=0;
				printf("screen is used\n");
				struct wifi_screen *a;
				a=wifi_screenheader.next;
				char *status;
				screen_list=(json_object *)malloc(wifi_screenheader.number*sizeof(json_object *));
				for(i=0;i<wifi_screenheader.number;i++)
					{
						screen_list[i]=json_object_new_object();
						json_object_object_add(screen_list[i],"deviceIdentify",json_object_new_string(a->name));
						if(a->status>0)
							{
								status="WORKING";
							}
						if(a->status==0)
							{
								status="OFFLINE";
							}
						json_object_object_add(screen_list[i],"deviceStatus",json_object_new_string(status));
	
						if(a->del==0)
						json_object_array_add(scree,screen_list[i]);
	
						if(i!=(wifi_screenheader.number-1))
							{
								a=a->next;
							}
					}
			json_object_object_add(screen1, "deviceType",json_object_new_string("DTS_SCREEN"));
			json_object_object_add(screen1,"status",scree);
				}*/




	














































	int have_sensor=0;
	//sensor
	if(wifi_sensorheader.number>0)
		{
	struct wifi_sensor *sensor_a;
	sensor_a=wifi_sensorheader.next;
	while(1)
		{
			if(sensor_a->del==1)
				{
					if(sensor_a->last==1)
						{
							have_sensor=0;
							break;
						}
					else
						{
							sensor_a=sensor_a->next;
							continue;
						}
				}
			else
				{
					have_sensor=1;
					break;
				}
		}
	char *sensor_status;
	if(have_sensor==1)
		{
	if(sensor_a->status>=1)
		{
			display_status="WORKING";
		}
	else
		{
			display_status="OFFLINE";
		}

	json_object_object_add(sensor,"identify",json_object_new_string("Sensor"));
	json_object_object_add(sensor,"status",json_object_new_string(display_status));
	json_object_object_add(sensor,"temperature",json_object_new_int(sensor_a->temperature));
	json_object_object_add(sensor,"humidness",json_object_new_int(sensor_a->humidity));
	json_object_object_add(sensor,"illumination",json_object_new_int(sensor_a->illumination));
		}
	}

	//rfid
	int have_rfid=0;
	if(wifi_rfidheader.number>0)
		{
	struct wifi_rfid *rfid_a;
	rfid_a=wifi_rfidheader.next;
	while(1)
		{
			if(rfid_a->del==1)
				{
					if(rfid_a->last==1)
						{
							have_rfid=0;
							break;
						}
					else
						{
							rfid_a=rfid_a->next;
							continue;
						}
				}
			else
				{
					have_rfid=1;
					break;
				}
		}
	char *rfid_status;
	if(have_rfid==1)
		{
	if(rfid_a->status>=1)
		{
			rfid_status="WORKING";
		}
	else
		{
			rfid_status="OFFLINE";
		}
	//json_object *rfid1=json_object_new_object();
	json_object_object_add(rfid,"identify",json_object_new_string("RFID"));
	json_object_object_add(rfid,"status",json_object_new_string(rfid_status));
	//json_object_array_add(rfid,rfid1);
		}
		}

	//switch




	struct switch_json_header switch_jsonheader;
	switch_jsonheader.number=0;
	switch_jsonheader.next=NULL;
	if(wifi_switchheader.number>0)
		{
			struct wifi_switch *switch_a;
			struct switch_json *switch_json_a;
			struct switch_json *switch_json_b;
			int lamp_nu=0;
			
			char lamp_numb[5];
			
			switch_a=wifi_switchheader.next;
			while(1)
				{
					
					if(switch_a->del==1)
						{
							switch_a=switch_a->next;
							continue;
						}
					else
						{
							//switch_json_a=(struct switch_json *)malloc(sizeof(struct switch_json));
							while((switch_json_a=(struct switch_json *)malloc(sizeof(struct switch_json)))==NULL)
								{
									printf("malloc false\n");
									sleep(2);
								}
							switch_json_a->lamp_number=switch_a->lamp_number;
							switch_json_a->last=1;
							switch_json_a->switch_array=json_object_new_array();
							switch_json_a->switch_switch=json_object_new_object();
							//switch_json_a->lamps=(json_object **)malloc(switch_json_a->lamp_number*sizeof(json_object *));
							while((switch_json_a->lamps=(json_object **)malloc(switch_json_a->lamp_number*sizeof(json_object *)))==NULL)
								{
									printf("malloc false\n");
									sleep(2);
								}
							for(lamp_nu=0;lamp_nu<switch_json_a->lamp_number;lamp_nu++)
								{
									switch_json_a->lamps[lamp_nu]=json_object_new_object();
									memset(lamp_numb,0,5);
									sprintf(lamp_numb,"%d",(lamp_nu+1));
									json_object_object_add(switch_json_a->lamps[lamp_nu],"identify",json_object_new_string(lamp_numb));
									json_object_object_add(switch_json_a->lamps[lamp_nu],"status",json_object_new_string(switch_a->lamp[lamp_nu]));
									json_object_array_add(switch_json_a->switch_array,switch_json_a->lamps[lamp_nu]);
								}

							
							json_object_object_add(switch_json_a->switch_switch,"name",json_object_new_string(switch_a->name));
							json_object_object_add(switch_json_a->switch_switch,"mac",json_object_new_string(switch_a->mac));
							json_object_object_add(switch_json_a->switch_switch,"switch",switch_json_a->switch_array);
							json_object_array_add(switchn,switch_json_a->switch_switch);


							if(switch_jsonheader.number==0)
								{
									switch_jsonheader.next=switch_json_a;
									switch_json_b=switch_json_a;
									switch_jsonheader.number++;
								}
							else
								{
									switch_json_b->next=switch_json_a;
									switch_json_b->last=0;
									switch_json_b=switch_json_a;
									switch_jsonheader.number++;
								}
							if(switch_a->last==1)
								{
									break;
								}
							else
								{
									switch_a=switch_a->next;
								}
						}
				}
		}

	

	/*json_object *switch_array=json_object_new_array();
	json_object *switch_array1=json_object_new_array();
	json_object *switch_array2=json_object_new_array();

	json_object *switch1=json_object_new_object();
	json_object *lamp1_1=json_object_new_object();
	json_object *lamp1_2=json_object_new_object();

	json_object *switch2=json_object_new_object();
	json_object *lamp2_1=json_object_new_object();
	json_object *lamp2_2=json_object_new_object();

	char *lamp1_1_status,*lamp1_2_status,*lamp2_1_status,*lamp2_2_status;
	if(info.switch1_lamp1==1)
		{
			lamp1_1_status="WORKING";
		}
	else
		{
			lamp1_1_status="STANDBY";
		}
	
	if(info.switch1_lamp2==1)
		{
			lamp1_2_status="WORKING";
		}
	else
		{
			lamp1_2_status="STANDBY";
		}
	
	if(info.switch2_lamp1==1)
		{
			lamp2_1_status="WORKING";
		}
	else
		{
			lamp2_1_status="STANDBY";
		}
	
	if(info.switch2_lamp2==1)
		{
			lamp2_2_status="WORKING";
		}
	else
		{
			lamp2_2_status="STANDBY";
		}
	json_object_object_add(lamp1_1,"identify",json_object_new_string("1"));
	json_object_object_add(lamp1_1,"status",json_object_new_string(lamp1_1_status));
	json_object_object_add(lamp1_2,"identify",json_object_new_string("2"));
	json_object_object_add(lamp1_2,"status",json_object_new_string(lamp1_2_status));

	json_object_object_add(lamp2_1,"identify",json_object_new_string("1"));
	json_object_object_add(lamp2_1,"status",json_object_new_string(lamp2_1_status));
	json_object_object_add(lamp2_2,"identify",json_object_new_string("2"));
	json_object_object_add(lamp2_2,"status",json_object_new_string(lamp2_2_status));

	json_object_array_add(switch_array1,lamp1_1);
	json_object_array_add(switch_array1,lamp1_2);

	json_object_array_add(switch_array2,lamp2_1);
	json_object_array_add(switch_array2,lamp2_2);

	json_object_object_add(switch1,"name",json_object_new_string("switch1"));
	json_object_object_add(switch1,"mac",json_object_new_string(info.switch1_mac));
	json_object_object_add(switch1,"switch",switch_array1);

	json_object_object_add(switch2,"name",json_object_new_string("switch2"));
	json_object_object_add(switch2,"mac",json_object_new_string(info.switch2_mac));
	json_object_object_add(switch2,"switch",switch_array2);

	json_object_array_add(switchn,switch1);
	json_object_array_add(switchn,switch2);*/
	
 	int have_screen=0;
	//screen
	if(wifi_screenheader.number>0)
		{
	struct wifi_screen *screen_a;
	screen_a=wifi_screenheader.next;
	while(1)
		{
			if(screen_a->del==1)
				{
					if(screen_a->last==1)
						{
							have_screen=0;
							break;
						}
					else
						{
							screen_a=screen_a->next;
							continue;
						}
				}
			else
				{
					have_screen=1;
					break;
				}
		}
	
		
	char *screen_status;
	
	if(have_screen==1)
		{
	if(screen_a->status>=1)
		{
			screen_status="WORKING";
		}
	else
		{
			screen_status="OFFLINE";
		}
	json_object_object_add(screen,"identify",json_object_new_string("screen"));
	json_object_object_add(screen,"status",json_object_new_string(screen_status));
		}
		}

	//sunmary
	
	json_object_object_add(mainjson1,"result",json_object_new_int(1));
	json_object_object_add(mainjson1,"function",json_object_new_string("reportstatus"));
	json_object_object_add(mainjson1,"msg",json_object_new_string("chenggong"));
	json_object_object_add(mainjson1,"dtsName",json_object_new_string(base_info.name));
	json_object_object_add(mainjson1,"dtsHost",host);
	json_object_object_add(mainjson1,"dtsPC",pc);
	json_object_object_add(mainjson1,"dtsVolume",json_object_new_int(volume_3105.volume*2));
	json_object_object_add(mainjson1,"dtsChannel",json_object_new_int(device_status.ep-1));
	json_object_object_add(mainjson1,"dtsBooth",booth);
	json_object_object_add(mainjson1,"dtsDisplayer",display);
	json_object_object_add(mainjson1,"mediaaip",json_object_new_string(base_info.mediaa));
	json_object_object_add(mainjson1,"mediabip",json_object_new_string(base_info.mediab));
	if(have_sensor==1)
		{
	json_object_object_add(mainjson1,"dtsSensor",sensor);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsSensor",json_object_new_string("null"));
		}
	if(have_rfid==1)
		{
	json_object_object_add(mainjson1,"dtsRfid",rfid);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsRfid",json_object_new_string("null"));
		}
	if(switch_jsonheader.number>0)
		{
	json_object_object_add(mainjson1,"dtsSwitch",switchn);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsSwitch",json_object_new_string("null"));
		}
	if(have_screen==1)
		{
	json_object_object_add(mainjson1,"dtsScreen",screen);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsScreen",json_object_new_string("null"));
		}

	//json_object_object_add(mainjson,"dtsinfo",mainjson1);
	memset(info_to_report,0,2048);
	sprintf(info_to_report,"%s",json_object_to_json_string(mainjson1));

	int lent;
	lent=strlen(info_to_report);
	char *requestrfid="{\"function\":\"requestrfidnotsend\", \"msg\":\"success\"}";
	int i=0;
	int j=0;
	int return_sendlength=0;
	
	printf("dtsinfo = %s\n%d\n",info_to_report,lent);
	
	if(nopoll_info.nopoll_isok_flag==1)
		{
			if(nopoll_conn_is_ready(nopoll_info.conn))
				{
					if(nopoll_info.request_rfidinfo<2)
						{
							//printf("requestrfidindo = %s\n",requestrfid);
							nopoll_conn_send_text(nopoll_info.conn,requestrfid,strlen(requestrfid));
							usleep(200000);
						}
					//printf("report status = %s\n",info_to_report);
					return_sendlength=nopoll_conn_send_text(nopoll_info.conn,info_to_report,lent);
					printf("return_sendlength = %d    lent=%d\n",return_sendlength,lent);
					//if(lent!=return_sendlength)
						//{
					nopoll_info.count_of_failure++;
						//}

					//printf("nopoll_conn_is_ready success count_of_failure =%d\n",nopoll_info.count_of_failure);
				}
			else
				{
					nopoll_info.count_of_failure++;
					printf("nopoll_conn_is_ready false count_of_failure =%d\n",nopoll_info.count_of_failure);
				}
			if(nopoll_info.count_of_failure>1)
				{
					nopoll_loop_stop(nopoll_info.ctx);
				}
		}
	
	/*while(info_to_report[i]!='\0')
		{
			if(info_to_report[i]!=' ')
				{
					info_to_report[j]=info_to_report[i];
					j++;
					i++;
				}
			else
				{
					i++;
				}
		}
	info_to_report[j]='\0';
	lent=strlen(info_to_report);
	printf("dtsinfo = %s\n%d\n",info_to_report,lent);*/
	
	
	json_object_put(mainjson);
	json_object_put(mainjson1);
	json_object_put(host);
	json_object_put(pc);
	json_object_put(booth);
	json_object_put(display);
	json_object_put(sensor);
	json_object_put(rfid);
	json_object_put(switchn);
	json_object_put(screen);
	if(switch_jsonheader.number>0)
		{
			struct switch_json *a;
			int a1=0;
			while(1)
				{	
					a=switch_jsonheader.next;
					json_object_put(a->switch_switch);
					json_object_put(a->switch_array);
					for(a1=0;a1<a->lamp_number;a1++)
						{
							json_object_put(a->lamps[a1]);
						}
					free(a->lamps);
					if(a->last!=1)
						{
							switch_jsonheader.next=a->next;
							free(a);
						}
					else
						{
							free(a);
							break;
						}
					
				}
		}
	pthread_mutex_unlock(&nopoll_info.mutex);
	return 1;
}

int report_fuction1(void)
{
	char info_to_report[2048];
	json_object *screen=json_object_new_object();
	json_object *switchn=json_object_new_array();
	json_object *sensor=json_object_new_object();
	json_object *rfid=json_object_new_object();

	json_object *host=json_object_new_object();
	json_object *pc=json_object_new_object();
	json_object *display=json_object_new_object();
	json_object *booth=json_object_new_object();	

	json_object *mainjson=json_object_new_object();
	json_object *mainjson1=json_object_new_object();
	//host

	/*pthread_mutex_lock(&projector_info.mutex);
	read_all_data_from_mcu();
	//sleep(2);
	pthread_mutex_unlock(&projector_info.mutex);*/





	pthread_mutex_lock(&nopoll_info.mutex);

	
	char *host_status;
	if(device_status.k4==1)
		{
			host_status="WORKING";
		}
	else
		{
			host_status="STANDBY";
		}

	json_object_object_add(host,"status",json_object_new_string(host_status));


	//pc
	char *pc_status;
	if(device_status.pc==1)
		{
			pc_status="WORKING";
		}
	else
		{
			pc_status="STANDBY";
		}
	

	json_object_object_add(pc,"identify",json_object_new_string("pc"));
	json_object_object_add(pc,"status",json_object_new_string(pc_status));


	//booth
	char *booth_status;
	if(device_status.stand_lock==1)
		{
			booth_status="WORKING";
		}
	else
		{
			booth_status="STANDBY";
		}
	

	json_object_object_add(booth,"identify",json_object_new_string("booth"));
	json_object_object_add(booth,"status",json_object_new_string(booth_status));


	//display
	char *display_status;
	if(device_status.projector==1)
		{
			display_status="WORKING";
		}
	else
		{
			display_status="STANDBY";
		}
	

	json_object_object_add(display,"identify",json_object_new_string("dtsDisplayer"));
	json_object_object_add(display,"status",json_object_new_string(display_status));









/**screen***/

	/*json_object *status_screen=json_object_new_object();
	json_object *scree=json_object_new_array();
	json_object **screen_list;
	///////////////////////////////////////////////////////////////////////
	if(wifi_screenheader.number>0)
			{
				int i=0;
				printf("screen is used\n");
				struct wifi_screen *a;
				a=wifi_screenheader.next;
				char *status;
				screen_list=(json_object *)malloc(wifi_screenheader.number*sizeof(json_object *));
				for(i=0;i<wifi_screenheader.number;i++)
					{
						screen_list[i]=json_object_new_object();
						json_object_object_add(screen_list[i],"deviceIdentify",json_object_new_string(a->name));
						if(a->status>0)
							{
								status="WORKING";
							}
						if(a->status==0)
							{
								status="OFFLINE";
							}
						json_object_object_add(screen_list[i],"deviceStatus",json_object_new_string(status));
	
						if(a->del==0)
						json_object_array_add(scree,screen_list[i]);
	
						if(i!=(wifi_screenheader.number-1))
							{
								a=a->next;
							}
					}
			json_object_object_add(screen1, "deviceType",json_object_new_string("DTS_SCREEN"));
			json_object_object_add(screen1,"status",scree);
				}*/




	














































	int have_sensor=0;
	//sensor
	if(wifi_sensorheader.number>0)
		{
	struct wifi_sensor *sensor_a;
	sensor_a=wifi_sensorheader.next;
	while(1)
		{
			if(sensor_a->del==1)
				{
					if(sensor_a->last==1)
						{
							have_sensor=0;
							break;
						}
					else
						{
							sensor_a=sensor_a->next;
							continue;
						}
				}
			else
				{
					have_sensor=1;
					break;
				}
		}
	char *sensor_status;
	if(have_sensor==1)
		{
	if(sensor_a->status>=1)
		{
			display_status="WORKING";
		}
	else
		{
			display_status="OFFLINE";
		}

	json_object_object_add(sensor,"identify",json_object_new_string("Sensor"));
	json_object_object_add(sensor,"status",json_object_new_string(display_status));
	json_object_object_add(sensor,"temperature",json_object_new_int(sensor_a->temperature));
	json_object_object_add(sensor,"humidness",json_object_new_int(sensor_a->humidity));
	json_object_object_add(sensor,"illumination",json_object_new_int(sensor_a->illumination));
		}
	}

	//rfid
	int have_rfid=0;
	if(wifi_rfidheader.number>0)
		{
	struct wifi_rfid *rfid_a;
	rfid_a=wifi_rfidheader.next;
	while(1)
		{
			if(rfid_a->del==1)
				{
					if(rfid_a->last==1)
						{
							have_rfid=0;
							break;
						}
					else
						{
							rfid_a=rfid_a->next;
							continue;
						}
				}
			else
				{
					have_rfid=1;
					break;
				}
		}
	char *rfid_status;
	if(have_rfid==1)
		{
	if(rfid_a->status>=1)
		{
			rfid_status="WORKING";
		}
	else
		{
			rfid_status="OFFLINE";
		}
	//json_object *rfid1=json_object_new_object();
	json_object_object_add(rfid,"identify",json_object_new_string("RFID"));
	json_object_object_add(rfid,"status",json_object_new_string(rfid_status));
	//json_object_array_add(rfid,rfid1);
		}
		}

	//switch




	struct switch_json_header switch_jsonheader;
	switch_jsonheader.number=0;
	switch_jsonheader.next=NULL;
	if(wifi_switchheader.number>0)
		{
			struct wifi_switch *switch_a;
			struct switch_json *switch_json_a;
			struct switch_json *switch_json_b;
			int lamp_nu=0;
			
			char lamp_numb[5];
			
			switch_a=wifi_switchheader.next;
			while(1)
				{
					
					if(switch_a->del==1)
						{
							switch_a=switch_a->next;
							continue;
						}
					else
						{
							//switch_json_a=(struct switch_json *)malloc(sizeof(struct switch_json));
							while((switch_json_a=(struct switch_json *)malloc(sizeof(struct switch_json)))==NULL)
								{
									sleep(2);
								}
							switch_json_a->lamp_number=switch_a->lamp_number;
							switch_json_a->last=1;
							switch_json_a->switch_array=json_object_new_array();
							switch_json_a->switch_switch=json_object_new_object();
							//switch_json_a->lamps=(json_object **)malloc(switch_json_a->lamp_number*sizeof(json_object *));
							while((switch_json_a->lamps=(json_object **)malloc(switch_json_a->lamp_number*sizeof(json_object *)))==NULL)
								{
									sleep(2);
								}
							for(lamp_nu=0;lamp_nu<switch_json_a->lamp_number;lamp_nu++)
								{
									switch_json_a->lamps[lamp_nu]=json_object_new_object();
									memset(lamp_numb,0,5);
									sprintf(lamp_numb,"%d",(lamp_nu+1));
									json_object_object_add(switch_json_a->lamps[lamp_nu],"identify",json_object_new_string(lamp_numb));
									json_object_object_add(switch_json_a->lamps[lamp_nu],"status",json_object_new_string(switch_a->lamp[lamp_nu]));
									json_object_array_add(switch_json_a->switch_array,switch_json_a->lamps[lamp_nu]);
								}

							
							json_object_object_add(switch_json_a->switch_switch,"name",json_object_new_string(switch_a->name));
							json_object_object_add(switch_json_a->switch_switch,"mac",json_object_new_string(switch_a->mac));
							json_object_object_add(switch_json_a->switch_switch,"switch",switch_json_a->switch_array);
							json_object_array_add(switchn,switch_json_a->switch_switch);


							if(switch_jsonheader.number==0)
								{
									switch_jsonheader.next=switch_json_a;
									switch_json_b=switch_json_a;
									switch_jsonheader.number++;
								}
							else
								{
									switch_json_b->next=switch_json_a;
									switch_json_b->last=0;
									switch_json_b=switch_json_a;
									switch_jsonheader.number++;
								}
							if(switch_a->last==1)
								{
									break;
								}
							else
								{
									switch_a=switch_a->next;
								}
						}
				}
		}

	

	/*json_object *switch_array=json_object_new_array();
	json_object *switch_array1=json_object_new_array();
	json_object *switch_array2=json_object_new_array();

	json_object *switch1=json_object_new_object();
	json_object *lamp1_1=json_object_new_object();
	json_object *lamp1_2=json_object_new_object();

	json_object *switch2=json_object_new_object();
	json_object *lamp2_1=json_object_new_object();
	json_object *lamp2_2=json_object_new_object();

	char *lamp1_1_status,*lamp1_2_status,*lamp2_1_status,*lamp2_2_status;
	if(info.switch1_lamp1==1)
		{
			lamp1_1_status="WORKING";
		}
	else
		{
			lamp1_1_status="STANDBY";
		}
	
	if(info.switch1_lamp2==1)
		{
			lamp1_2_status="WORKING";
		}
	else
		{
			lamp1_2_status="STANDBY";
		}
	
	if(info.switch2_lamp1==1)
		{
			lamp2_1_status="WORKING";
		}
	else
		{
			lamp2_1_status="STANDBY";
		}
	
	if(info.switch2_lamp2==1)
		{
			lamp2_2_status="WORKING";
		}
	else
		{
			lamp2_2_status="STANDBY";
		}
	json_object_object_add(lamp1_1,"identify",json_object_new_string("1"));
	json_object_object_add(lamp1_1,"status",json_object_new_string(lamp1_1_status));
	json_object_object_add(lamp1_2,"identify",json_object_new_string("2"));
	json_object_object_add(lamp1_2,"status",json_object_new_string(lamp1_2_status));

	json_object_object_add(lamp2_1,"identify",json_object_new_string("1"));
	json_object_object_add(lamp2_1,"status",json_object_new_string(lamp2_1_status));
	json_object_object_add(lamp2_2,"identify",json_object_new_string("2"));
	json_object_object_add(lamp2_2,"status",json_object_new_string(lamp2_2_status));

	json_object_array_add(switch_array1,lamp1_1);
	json_object_array_add(switch_array1,lamp1_2);

	json_object_array_add(switch_array2,lamp2_1);
	json_object_array_add(switch_array2,lamp2_2);

	json_object_object_add(switch1,"name",json_object_new_string("switch1"));
	json_object_object_add(switch1,"mac",json_object_new_string(info.switch1_mac));
	json_object_object_add(switch1,"switch",switch_array1);

	json_object_object_add(switch2,"name",json_object_new_string("switch2"));
	json_object_object_add(switch2,"mac",json_object_new_string(info.switch2_mac));
	json_object_object_add(switch2,"switch",switch_array2);

	json_object_array_add(switchn,switch1);
	json_object_array_add(switchn,switch2);*/
	
 	int have_screen=0;
	//screen
	if(wifi_screenheader.number>0)
		{
	struct wifi_screen *screen_a;
	screen_a=wifi_screenheader.next;
	while(1)
		{
			if(screen_a->del==1)
				{
					if(screen_a->last==1)
						{
							have_screen=0;
							break;
						}
					else
						{
							screen_a=screen_a->next;
							continue;
						}
				}
			else
				{
					have_screen=1;
					break;
				}
		}
	
		
	char *screen_status;
	
	if(have_screen==1)
		{
	if(screen_a->status>=1)
		{
			screen_status="WORKING";
		}
	else
		{
			screen_status="OFFLINE";
		}
	json_object_object_add(screen,"identify",json_object_new_string("screen"));
	json_object_object_add(screen,"status",json_object_new_string(screen_status));
		}
		}

	//sunmary
	
	json_object_object_add(mainjson1,"result",json_object_new_int(1));
	json_object_object_add(mainjson1,"function",json_object_new_string("reportstatus"));
	json_object_object_add(mainjson1,"msg",json_object_new_string("chenggong"));
	json_object_object_add(mainjson1,"dtsName",json_object_new_string(base_info.name));
	json_object_object_add(mainjson1,"dtsHost",host);
	json_object_object_add(mainjson1,"dtsPC",pc);
	json_object_object_add(mainjson1,"dtsVolume",json_object_new_int(volume_3105.volume*2));
	json_object_object_add(mainjson1,"dtsChannel",json_object_new_int(device_status.ep-1));
	json_object_object_add(mainjson1,"dtsBooth",booth);
	json_object_object_add(mainjson1,"dtsDisplayer",display);
	json_object_object_add(mainjson1,"mediaaip",json_object_new_string(base_info.mediaa));
	json_object_object_add(mainjson1,"mediabip",json_object_new_string(base_info.mediab));
	if(have_sensor==1)
		{
	json_object_object_add(mainjson1,"dtsSensor",sensor);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsSensor",json_object_new_string("null"));
		}
	if(have_rfid==1)
		{
	json_object_object_add(mainjson1,"dtsRfid",rfid);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsRfid",json_object_new_string("null"));
		}
	if(switch_jsonheader.number>0)
		{
	json_object_object_add(mainjson1,"dtsSwitch",switchn);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsSwitch",json_object_new_string("null"));
		}
	if(have_screen==1)
		{
	json_object_object_add(mainjson1,"dtsScreen",screen);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsScreen",json_object_new_string("null"));
		}

	//json_object_object_add(mainjson,"dtsinfo",mainjson1);
	memset(info_to_report,0,2048);
	sprintf(info_to_report,"%s",json_object_to_json_string(mainjson1));

	int lent;
	lent=strlen(info_to_report);
	char *requestrfid="{\"function\":\"requestrfidnotsend\", \"msg\":\"success\"}";
	int i=0;
	int j=0;
	printf("dtsinfo = %s\n%d\n",info_to_report,lent);
	if(nopoll_info.nopoll_isok_flag==1)
		{
			if(nopoll_conn_is_ready(nopoll_info.conn))
				{
					if(nopoll_info.request_rfidinfo<2)
						{
							//printf("requestrfidindo = %s\n",requestrfid);
							nopoll_conn_send_text(nopoll_info.conn,requestrfid,strlen(requestrfid));
							nopoll_info.request_rfidinfo++;
							sleep(1);
						}
					//printf("report status = %s\n",info_to_report);
					nopoll_conn_send_text(nopoll_info.conn,info_to_report,lent);					
				}
			else
				{
					nopoll_info.count_of_failure++;
				}
			if(nopoll_info.count_of_failure>5)
				{
					nopoll_loop_stop(nopoll_info.ctx);
				}
		}
	
	/*while(info_to_report[i]!='\0')
		{
			if(info_to_report[i]!=' ')
				{
					info_to_report[j]=info_to_report[i];
					j++;
					i++;
				}
			else
				{
					i++;
				}
		}
	info_to_report[j]='\0';
	lent=strlen(info_to_report);
	printf("dtsinfo = %s\n%d\n",info_to_report,lent);*/
	
	
	json_object_put(mainjson);
	json_object_put(mainjson1);
	json_object_put(host);
	json_object_put(pc);
	json_object_put(booth);
	json_object_put(display);
	json_object_put(sensor);
	json_object_put(rfid);
	json_object_put(switchn);
	json_object_put(screen);
	if(switch_jsonheader.number>0)
		{
			struct switch_json *a;
			int a1=0;
			while(1)
				{	
					a=switch_jsonheader.next;
					json_object_put(a->switch_switch);
					json_object_put(a->switch_array);
					for(a1=0;a1<a->lamp_number;a1++)
						{
							json_object_put(a->lamps[a1]);
						}
					free(a->lamps);
					if(a->last!=1)
						{
							switch_jsonheader.next=a->next;
							free(a);
						}
					else
						{
							free(a);
							break;
						}
					
				}
		}
	pthread_mutex_unlock(&nopoll_info.mutex);
	return 1;
}

int report_fuction2(void)
{
	char info_to_report[2048];
	json_object *screen=json_object_new_object();
	json_object *switchn=json_object_new_array();
	json_object *sensor=json_object_new_object();
	json_object *rfid=json_object_new_object();

	json_object *host=json_object_new_object();
	json_object *pc=json_object_new_object();
	json_object *display=json_object_new_object();
	json_object *booth=json_object_new_object();	

	json_object *mainjson=json_object_new_object();
	json_object *mainjson1=json_object_new_object();
	//host

	/*pthread_mutex_lock(&projector_info.mutex);
	read_all_data_from_mcu();
	//sleep(2);
	pthread_mutex_unlock(&projector_info.mutex);*/





	pthread_mutex_lock(&nopoll_info.mutex);

	
	char *host_status;
	if(device_status.k4==1)
		{
			host_status="WORKING";
		}
	else
		{
			host_status="STANDBY";
		}

	json_object_object_add(host,"status",json_object_new_string(host_status));


	//pc
	char *pc_status;
	if(device_status.pc==1)
		{
			pc_status="WORKING";
		}
	else
		{
			pc_status="STANDBY";
		}
	

	json_object_object_add(pc,"identify",json_object_new_string("pc"));
	json_object_object_add(pc,"status",json_object_new_string(pc_status));


	//booth
	char *booth_status;
	if(device_status.stand_lock==1)
		{
			booth_status="WORKING";
		}
	else
		{
			booth_status="STANDBY";
		}
	

	json_object_object_add(booth,"identify",json_object_new_string("booth"));
	json_object_object_add(booth,"status",json_object_new_string(booth_status));


	//display
	char *display_status;
	if(device_status.projector==1)
		{
			display_status="WORKING";
		}
	else
		{
			display_status="STANDBY";
		}
	

	json_object_object_add(display,"identify",json_object_new_string("dtsDisplayer"));
	json_object_object_add(display,"status",json_object_new_string(display_status));









/**screen***/

	/*json_object *status_screen=json_object_new_object();
	json_object *scree=json_object_new_array();
	json_object **screen_list;
	///////////////////////////////////////////////////////////////////////
	if(wifi_screenheader.number>0)
			{
				int i=0;
				printf("screen is used\n");
				struct wifi_screen *a;
				a=wifi_screenheader.next;
				char *status;
				screen_list=(json_object *)malloc(wifi_screenheader.number*sizeof(json_object *));
				for(i=0;i<wifi_screenheader.number;i++)
					{
						screen_list[i]=json_object_new_object();
						json_object_object_add(screen_list[i],"deviceIdentify",json_object_new_string(a->name));
						if(a->status>0)
							{
								status="WORKING";
							}
						if(a->status==0)
							{
								status="OFFLINE";
							}
						json_object_object_add(screen_list[i],"deviceStatus",json_object_new_string(status));
	
						if(a->del==0)
						json_object_array_add(scree,screen_list[i]);
	
						if(i!=(wifi_screenheader.number-1))
							{
								a=a->next;
							}
					}
			json_object_object_add(screen1, "deviceType",json_object_new_string("DTS_SCREEN"));
			json_object_object_add(screen1,"status",scree);
				}*/




	














































	int have_sensor=0;
	//sensor
	if(wifi_sensorheader.number>0)
		{
	struct wifi_sensor *sensor_a;
	sensor_a=wifi_sensorheader.next;
	while(1)
		{
			if(sensor_a->del==1)
				{
					if(sensor_a->last==1)
						{
							have_sensor=0;
							break;
						}
					else
						{
							sensor_a=sensor_a->next;
							continue;
						}
				}
			else
				{
					have_sensor=1;
					break;
				}
		}
	char *sensor_status;
	if(have_sensor==1)
		{
	if(sensor_a->status>=1)
		{
			display_status="WORKING";
		}
	else
		{
			display_status="OFFLINE";
		}

	json_object_object_add(sensor,"identify",json_object_new_string("Sensor"));
	json_object_object_add(sensor,"status",json_object_new_string(display_status));
	json_object_object_add(sensor,"temperature",json_object_new_int(sensor_a->temperature));
	json_object_object_add(sensor,"humidness",json_object_new_int(sensor_a->humidity));
	json_object_object_add(sensor,"illumination",json_object_new_int(sensor_a->illumination));
		}
	}

	//rfid
	int have_rfid=0;
	if(wifi_rfidheader.number>0)
		{
	struct wifi_rfid *rfid_a;
	rfid_a=wifi_rfidheader.next;
	while(1)
		{
			if(rfid_a->del==1)
				{
					if(rfid_a->last==1)
						{
							have_rfid=0;
							break;
						}
					else
						{
							rfid_a=rfid_a->next;
							continue;
						}
				}
			else
				{
					have_rfid=1;
					break;
				}
		}
	char *rfid_status;
	if(have_rfid==1)
		{
	if(rfid_a->status>=1)
		{
			rfid_status="WORKING";
		}
	else
		{
			rfid_status="OFFLINE";
		}
	//json_object *rfid1=json_object_new_object();
	json_object_object_add(rfid,"identify",json_object_new_string("RFID"));
	json_object_object_add(rfid,"status",json_object_new_string(rfid_status));
	//json_object_array_add(rfid,rfid1);
		}
		}

	//switch




	struct switch_json_header switch_jsonheader;
	switch_jsonheader.number=0;
	switch_jsonheader.next=NULL;
	if(wifi_switchheader.number>0)
		{
			struct wifi_switch *switch_a;
			struct switch_json *switch_json_a;
			struct switch_json *switch_json_b;
			int lamp_nu=0;
			
			char lamp_numb[5];
			
			switch_a=wifi_switchheader.next;
			while(1)
				{
					
					if(switch_a->del==1)
						{
							switch_a=switch_a->next;
							continue;
						}
					else
						{
							//switch_json_a=(struct switch_json *)malloc(sizeof(struct switch_json));
							while((switch_json_a=(struct switch_json *)malloc(sizeof(struct switch_json)))==NULL)
								{
									sleep(2);
								}
							switch_json_a->lamp_number=switch_a->lamp_number;
							switch_json_a->last=1;
							switch_json_a->switch_array=json_object_new_array();
							switch_json_a->switch_switch=json_object_new_object();
							//switch_json_a->lamps=(json_object **)malloc(switch_json_a->lamp_number*sizeof(json_object *));
							while((switch_json_a->lamps=(json_object **)malloc(switch_json_a->lamp_number*sizeof(json_object *)))==NULL)
								{
									sleep(2);
								}
							for(lamp_nu=0;lamp_nu<switch_json_a->lamp_number;lamp_nu++)
								{
									switch_json_a->lamps[lamp_nu]=json_object_new_object();
									memset(lamp_numb,0,5);
									sprintf(lamp_numb,"%d",(lamp_nu+1));
									json_object_object_add(switch_json_a->lamps[lamp_nu],"identify",json_object_new_string(lamp_numb));
									json_object_object_add(switch_json_a->lamps[lamp_nu],"status",json_object_new_string(switch_a->lamp[lamp_nu]));
									json_object_array_add(switch_json_a->switch_array,switch_json_a->lamps[lamp_nu]);
								}

							
							json_object_object_add(switch_json_a->switch_switch,"name",json_object_new_string(switch_a->name));
							json_object_object_add(switch_json_a->switch_switch,"mac",json_object_new_string(switch_a->mac));
							json_object_object_add(switch_json_a->switch_switch,"switch",switch_json_a->switch_array);
							json_object_array_add(switchn,switch_json_a->switch_switch);


							if(switch_jsonheader.number==0)
								{
									switch_jsonheader.next=switch_json_a;
									switch_json_b=switch_json_a;
									switch_jsonheader.number++;
								}
							else
								{
									switch_json_b->next=switch_json_a;
									switch_json_b->last=0;
									switch_json_b=switch_json_a;
									switch_jsonheader.number++;
								}
							if(switch_a->last==1)
								{
									break;
								}
							else
								{
									switch_a=switch_a->next;
								}
						}
				}
		}

	

	/*json_object *switch_array=json_object_new_array();
	json_object *switch_array1=json_object_new_array();
	json_object *switch_array2=json_object_new_array();

	json_object *switch1=json_object_new_object();
	json_object *lamp1_1=json_object_new_object();
	json_object *lamp1_2=json_object_new_object();

	json_object *switch2=json_object_new_object();
	json_object *lamp2_1=json_object_new_object();
	json_object *lamp2_2=json_object_new_object();

	char *lamp1_1_status,*lamp1_2_status,*lamp2_1_status,*lamp2_2_status;
	if(info.switch1_lamp1==1)
		{
			lamp1_1_status="WORKING";
		}
	else
		{
			lamp1_1_status="STANDBY";
		}
	
	if(info.switch1_lamp2==1)
		{
			lamp1_2_status="WORKING";
		}
	else
		{
			lamp1_2_status="STANDBY";
		}
	
	if(info.switch2_lamp1==1)
		{
			lamp2_1_status="WORKING";
		}
	else
		{
			lamp2_1_status="STANDBY";
		}
	
	if(info.switch2_lamp2==1)
		{
			lamp2_2_status="WORKING";
		}
	else
		{
			lamp2_2_status="STANDBY";
		}
	json_object_object_add(lamp1_1,"identify",json_object_new_string("1"));
	json_object_object_add(lamp1_1,"status",json_object_new_string(lamp1_1_status));
	json_object_object_add(lamp1_2,"identify",json_object_new_string("2"));
	json_object_object_add(lamp1_2,"status",json_object_new_string(lamp1_2_status));

	json_object_object_add(lamp2_1,"identify",json_object_new_string("1"));
	json_object_object_add(lamp2_1,"status",json_object_new_string(lamp2_1_status));
	json_object_object_add(lamp2_2,"identify",json_object_new_string("2"));
	json_object_object_add(lamp2_2,"status",json_object_new_string(lamp2_2_status));

	json_object_array_add(switch_array1,lamp1_1);
	json_object_array_add(switch_array1,lamp1_2);

	json_object_array_add(switch_array2,lamp2_1);
	json_object_array_add(switch_array2,lamp2_2);

	json_object_object_add(switch1,"name",json_object_new_string("switch1"));
	json_object_object_add(switch1,"mac",json_object_new_string(info.switch1_mac));
	json_object_object_add(switch1,"switch",switch_array1);

	json_object_object_add(switch2,"name",json_object_new_string("switch2"));
	json_object_object_add(switch2,"mac",json_object_new_string(info.switch2_mac));
	json_object_object_add(switch2,"switch",switch_array2);

	json_object_array_add(switchn,switch1);
	json_object_array_add(switchn,switch2);*/
	
 	int have_screen=0;
	//screen
	if(wifi_screenheader.number>0)
		{
	struct wifi_screen *screen_a;
	screen_a=wifi_screenheader.next;
	while(1)
		{
			if(screen_a->del==1)
				{
					if(screen_a->last==1)
						{
							have_screen=0;
							break;
						}
					else
						{
							screen_a=screen_a->next;
							continue;
						}
				}
			else
				{
					have_screen=1;
					break;
				}
		}
	
		
	char *screen_status;
	
	if(have_screen==1)
		{
	if(screen_a->status>=1)
		{
			screen_status="WORKING";
		}
	else
		{
			screen_status="OFFLINE";
		}
	json_object_object_add(screen,"identify",json_object_new_string("screen"));
	json_object_object_add(screen,"status",json_object_new_string(screen_status));
		}
		}

	//sunmary
	
	json_object_object_add(mainjson1,"result",json_object_new_int(1));
	json_object_object_add(mainjson1,"function",json_object_new_string("reportstatus"));
	json_object_object_add(mainjson1,"msg",json_object_new_string("chenggong"));
	json_object_object_add(mainjson1,"dtsName",json_object_new_string(base_info.name));
	json_object_object_add(mainjson1,"dtsHost",host);
	json_object_object_add(mainjson1,"dtsPC",pc);
	json_object_object_add(mainjson1,"dtsVolume",json_object_new_int(volume_3105.volume*2));
	json_object_object_add(mainjson1,"dtsChannel",json_object_new_int(device_status.ep-1));
	json_object_object_add(mainjson1,"dtsBooth",booth);
	json_object_object_add(mainjson1,"dtsDisplayer",display);
	json_object_object_add(mainjson1,"mediaaip",json_object_new_string(base_info.mediaa));
	json_object_object_add(mainjson1,"mediabip",json_object_new_string(base_info.mediab));
	if(have_sensor==1)
		{
	json_object_object_add(mainjson1,"dtsSensor",sensor);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsSensor",json_object_new_string("null"));
		}
	if(have_rfid==1)
		{
	json_object_object_add(mainjson1,"dtsRfid",rfid);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsRfid",json_object_new_string("null"));
		}
	if(switch_jsonheader.number>0)
		{
	json_object_object_add(mainjson1,"dtsSwitch",switchn);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsSwitch",json_object_new_string("null"));
		}
	if(have_screen==1)
		{
	json_object_object_add(mainjson1,"dtsScreen",screen);
		}
	else
		{
			json_object_object_add(mainjson1,"dtsScreen",json_object_new_string("null"));
		}

	//json_object_object_add(mainjson,"dtsinfo",mainjson1);
	memset(info_to_report,0,2048);
	sprintf(info_to_report,"%s",json_object_to_json_string(mainjson1));

	int lent;
	lent=strlen(info_to_report);
	char *requestrfid="{\"function\":\"requestrfidnotsend\", \"msg\":\"success\"}";
	int i=0;
	int j=0;
	printf("dtsinfo = %s\n%d\n",info_to_report,lent);
	if(nopoll_info.nopoll_isok_flag==1)
		{
			if(nopoll_conn_is_ready(nopoll_info.conn))
				{
					if(nopoll_info.request_rfidinfo<2)
						{
							//printf("requestrfidindo = %s\n",requestrfid);
							nopoll_conn_send_text(nopoll_info.conn,requestrfid,strlen(requestrfid));
							nopoll_info.request_rfidinfo++;
							sleep(1);
						}
					//printf("report status = %s\n",info_to_report);
					nopoll_conn_send_text(nopoll_info.conn,info_to_report,lent);					
				}
			else
				{
					nopoll_info.count_of_failure++;
				}
			if(nopoll_info.count_of_failure>5)
				{
					nopoll_loop_stop(nopoll_info.ctx);
				}
		}
	
	/*while(info_to_report[i]!='\0')
		{
			if(info_to_report[i]!=' ')
				{
					info_to_report[j]=info_to_report[i];
					j++;
					i++;
				}
			else
				{
					i++;
				}
		}
	info_to_report[j]='\0';
	lent=strlen(info_to_report);
	printf("dtsinfo = %s\n%d\n",info_to_report,lent);*/
	
	
	json_object_put(mainjson);
	json_object_put(mainjson1);
	json_object_put(host);
	json_object_put(pc);
	json_object_put(booth);
	json_object_put(display);
	json_object_put(sensor);
	json_object_put(rfid);
	json_object_put(switchn);
	json_object_put(screen);
	if(switch_jsonheader.number>0)
		{
			struct switch_json *a;
			int a1=0;
			while(1)
				{	
					a=switch_jsonheader.next;
					json_object_put(a->switch_switch);
					json_object_put(a->switch_array);
					for(a1=0;a1<a->lamp_number;a1++)
						{
							json_object_put(a->lamps[a1]);
						}
					free(a->lamps);
					if(a->last!=1)
						{
							switch_jsonheader.next=a->next;
							free(a);
						}
					else
						{
							free(a);
							break;
						}
					
				}
		}
	pthread_mutex_unlock(&nopoll_info.mutex);
	return 1;
}

void *report_to_server_thread(void)
{
	int ret=0;
	char *sendtext;
	while(1)
		{
			
			while(1)
				{
					report_fuction();
					ret=0;
					while(1)
						{
							sleep(1);
							
							if(nopoll_info.connect_report_flag==1)
								{
									nopoll_info.connect_report_flag=0;
									break;
								}
							ret++;
							if(ret>40)
								{
									break;
								}
						}
					
				}
		}
	return;


		
}



/****************************************************************************
播放计划任务函数及线程



******************************************************************************/

int init_schema_play(void)
{
	int k=44100;
	schema_play_info.sync=0;
	schema_play_info.stream_over_flag=0;
	schema_play_info.first_decode=0;
	schema_play_info.stream_over_flag_re=0;
	schema_play_info.first_save=0;
	schema_play_info.recive_buf_not_enough=0;
	schema_play_info.recive_size=0;
	schema_play_info.recivesizebigerr=0;
	schema_play_info.is_working=0;
	schema_play_info.first_run=0;



	schema_play_info.write_pointer=schema_play_info.recivebuf;
	schema_play_info.recive_pointer=schema_play_info.recivebuf;


	
	mad_stream_init(&schema_play_info.stream);
	mad_frame_init(&schema_play_info.frame);    
	mad_synth_init(&schema_play_info.synth);

	sem_init(&schema_play_info.wake_write,0,0);
	sem_init(&schema_play_info.schedule_play_wait_server,0,0);

	if(snd_pcm_open(&schema_play_info.handle,"dmix",SND_PCM_STREAM_PLAYBACK,0)<0)
		{
			printf("open alsa dmix false\n");
			
			return -1;
		}
	if(snd_pcm_hw_params_malloc(&schema_play_info.params)!=0)
		{
			snd_pcm_close(schema_play_info.handle);
			return -1;
		}
	if(snd_pcm_hw_params_any(schema_play_info.handle,schema_play_info.params)!=0)
		{
			snd_pcm_close(schema_play_info.handle);
			return -1;
		}
	if(snd_pcm_hw_params_set_access(schema_play_info.handle,schema_play_info.params,SND_PCM_ACCESS_RW_INTERLEAVED)!=0)
		{
			snd_pcm_close(schema_play_info.handle);
			return -1;
		}
	if(snd_pcm_hw_params_set_format(schema_play_info.handle,schema_play_info.params,SND_PCM_FORMAT_S16_LE)!=0)
		{
			snd_pcm_close(schema_play_info.handle);
			return -1;
		}
	if(snd_pcm_hw_params_set_rate_near(schema_play_info.handle,schema_play_info.params,&k,0)!=0)
		{
			snd_pcm_close(schema_play_info.handle);
			return -1;
		}
	if(snd_pcm_hw_params_set_channels(schema_play_info.handle,schema_play_info.params,2)!=0)
		{
			snd_pcm_close(schema_play_info.handle);
			return -1;
		}
	if(snd_pcm_hw_params_get_period_size(schema_play_info.params,&schema_play_info.frame_period_second,0)!=0)
		{
			return -1;
		}
	schema_play_info.sample=16;
	schema_play_info.byte_period_second=schema_play_info.sample*schema_play_info.frame_period_second*2/8;
	if(snd_pcm_hw_params(schema_play_info.handle,schema_play_info.params)!=0)
		{
			snd_pcm_close(schema_play_info.handle);
			return -1;
		}
	snd_pcm_hw_params_free(schema_play_info.params);
	return 1;
}
static size_t schema_libcurl_write_callback(void *ptr, size_t size, size_t nmemb, void *stream)
{
	int length=0;
	int this_size,next_size;
	long long int to_move=0;
	char *getstr;
	char *http_to_delete_buf;
	char *getstr1;
	
	
	length=size*nmemb;
	if(schema_play_info.is_working==0)
		{
			return -1;
		}
	if(schema_play_info.recivesizebigerr==1)
		{
			schema_play_info.recivesizebigerr=0;
			return length;
		}
	//printf("this is schema program length = %d\n",length);
	if(length!=0)
		{

			memset(schema_play_info.get_ptr,0,12800);
			memmove(schema_play_info.get_ptr,ptr,length);
			if((schema_play_info.recive_size+length)>=(40960*2))
				{
					schema_play_info.recivesizebigerr=1;
					printf("(schema_play_info.recive_size+length)>=(40960*2)\nlength = %d\n",length);
					return -1;	
				}
			else
				{
					schema_play_info.recive_size+=length;
					to_move=(40960*2)-(schema_play_info.recive_pointer-schema_play_info.recivebuf);
					if(to_move<length)
						{


							getstr=schema_play_info.get_ptr;
							this_size=to_move;
							next_size=length-to_move;
							memmove(schema_play_info.recive_pointer,getstr,this_size);
							getstr=getstr+this_size;
							memmove(schema_play_info.recivebuf,getstr,next_size);
							schema_play_info.recive_pointer=schema_play_info.recivebuf+next_size;
							
						}
					else
						{
							memmove(schema_play_info.recive_pointer,schema_play_info.get_ptr,length);
							schema_play_info.recive_pointer+=length;
							//recive_size+=length;
						}
				}
		}
		return length;
}

void *schema_recive_from_server(void)
{
	int res;
	char *request_schema_play_info="{\"function\":\"schedule\", \"operation\":\"schedule_requestinfo\"}";
	//schema_play_info.curl=curl_easy_init();
	schema_play_info.sync++;
	while(1)
		{
			if(schema_play_info.sync==2)
				{
					break;
				}
			usleep(100000);
		}
	sprintf(schema_play_info.libcurl_url,"http://%s:8128/schema.mp3",base_info.ip);
	//curl_global_cleanup();
	//curl_global_init(CURL_GLOBAL_ALL);
	schema_play_info.curl=curl_easy_init();
	curl_easy_setopt(schema_play_info.curl,CURLOPT_CONNECTTIMEOUT,3);
	curl_easy_setopt(schema_play_info.curl,CURLOPT_LOW_SPEED_LIMIT,200);
	curl_easy_setopt(schema_play_info.curl,CURLOPT_LOW_SPEED_TIME,2);
	curl_easy_setopt(schema_play_info.curl,CURLOPT_WRITEFUNCTION,schema_libcurl_write_callback);
	curl_easy_setopt(schema_play_info.curl,CURLOPT_WRITEDATA,schema_play_info.recivebuf);
	while(1)
		{
			//printf("schema_play_info.libcurl_url = %s\n",schema_play_info.libcurl_url);

			while(nopoll_info.nopoll_isok_flag==0)
				{
					//printf("curl wait nopoll is ok\n");
					sleep(1);
				}
			if(nopoll_info.nopoll_isok_flag==1)
				{
					schema_play_info.first_run=1;
					if(nopoll_conn_is_ready(nopoll_info.conn))
						{
									
							if(nopoll_conn_send_text(nopoll_info.conn,request_schema_play_info,strlen(request_schema_play_info))!=strlen(request_schema_play_info))
								{
									printf("controll device nopoll send false\n");
											
								}
							//a->already_send_broadcast++;
						}
				}
			while(schema_play_info.is_working==0)
				{
					//printf("curl wait nopoll is ok\n");
					sleep(1);
				}
			//sem_wait(&schema_play_info.schedule_play_wait_server);
			sprintf(schema_play_info.libcurl_url,"http://%s:8128/schema.mp3",base_info.ip);
			if(curl_easy_setopt(schema_play_info.curl,CURLOPT_URL,schema_play_info.libcurl_url)==CURLE_OK)
				{
					printf("all curl set is ok\n");
					res = curl_easy_perform(schema_play_info.curl);
					printf("now reset curl nopoll_info.nopoll_isok_flag =%d\n",nopoll_info.nopoll_isok_flag);
					schema_play_info.stream_over_flag=1;
					
				}
			/*if((schema_play_info.curl=curl_easy_init())!=NULL)
				{
					sprintf(schema_play_info.libcurl_url,"http://%s:8127/test.mp3",base_info.ip);
					if(curl_easy_setopt(schema_play_info.curl,CURLOPT_URL,schema_play_info.libcurl_url)==CURLE_OK)
						{
							if(curl_easy_setopt(schema_play_info.curl,CURLOPT_CONNECTTIMEOUT,3)==CURLE_OK)
								{
									if(curl_easy_setopt(schema_play_info.curl,CURLOPT_WRITEFUNCTION,schema_libcurl_write_callback)==CURLE_OK)
										{
											if(curl_easy_setopt(schema_play_info.curl,CURLOPT_WRITEDATA,schema_play_info.recivebuf)==CURLE_OK)
												{
													printf("all curl set is ok\n");
													res = curl_easy_perform(schema_play_info.curl);
													printf("now reset curl nopoll_info.nopoll_isok_flag =%d\n",nopoll_info.nopoll_isok_flag);
													schema_play_info.stream_over_flag=1;
													schema_play_info.write_pointer=schema_play_info.recivebuf;
													schema_play_info.recive_pointer=schema_play_info.recivebuf;
													schema_play_info.recive_size=0;
												}
											else
												{
													printf("CURLOPT_WRITEDATA false\n");
												}
										}
									else
										{
											printf("CURLOPT_WRITEFUNCTION false\n");
										}
								}
							else
								{
									printf("CURLOPT_CONNECTTIMEOUT false\n");
								}
						}
					else
						{
							printf("CURLOPT_URL false\n");
						}
					//printf("now cleanup curl\n");
					//curl_easy_cleanup(schema_play_info.curl);
					//printf("cleanup curl false");
				}
				else
					{
						printf("curl_easy_init false\n");
					}*/
			//sleep(1);
			
			schema_play_info.is_working=0;
			sleep(1);


			
			/*schema_play_info.curl=curl_easy_init();
			sprintf(schema_play_info.libcurl_url,"http://%s:8127/test.mp3",base_info.ip);
			printf("schema_play_info.libcurl_url = %s\n",schema_play_info.libcurl_url);
			curl_easy_setopt(schema_play_info.curl,CURLOPT_URL,schema_play_info.libcurl_url);
			curl_easy_setopt(schema_play_info.curl,CURLOPT_CONNECTTIMEOUT,5);
			curl_easy_setopt(schema_play_info.curl,CURLOPT_WRITEFUNCTION,schema_libcurl_write_callback);
			curl_easy_setopt(schema_play_info.curl,CURLOPT_WRITEDATA,schema_play_info.recivebuf);
			res = curl_easy_perform(schema_play_info.curl);
			sleep(1);
			printf("now reset player\n");
			schema_play_info.stream_over_flag=1;
			schema_play_info.write_pointer=schema_play_info.recivebuf;
			schema_play_info.recive_pointer=schema_play_info.recivebuf;
			schema_play_info.recive_size=0;
			//curl_easy_reset(schema_play_info.curl);
			curl_easy_cleanup(schema_play_info.curl);
			//sleep(1);*/
		}
	//curl_easy_cleanup(schema_play_info.curl);
	return;
}


signed short to_short(mad_fixed_t fixed)
{
    /*if(fixed>=MAD_F_ONE)
        return(0);
    if(fixed<=-MAD_F_ONE)
        return(0);
    fixed=fixed>>(MAD_F_FRACBITS-15);
    return((signed short)fixed);*/
    fixed += (1L << (MAD_F_FRACBITS - 16));
    if (fixed >= MAD_F_ONE)
       fixed = MAD_F_ONE - 1;
    else if (fixed < -MAD_F_ONE)
         fixed = -MAD_F_ONE;
    return fixed >> (MAD_F_FRACBITS + 1 - 16);
}


int setup()
    {
        size_t    remaining=0;
		long long int last_size;
		int this_size;
		int next_size;
		int marginal=4096;
		size_t reading=0;
		size_t remain=0;
		size_t to_mad_stream_length=0;
        unsigned char *read1;
		last_size=(40960*2)-(schema_play_info.write_pointer-schema_play_info.recivebuf);
		if(schema_play_info.first_decode==0)
			{
				read1=schema_play_info.libmad_mp3_buf;
			}
		else if(schema_play_info.stream.next_frame!=NULL)
			{
				remaining=schema_play_info.stream.bufend-schema_play_info.stream.next_frame;//剩下的未解码的大小
				memmove(schema_play_info.libmad_mp3_buf,schema_play_info.stream.next_frame,remaining);//将未解码部分移入缓冲区开头
				read1=schema_play_info.libmad_mp3_buf+remaining;
				//printf("remain data\n");
			}
		else
			{
				read1=schema_play_info.libmad_mp3_buf;
			}
		
		to_mad_stream_length=4096-remaining;

		while(schema_play_info.recive_size<marginal)
			{
				
				if(schema_play_info.stream_over_flag==1)
					{
						schema_play_info.stream_over_flag=0;
						schema_play_info.write_pointer=schema_play_info.recivebuf;
						schema_play_info.recive_pointer=schema_play_info.recivebuf;
						schema_play_info.recive_size=0;
						printf("revice over in write while\n");
						return -1;
					}
				marginal=12488;
				//printf("in write schema_play_info.recive_size=%d\n",schema_play_info.recive_size);
				usleep(100000);
			}
		marginal=4096;
		if(last_size<to_mad_stream_length)
			{
				this_size=last_size;
				next_size=to_mad_stream_length-last_size;
				memmove(read1,schema_play_info.write_pointer,this_size);
				//libmad_buf=http_recive_buf;
				read1=read1+this_size;
				memmove(read1,schema_play_info.recivebuf,next_size);
				schema_play_info.write_pointer=schema_play_info.recivebuf+next_size;
				schema_play_info.recive_size-=to_mad_stream_length;
			}
		else
			{
				memmove(read1,schema_play_info.write_pointer,to_mad_stream_length);
				schema_play_info.write_pointer+=to_mad_stream_length;
				schema_play_info.recive_size-=to_mad_stream_length;
			}
        mad_stream_buffer(&schema_play_info.stream,schema_play_info.libmad_mp3_buf,4096);
		schema_play_info.first_decode=1;
        char dest[100];
       // mad_timer_string(timer, dest, "lu:u:u", MAD_UNITS_HOURS, MAD_UNITS_MILLISECONDS, 0);
        schema_play_info.stream.error=0;
        return 0;        
    }

long readFrame(unsigned char* pcm)
    {
    	int k;
        if(schema_play_info.stream.buffer == NULL)
        {
        	while(1)
            		{
            			k=setup();
                		if (k == -1) 
							{
								return -1;
                			}
						else if(k==0)
							{
								break;
							}
            		}
           //if(setup() == -1) 
              // return -1;
        }
    while(mad_frame_decode(&schema_play_info.frame,&schema_play_info.stream) != 0)
    {
        if(MAD_RECOVERABLE(schema_play_info.stream.error))
        {
            if(schema_play_info.stream.error!=MAD_ERROR_LOSTSYNC)
            {
            }
            continue;
        }
        else
            if(schema_play_info.stream.error==MAD_ERROR_BUFLEN)
            { 
            	while(1)
            		{
            			k=setup();
                		if (k == -1) 
							{
								return -1;
                			}
						else if(k==0)
							{
								break;
							}
            		}
                	continue;
            }
            else
            {
                return -1;
				//continue;
            }
    }
    mad_synth_frame(&schema_play_info.synth,&schema_play_info.frame);

    //! 获取频率
    //freq = synth.pcm.samplerate; 
    
    //bitrate = frame.header.bitrate;
    //channels = (frame.header.mode == MAD_MODE_SINGLE_CHANNEL) ? 1 : 2;
    
    //mad_timer_add(&timer,frame.header.duration);
    int j = 0;
	int i=0;
 
    for(i=0;i<schema_play_info.synth.pcm.length;i++)
    {
        signed short    sample;
        sample=to_short(schema_play_info.synth.pcm.samples[0][i]);
        pcm[j++] = sample&0xff;
        pcm[j++] = sample>>8;
        if(MAD_NCHANNELS(&schema_play_info.frame.header)==2)
            sample=to_short(schema_play_info.synth.pcm.samples[1][i]);
        pcm[j++] = sample&0xff;
        pcm[j++] = sample>>8;
    }

    char dest[120];
   // mad_timer_string(timer,dest, "%lu:lu.u", MAD_UNITS_MINUTES, MAD_UNITS_MILLISECONDS, 0);
    return j;        
}

void *schema_write_to_pcm(void)
{
	unsigned char buffer1[4096*2]={0};
	int len;
	int r;
	schema_play_info.sync++;
	while(1)
		{
			if(schema_play_info.sync==2)
				{
					break;
				}
			usleep(100000);
		}
	while(1)
		{
			
			while(schema_play_info.recive_size<12488)
			{
				if(schema_play_info.stream_over_flag==1)
					{
						printf("revice over\n");
						schema_play_info.write_pointer=schema_play_info.recivebuf;
						schema_play_info.recive_pointer=schema_play_info.recivebuf;
						schema_play_info.recive_size=0;
						break;
					}
				usleep(100000);
			}
			schema_play_info.first_decode=0;
			printf("schema_play_info.recive_size = %d\n",schema_play_info.recive_size);
			while((len =readFrame(buffer1))>0)
   			 {

				r=snd_pcm_writei(schema_play_info.handle,(char*)buffer1,(len/4));
				if(!(r>0))
					{
						snd_pcm_prepare(schema_play_info.handle);
						printf("schema_play_info  snd_pcm_prepare--------- r = %d\n",r);
					}
   			 }
			snd_pcm_prepare(schema_play_info.handle);
		}
	snd_pcm_close(schema_play_info.handle);
	return;
}


/****************************************************************************
播放广播函数及线程



******************************************************************************/

int init_broadcast_mpe_play(void)
{
	int k=44100;
	broadcast_play_mp3_info.sync=0;
	broadcast_play_mp3_info.stream_over_flag=0;
	broadcast_play_mp3_info.first_decode=0;
	broadcast_play_mp3_info.stream_over_flag_re=0;
	broadcast_play_mp3_info.first_save=0;
	broadcast_play_mp3_info.recive_buf_not_enough=0;
	broadcast_play_mp3_info.recive_size=0;
	broadcast_play_mp3_info.recivesizebigerr=0;
	broadcast_play_mp3_info.is_working=0;
	broadcast_play_mp3_info.first_run=0;





	broadcast_play_mp3_info.write_pointer=broadcast_play_mp3_info.recivebuf;
	broadcast_play_mp3_info.recive_pointer=broadcast_play_mp3_info.recivebuf;


	
	mad_stream_init(&broadcast_play_mp3_info.stream);
	mad_frame_init(&broadcast_play_mp3_info.frame);    
	mad_synth_init(&broadcast_play_mp3_info.synth);

	sem_init(&broadcast_play_mp3_info.wake_write,0,0);
	sem_init(&broadcast_play_mp3_info.broadcast_play_mp3_wait_server,0,0);

	if(snd_pcm_open(&broadcast_play_mp3_info.handle,"dmix",SND_PCM_STREAM_PLAYBACK,0)<0)
		{
			printf("open alsa dmix false\n");
			
			return -1;
		}
	if(snd_pcm_hw_params_malloc(&broadcast_play_mp3_info.params)!=0)
		{
			snd_pcm_close(broadcast_play_mp3_info.handle);
			return -1;
		}
	if(snd_pcm_hw_params_any(broadcast_play_mp3_info.handle,broadcast_play_mp3_info.params)!=0)
		{
			snd_pcm_close(broadcast_play_mp3_info.handle);
			return -1;
		}
	if(snd_pcm_hw_params_set_access(broadcast_play_mp3_info.handle,broadcast_play_mp3_info.params,SND_PCM_ACCESS_RW_INTERLEAVED)!=0)
		{
			snd_pcm_close(broadcast_play_mp3_info.handle);
			return -1;
		}
	if(snd_pcm_hw_params_set_format(broadcast_play_mp3_info.handle,broadcast_play_mp3_info.params,SND_PCM_FORMAT_S16_LE)!=0)
		{
			snd_pcm_close(broadcast_play_mp3_info.handle);
			return -1;
		}
	if(snd_pcm_hw_params_set_rate_near(broadcast_play_mp3_info.handle,broadcast_play_mp3_info.params,&k,0)!=0)
		{
			snd_pcm_close(broadcast_play_mp3_info.handle);
			return -1;
		}
	if(snd_pcm_hw_params_set_channels(broadcast_play_mp3_info.handle,broadcast_play_mp3_info.params,2)!=0)
		{
			snd_pcm_close(broadcast_play_mp3_info.handle);
			return -1;
		}
	if(snd_pcm_hw_params_get_period_size(broadcast_play_mp3_info.params,&broadcast_play_mp3_info.frame_period_second,0)!=0)
		{
			return -1;
		}
	broadcast_play_mp3_info.sample=16;
	broadcast_play_mp3_info.byte_period_second=broadcast_play_mp3_info.sample*broadcast_play_mp3_info.frame_period_second*2/8;
	if(snd_pcm_hw_params(broadcast_play_mp3_info.handle,broadcast_play_mp3_info.params)!=0)
		{
			snd_pcm_close(broadcast_play_mp3_info.handle);
			return -1;
		}
	snd_pcm_hw_params_free(broadcast_play_mp3_info.params);
	return 1;
}
static size_t broadcast_play_mp3_libcurl_write_callback(void *ptr, size_t size, size_t nmemb, void *stream)
{
	int length=0;
	int this_size,next_size;
	long long int to_move=0;
	char *getstr;
	char *http_to_delete_buf;
	char *getstr1;
	//broadcast_play_mp3_info.is_working=1
	
	length=size*nmemb;
	//printf("this is broadcast program length = %d\n",length);
	//broadcast_play_mp3_info.recivesizebigerr
	if(broadcast_play_mp3_info.is_working==0)
		{
			return -1;
		}
	if(broadcast_play_mp3_info.recivesizebigerr==1)
		{
			broadcast_play_mp3_info.recivesizebigerr=0;
			return length;
		}
	if(length!=0)
		{

			memset(broadcast_play_mp3_info.get_ptr,0,12800);
			memmove(broadcast_play_mp3_info.get_ptr,ptr,length);
			if((broadcast_play_mp3_info.recive_size+length)>=(40960*2))
				{
					broadcast_play_mp3_info.recivesizebigerr=1;
					printf("(schema_play_info.recive_size+length)>=(40960*2)\nlength = %d\n",length);
					return -1;	
				}
			else
				{
					broadcast_play_mp3_info.recive_size+=length;
					to_move=(40960*2)-(broadcast_play_mp3_info.recive_pointer-broadcast_play_mp3_info.recivebuf);
					if(to_move<length)
						{


							getstr=broadcast_play_mp3_info.get_ptr;
							this_size=to_move;
							next_size=length-to_move;
							memmove(broadcast_play_mp3_info.recive_pointer,getstr,this_size);
							getstr=getstr+this_size;
							memmove(broadcast_play_mp3_info.recivebuf,getstr,next_size);
							broadcast_play_mp3_info.recive_pointer=broadcast_play_mp3_info.recivebuf+next_size;
							
						}
					else
						{
							memmove(broadcast_play_mp3_info.recive_pointer,broadcast_play_mp3_info.get_ptr,length);
							broadcast_play_mp3_info.recive_pointer+=length;
							//recive_size+=length;
						}
				}
		}
		return length;
}

void *broadcast_play_mp3_recive_from_server(void)
{
	int res;
	char *request_broadcast_play_info="{\"function\":\"broadcastmp3\", \"operation\":\"broadcastmp3_requestinfo\"}";
	//schema_play_info.curl=curl_easy_init();
	broadcast_play_mp3_info.sync++;
	while(1)
		{
			if(broadcast_play_mp3_info.sync==2)
				{
					break;
				}
			usleep(100000);
		}
	sprintf(broadcast_play_mp3_info.libcurl_url,"http://%s:8128/broadcast.mp3",base_info.ip);
	//curl_global_cleanup();
	//curl_global_init(CURL_GLOBAL_ALL);
	broadcast_play_mp3_info.curl=curl_easy_init();
	curl_easy_setopt(broadcast_play_mp3_info.curl,CURLOPT_CONNECTTIMEOUT,3);
	curl_easy_setopt(broadcast_play_mp3_info.curl,CURLOPT_LOW_SPEED_LIMIT,200);
	curl_easy_setopt(broadcast_play_mp3_info.curl,CURLOPT_LOW_SPEED_TIME,2);
	curl_easy_setopt(broadcast_play_mp3_info.curl,CURLOPT_WRITEFUNCTION,broadcast_play_mp3_libcurl_write_callback);
	curl_easy_setopt(broadcast_play_mp3_info.curl,CURLOPT_WRITEDATA,broadcast_play_mp3_info.recivebuf);
	while(1)
		{
			//printf("schema_play_info.libcurl_url = %s\n",schema_play_info.libcurl_url);

			while(nopoll_info.nopoll_isok_flag==0)
				{
					//printf("curl wait nopoll is ok\n");
					sleep(1);
				}
			if(nopoll_info.nopoll_isok_flag==1)
				{
					broadcast_play_mp3_info.first_run=1;
					if(nopoll_conn_is_ready(nopoll_info.conn))
						{
							printf("request broadcast info\n");
							if(nopoll_conn_send_text(nopoll_info.conn,request_broadcast_play_info,strlen(request_broadcast_play_info))!=strlen(request_broadcast_play_info))
								{
									printf("controll device nopoll send false\n");
											
								}
							//a->already_send_broadcast++;
						}
				}
			while(broadcast_play_mp3_info.is_working==0)
				{
					//printf("curl wait nopoll is ok\n");
					sleep(1);
				}
			//printf("broadcast wait signel\n");
			//sem_wait(&broadcast_play_mp3_info.broadcast_play_mp3_wait_server);
			//broadcast_play_mp3_info.is_working=1;
			sprintf(broadcast_play_mp3_info.libcurl_url,"http://%s:8128/broadcast.mp3",base_info.ip);
			if(curl_easy_setopt(broadcast_play_mp3_info.curl,CURLOPT_URL,broadcast_play_mp3_info.libcurl_url)==CURLE_OK)
				{
					printf("all curl broadcast set is ok\n");
					res = curl_easy_perform(broadcast_play_mp3_info.curl);
					printf("now reset broadcast curl nopoll_info.nopoll_isok_flag =%d\n",nopoll_info.nopoll_isok_flag);
					//sleep(2);
					broadcast_play_mp3_info.stream_over_flag=1;
				}
			/*if((schema_play_info.curl=curl_easy_init())!=NULL)
				{
					sprintf(schema_play_info.libcurl_url,"http://%s:8127/test.mp3",base_info.ip);
					if(curl_easy_setopt(schema_play_info.curl,CURLOPT_URL,schema_play_info.libcurl_url)==CURLE_OK)
						{
							if(curl_easy_setopt(schema_play_info.curl,CURLOPT_CONNECTTIMEOUT,3)==CURLE_OK)
								{
									if(curl_easy_setopt(schema_play_info.curl,CURLOPT_WRITEFUNCTION,schema_libcurl_write_callback)==CURLE_OK)
										{
											if(curl_easy_setopt(schema_play_info.curl,CURLOPT_WRITEDATA,schema_play_info.recivebuf)==CURLE_OK)
												{
													printf("all curl set is ok\n");
													res = curl_easy_perform(schema_play_info.curl);
													printf("now reset curl nopoll_info.nopoll_isok_flag =%d\n",nopoll_info.nopoll_isok_flag);
													schema_play_info.stream_over_flag=1;
													schema_play_info.write_pointer=schema_play_info.recivebuf;
													schema_play_info.recive_pointer=schema_play_info.recivebuf;
													schema_play_info.recive_size=0;
												}
											else
												{
													printf("CURLOPT_WRITEDATA false\n");
												}
										}
									else
										{
											printf("CURLOPT_WRITEFUNCTION false\n");
										}
								}
							else
								{
									printf("CURLOPT_CONNECTTIMEOUT false\n");
								}
						}
					else
						{
							printf("CURLOPT_URL false\n");
						}
					//printf("now cleanup curl\n");
					//curl_easy_cleanup(schema_play_info.curl);
					//printf("cleanup curl false");
				}
				else
					{
						printf("curl_easy_init false\n");
					}*/
			//sleep(1);
			printf("curl_broadcast perform res = %d\n",res);
			sleep(1);
			broadcast_play_mp3_info.is_working=0;
			//broadcast_play_mp3_info.is_working=0;


			
			/*schema_play_info.curl=curl_easy_init();
			sprintf(schema_play_info.libcurl_url,"http://%s:8127/test.mp3",base_info.ip);
			printf("schema_play_info.libcurl_url = %s\n",schema_play_info.libcurl_url);
			curl_easy_setopt(schema_play_info.curl,CURLOPT_URL,schema_play_info.libcurl_url);
			curl_easy_setopt(schema_play_info.curl,CURLOPT_CONNECTTIMEOUT,5);
			curl_easy_setopt(schema_play_info.curl,CURLOPT_WRITEFUNCTION,schema_libcurl_write_callback);
			curl_easy_setopt(schema_play_info.curl,CURLOPT_WRITEDATA,schema_play_info.recivebuf);
			res = curl_easy_perform(schema_play_info.curl);
			sleep(1);
			printf("now reset player\n");
			schema_play_info.stream_over_flag=1;
			schema_play_info.write_pointer=schema_play_info.recivebuf;
			schema_play_info.recive_pointer=schema_play_info.recivebuf;
			schema_play_info.recive_size=0;
			//curl_easy_reset(schema_play_info.curl);
			curl_easy_cleanup(schema_play_info.curl);
			//sleep(1);*/
		}
	//curl_easy_cleanup(schema_play_info.curl);
	return;
}


signed short broadcast_play_mp3_to_short(mad_fixed_t fixed)
{
    /*if(fixed>=MAD_F_ONE)
        return(0);
    if(fixed<=-MAD_F_ONE)
        return(0);
    fixed=fixed>>(MAD_F_FRACBITS-15);
    return((signed short)fixed);*/
    fixed += (1L << (MAD_F_FRACBITS - 16));
    if (fixed >= MAD_F_ONE)
       fixed = MAD_F_ONE - 1;
    else if (fixed < -MAD_F_ONE)
         fixed = -MAD_F_ONE;
    return fixed >> (MAD_F_FRACBITS + 1 - 16);
}


int broadcast_play_mp3_setup()
    {
        size_t    remaining=0;
		long long int last_size;
		int this_size;
		int next_size;
		int marginal=4096;
		size_t reading=0;
		size_t remain=0;
		size_t to_mad_stream_length=0;
        unsigned char *read1;
		last_size=(40960*2)-(broadcast_play_mp3_info.write_pointer-broadcast_play_mp3_info.recivebuf);
		if(broadcast_play_mp3_info.first_decode==0)
			{
				read1=broadcast_play_mp3_info.libmad_mp3_buf;
			}
		else if(broadcast_play_mp3_info.stream.next_frame!=NULL)
			{
				remaining=broadcast_play_mp3_info.stream.bufend-broadcast_play_mp3_info.stream.next_frame;//剩下的未解码的大小
				memmove(broadcast_play_mp3_info.libmad_mp3_buf,broadcast_play_mp3_info.stream.next_frame,remaining);//将未解码部分移入缓冲区开头
				read1=broadcast_play_mp3_info.libmad_mp3_buf+remaining;
				//printf("remain data\n");
			}
		else
			{
				read1=broadcast_play_mp3_info.libmad_mp3_buf;
			}
		
		to_mad_stream_length=4096-remaining;
		//printf("in write schema_play_info.recive_size=%d\n",broadcast_play_mp3_info.recive_size);
		while(broadcast_play_mp3_info.recive_size<marginal)
			{
				
				if(broadcast_play_mp3_info.stream_over_flag==1)
					{
						broadcast_play_mp3_info.stream_over_flag=0;
						broadcast_play_mp3_info.write_pointer=broadcast_play_mp3_info.recivebuf;
						broadcast_play_mp3_info.recive_pointer=broadcast_play_mp3_info.recivebuf;
						broadcast_play_mp3_info.recive_size=0;	
						printf("revice over in write while\n");
						return -1;
					}
				
				marginal=12488;
				printf("int while in write schema_play_info.recive_size=%d\n",broadcast_play_mp3_info.recive_size);
				usleep(100000);
			}
		marginal=4096;
		if(last_size<to_mad_stream_length)
			{
				this_size=last_size;
				next_size=to_mad_stream_length-last_size;
				memmove(read1,broadcast_play_mp3_info.write_pointer,this_size);
				//libmad_buf=http_recive_buf;
				read1=read1+this_size;
				memmove(read1,broadcast_play_mp3_info.recivebuf,next_size);
				broadcast_play_mp3_info.write_pointer=broadcast_play_mp3_info.recivebuf+next_size;
				broadcast_play_mp3_info.recive_size-=to_mad_stream_length;
			}
		else
			{
				memmove(read1,broadcast_play_mp3_info.write_pointer,to_mad_stream_length);
				broadcast_play_mp3_info.write_pointer+=to_mad_stream_length;
				broadcast_play_mp3_info.recive_size-=to_mad_stream_length;
			}
        mad_stream_buffer(&broadcast_play_mp3_info.stream,broadcast_play_mp3_info.libmad_mp3_buf,4096);
		broadcast_play_mp3_info.first_decode=1;
        char dest[100];
       // mad_timer_string(timer, dest, "lu:u:u", MAD_UNITS_HOURS, MAD_UNITS_MILLISECONDS, 0);
        broadcast_play_mp3_info.stream.error=0;
        return 0;        
    }

long broadcast_play_mp3_readFrame(unsigned char* pcm)
    {
    	int k;
        if(broadcast_play_mp3_info.stream.buffer == NULL)
        {
        	while(1)
            		{
            			k=broadcast_play_mp3_setup();
                		if (k == -1) 
							{
								return -1;
                			}
						else if(k==0)
							{
								break;
							}
            		}
           //if(setup() == -1) 
              // return -1;
        }
    while(mad_frame_decode(&broadcast_play_mp3_info.frame,&broadcast_play_mp3_info.stream) != 0)
    {
        if(MAD_RECOVERABLE(broadcast_play_mp3_info.stream.error))
        {
            if(broadcast_play_mp3_info.stream.error!=MAD_ERROR_LOSTSYNC)
            {
            }
            continue;
        }
        else
            if(broadcast_play_mp3_info.stream.error==MAD_ERROR_BUFLEN)
            { 
            	while(1)
            		{
            			k=broadcast_play_mp3_setup();
                		if (k == -1) 
							{
								return -1;
                			}
						else if(k==0)
							{
								break;
							}
            		}
                	continue;
            }
            else
            {
                return -1;
				//continue;
            }
    }
    mad_synth_frame(&broadcast_play_mp3_info.synth,&broadcast_play_mp3_info.frame);

    //! 获取频率
    //freq = synth.pcm.samplerate; 
    
    //bitrate = frame.header.bitrate;
    //channels = (frame.header.mode == MAD_MODE_SINGLE_CHANNEL) ? 1 : 2;
    
    //mad_timer_add(&timer,frame.header.duration);
    int j = 0;
	int i=0;
 
    for(i=0;i<broadcast_play_mp3_info.synth.pcm.length;i++)
    {
        signed short    sample;
        sample=broadcast_play_mp3_to_short(broadcast_play_mp3_info.synth.pcm.samples[0][i]);
        pcm[j++] = sample&0xff;
        pcm[j++] = sample>>8;
        if(MAD_NCHANNELS(&broadcast_play_mp3_info.frame.header)==2)
            sample=broadcast_play_mp3_to_short(broadcast_play_mp3_info.synth.pcm.samples[1][i]);
        pcm[j++] = sample&0xff;
        pcm[j++] = sample>>8;
    }

    char dest[120];
   // mad_timer_string(timer,dest, "%lu:lu.u", MAD_UNITS_MINUTES, MAD_UNITS_MILLISECONDS, 0);
    return j;        
}

void *broadcast_play_mp3_write_to_pcm(void)
{
	unsigned char buffer1[4096*2]={0};
	int len;
	int r;
	broadcast_play_mp3_info.sync++;
	while(1)
		{
			if(broadcast_play_mp3_info.sync==2)
				{
					break;
				}
			usleep(100000);
		}
	while(1)
		{
			
			while(broadcast_play_mp3_info.recive_size<12488)
			{
				if(broadcast_play_mp3_info.stream_over_flag==1)
					{
						printf("revice over\n");
						broadcast_play_mp3_info.write_pointer=broadcast_play_mp3_info.recivebuf;
						broadcast_play_mp3_info.recive_pointer=broadcast_play_mp3_info.recivebuf;
						broadcast_play_mp3_info.recive_size=0;	
						break;
					}
				usleep(100000);
			}
			broadcast_play_mp3_info.first_decode=0;
			printf("schema_play_info.recive_size = %d\n",broadcast_play_mp3_info.recive_size);
			while((len =broadcast_play_mp3_readFrame(buffer1))>0)
   			 {

				r=snd_pcm_writei(broadcast_play_mp3_info.handle,(char*)buffer1,(len/4));
				//printf("len = %d       r = %d\n",len,r);
				if(!(r>0))
					{
						snd_pcm_prepare(broadcast_play_mp3_info.handle);
						printf("broadcast_play_mp3_info  snd_pcm_prepare--------- r = %d\n",r);
						//printf("Write error: %s\n", snd_strerror(r));
					}
				//sleep(1);
   			 }
			snd_pcm_prepare(broadcast_play_mp3_info.handle);
		}
	snd_pcm_close(broadcast_play_mp3_info.handle);
	return;
}

//电脑检测线程
int pc_det_init(void)
{
	if((pc_det.fd=open("/dev/int9",O_RDWR))<0)
		{
			printf("open pc_det false\n");
			return -1;
		}
	return 1;
}
void * pc_det_thread(void *ptr)
{
	char pc_status[2];
	while(1)
		{
			read(pc_det.fd,pc_status,2);
			if(pc_status[0]==0)
				{
					device_status.pc=1;
					
				}
			else if(pc_status[0]==1)
				{
					device_status.pc=0;
				}
			//printf("pc_status =  %02x   %02x\n",pc_status[0],pc_status[1]);
			//printf("get pc status = %d\n",device_status.pc);
			sleep(1);
		}
	close(pc_det.fd);
	return;
}
void * output_run_pulse(void *ptr)
{
	unsigned char status[2];
	unsigned char pc_status[2];
	status[0]=0;
	status[1]=0;
	while(1)
		{
			
			if(status[1]==0)
				{
					status[1]=1;
				}
			else
				{
					status[1]=0;
				}
			write(pc_det.fd,status,2);
			//printf("pc_status =  %02x   %02x\n",pc_status[0],pc_status[1]);
			//printf("  status[1] = %d\n",status[1]);
			//read(pc_det.fd,pc_status,2);
			

			//printf("pc_status[0]   = %d\n",pc_status[0]);
			usleep(50000);
		}
	close(pc_det.fd);
	return;
}


/*************************************************************************************
7628与mcu 通信用到的函数


**************************************************************************************/






int read_lisence_from_mcu(unsigned char *getbuf)
{
	unsigned char buf[9][133];
	unsigned char *getbuff;
	unsigned char readbuf;
	unsigned char check=0;
	unsigned char block_number;
	unsigned int data_register=0;
	int flag=0;
	int i;
	int j;
	while(1)
		{
			
			for(i=0;i<8;i++)
				{
					
					block_number=0xbb+i;
					while(1)
						{
							check=0;
							flag=0;
							//printf("now write 0xbb to block_operation_register\n");
							i2c_write(i2c_address_mcu,mcu_i2c_info.block_operation_register,block_number);
							usleep(10000);
							//printf("write 0xbb to block_operation_register success\n");
							for(j=0;j<132;j++)
								{
									data_register=mcu_i2c_info.block_read_write_register+j;
									i2c_read(i2c_address_mcu,data_register,&buf[i][j]);
									//usleep(10000);
									printf("in read data   buf[%d][%d] =  %02x\n",i,j,buf[i][j]);
									if(j<131)
										{
											check+=buf[i][j];
											
										}
									
									if(j==131)
										{
											printf("check =%02x  buf[%d][%d] = %02x\n",check,i,j,buf[i][j]);
											if(check!=buf[i][j])
												{
													flag=1;//表示错误，重新读本快
													printf("read check error\n");
												}
											
										}
									
									/*if(flag==0)
										{
											break;
										}
									else
										{
											flag=0;
										}*/
								}
							//sleep(10000);
							j=0;
							while(1)
								{
									j++;
									if(j>=4)
										{
											i2c_write(i2c_address_mcu,mcu_i2c_info.block_operation_register,0);
											usleep(10000);
											printf("read status is not 0\n");
											break;
										}
									i2c_read(i2c_address_mcu,mcu_i2c_info.block_operation_register,&readbuf);
									usleep(10000);
									if(readbuf==0)
										{
											//flag=0;//写正确，不用重新写
											printf("read status right\n");
											break;
										}
									
									
								}
							if((flag==0)&&(j<4))
								{
									break;
								}
							printf("now to read lisence again\n");
						}
					
					//sleep(1);
				}
			break;
		}
	getbuff=getbuf;
	for(i=0;i<8;i++)
		{
			memmove(getbuff,&buf[i][3],128);
			if(i<7)
				{
					getbuff+=128;
				}
		}
	return 1;
}



int send_lisence_to_mcu(char *lisencedata)
{
	unsigned char senddata[133]={0};
	unsigned char readbuf;
	unsigned int data_register=0;
	int i;
	int j;
	int k;
	char *lisence;
	int flag=0;
	senddata[0]=132;
	senddata[1]=0;
	senddata[2]=1;
	lisence=lisencedata;
	printf("now in send_lisence_to_mcu to write\n");
	i2c_write(i2c_address_mcu,mcu_i2c_info.block_operation_register,0xaa);
	usleep(10000);
	for(i=0;i<9;i++)
		{
			if(i<8)
				{
					memmove(&senddata[3],lisence,128);
					senddata[131]=0;
					for(k=0;k<131;k++)
						{
							senddata[131]+=senddata[k];
						}
				}
			else
				{
					senddata[2]=0;
					senddata[131]=0;
					for(k=0;k<131;k++)
						{
							senddata[131]+=senddata[k];
						}
				}
			while(1)
				{
					
					for(j=0;j<132;j++)
						{
							data_register=mcu_i2c_info.block_read_write_register+j;
							i2c_write(i2c_address_mcu,data_register,senddata[j]);
							usleep(10000);
							printf("in write data data_register=%02x  senddata[%d] =  %02x\n",data_register,j,senddata[j]);
						}
					//sleep(1);
					usleep(10000);
					break;
					
					
					/*if(flag==0)
						{
							break;
						}*/
				}
			printf("block over i = %d\n",i);
			if(i<8)
				{
					lisence=lisence+128;
				}
		}
	j=0;
	while(1)
		{
			j++;
			if(j>=4)
				{
					i2c_write(i2c_address_mcu,mcu_i2c_info.block_operation_register,0);
					usleep(10000);
					printf("mcu status is not 0\n");
					return -1;
				}
			i2c_read(i2c_address_mcu,mcu_i2c_info.block_operation_register,&readbuf);
			usleep(10000);
			printf("status === %02x   %02x   %02x\n",i2c_address_mcu,mcu_i2c_info.block_operation_register,readbuf);
			if(readbuf==0)
				{
					//flag=0;//写正确，不用重新写
					printf("readbuf ==0 \n");
					break;
				}
							
							
		}
	//senddata[2]=0;
	//senddata[132]=132;
	return 1;
}

int send_lisence_to_mcu_test(char *lisencedata)
{
	unsigned char senddata[133]={0};
	unsigned char readbuf;
	unsigned int data_register=0;
	int i;
	int j;
	int k;
	char *lisence;
	int flag=0;
	senddata[0]=132;
	senddata[1]=0;
	senddata[2]=1;
	lisence=lisencedata;
	printf("now in send_lisence_to_mcu to write\n");
	i2c_write(i2c_address_mcu,mcu_i2c_info.block_operation_register,0xaa);
	usleep(10000);
	for(i=0;i<9;i++)
		{
			if(i<8)
				{
					memmove(&senddata[3],lisence,128);
					senddata[131]=0;
					for(k=0;k<131;k++)
						{
							senddata[131]+=senddata[k];
						}
				}
			else
				{
					senddata[2]=0;
					senddata[131]=0;
					for(k=0;k<131;k++)
						{
							senddata[131]+=senddata[k];
						}
				}
			while(1)
				{
					
					for(j=0;j<132;j++)
						{
							data_register=mcu_i2c_info.block_read_write_register+j;
							i2c_write(i2c_address_mcu,data_register,senddata[j]);
							usleep(10000);
							printf("in write data data_register=%02x  senddata[%d] =  %02x\n",data_register,j,senddata[j]);
						}
					//sleep(1);
					break;
					
					
					/*if(flag==0)
						{
							break;
						}*/
				}
			printf("block over i = %d\n",i);
			if(i<8)
				{
					lisence=lisence+128;
				}
		}
	while(1)
		{
			j++;
			if(j>=4)
				{
					i2c_write(i2c_address_mcu,mcu_i2c_info.block_operation_register,0);
					usleep(10000);
					printf("mcu status is not 0\n");
					return -1;
				}
			i2c_read(i2c_address_mcu,mcu_i2c_info.block_operation_register,&readbuf);
			usleep(10000);
			if(readbuf==0)
				{
					//flag=0;//写正确，不用重新写
					printf("readbuf ==0 \n");
					break;
				}
							
							
		}
	//senddata[2]=0;
	//senddata[132]=132;
	return 1;
}




int send_upgrade_mcu_file_to_mcu(char *path)
{
	unsigned char senddata[133]={0};
	unsigned char readbuf;
	int i;
	int j;
	int k;
	int fd;
	char *lisence;
	int flag=0;
	int read_end_flag=0;
	int send_end_flag=0;
	senddata[0]=132;
	senddata[1]=1;
	senddata[2]=22;

	if((fd=open(path, O_RDWR))<0)
		{
			printf("can not open file = %s\n",path);
			return -1;
		}
	i2c_write(i2c_address_mcu,mcu_i2c_info.block_operation_register,0xaa);
	while(1)
		{
			if(read_end_flag==0)
				{
					//memmove(&senddata[3],lisence);

					memset(&senddata[3],0,128);
					if(read(fd,&senddata[3],128)!=128)
						{
							read_end_flag=1;
							printf("read mcu upgrate file %s to end\n",path);
						}
					senddata[132]=0;
					for(k=0;k<131;k++)
						{
							senddata[132]+=senddata[k];
						}
				}
			else
				{
					send_end_flag=1;
					senddata[2]=0;
					senddata[132]=0;
					for(k=0;k<131;k++)
						{
							senddata[132]+=senddata[k];
						}
				}
			while(1)
				{
					for(j=0;j<132;j++)
						{
							i2c_write(i2c_address_mcu,mcu_i2c_info.block_read_write_register,senddata[i]);
							while(1)
								{
									i2c_read(i2c_address_mcu,mcu_i2c_info.status_register,&readbuf);
									if(readbuf==0)
										{
											//flag=0;//写正确，不用重新写
											break;
										}
									else
										{
											//flag=2;
											sleep(1);
										}
									
								}
						}
					sleep(1);
					break;
				}
			if(send_end_flag==1)
				{
					break;
				}
			
		}
	//senddata[2]=0;
	//senddata[132]=132;
	return 1;
}


int get_lisence_from_mcu_to_check_old(void)
{
	while(1)
		{
			pthread_mutex_lock(&i2c_mutex);
			read_lisence_from_mcu(base_info.qqlicense);
			pthread_mutex_unlock(&i2c_mutex);
			if(qq_license_verification(base_info.qqlicense)==1)
				{
					printf("qqlicense verficastion success\n");
					break;
				}
			else
				{
					printf("qqlicense verficastion false\n");
				}
			while(1)
				{
					if(base_info.have_new_lisence==1)
						{
							break;
						}
					sleep(5);
				}
			base_info.have_new_lisence=0;
			
		}
	return 1;
}



int get_qq_sn_from_mcu(void)
{
	//char sendbuf[128]={0};
	int sendlength;
	int returncount=0;
	int returnflag=0;
	char sntmp[3]={0};
	int i;
	int j;

	
	projector_info.get_qq_sn_return_flag=0;
	uart_read_config(0x24);
	while(1)
		{
			if(projector_info.get_qq_sn_return_flag==1)
				{
					returnflag=1;
					break;
				}
			returncount++;
			if(returncount>20)
				{
					returnflag=0;
					break;
				}
			usleep(10000);
		}
	if(returnflag==1)
		{
			for(i=0;i<8;i++)
				{
					sprintf(sntmp,"%02x",base_info.sn_mcu[i]);
					j=i*2;
					memmove(&base_info.sn[j],sntmp,2);
				}
			
		}
	else
		{
			return -1;
		}
	return 1;
}


/*int get_lisence_from_mcu_to_check(void)
{
	while(1)
		{
			
			pthread_mutex_lock(&projector_info.mutex);
			get_qq_license_from_uart();
			pthread_mutex_unlock(&projector_info.mutex);
			if(qq_license_verification(base_info.qqlicense)==1)
				{
					printf("qqlicense verficastion success\n");
					break;
				}
			else
				{
					printf("qqlicense verficastion false\n");
				}
			sleep(5);
		}
	return 1;
}*/
int qq_license_produce(void)
{
	char *tmpfilename="/tmp/kkkqqtmp.kun";
	int fd;
	int nRet = 0;
	unsigned int nLicencelen   = 0;
	char szLicenceBuffer[512] = {};
	int nLicenceOutlen      = 0;
	int nLicenceBase16len   = 1024;
	char szLicenceBase16Buffer[1024] = {};

	fd=open(tmpfilename,O_RDWR|O_CREAT|O_TRUNC);
	if(fd<0)
		{
			printf("open %s fail\n",tmpfilename);
			return -1;
		}
	if(write(fd,qqkeydata,strlen(qqkeydata))!=strlen(qqkeydata))
		{
			printf("write string to %s fail\n",tmpfilename);
			return -1;
		}
	close(fd);
	nRet = ECDSASignToBuffer(tmpfilename,base_info.sn,16,szLicenceBuffer,&nLicencelen);
	if (1 == nRet)
	{
		
		
		base16Encode((unsigned char *)szLicenceBuffer,nLicencelen,(unsigned char *)szLicenceBase16Buffer,nLicenceBase16len,&nLicenceOutlen);
		
		memset(base_info.qqlicense,0,1024);
		memmove(base_info.qqlicense,szLicenceBase16Buffer,nLicenceOutlen);

	}
	else
		{
			return -1;
		}


	return 1;
}

int get_lisence_from_mcu_to_check(void)
{
	int flag;
	while(1)
		{
			
			pthread_mutex_lock(&projector_info.mutex);
			flag=get_qq_sn_from_mcu();
			pthread_mutex_unlock(&projector_info.mutex);
			if(flag==1)
				{
					if(qq_license_produce()>0)
						{
							
							if(qq_license_verification(base_info.qqlicense)==1)
								{
									printf("qqlicense verficastion success\n");
									break;
								}
							else
								{
									printf("qqlicense verficastion false\n");
								}
						}
					else
						{
							printf("produce qq license fail\n");
						}
					
				}
			else
				{
					printf("get qq license from mcu fail\n");
				}
			sleep(5);
		}
	return 1;
}


int get_qq_license_from_uart(void)
{
	unsigned char sendbuf[108];
	int run_flag=0;
	int i;


	projector_info.get_qq_license_busy=1;






	
	memset(base_info.qqlicense,0,1024);
	memset(projector_info.get_qq_license,0,256);
	//第一步发送读命令
	sendbuf[0]=0xaa;
	sendbuf[1]=0x55;
	
	sendbuf[2]=0x00;
	sendbuf[3]=0x06;
	
	sendbuf[4]=0x00;
	sendbuf[5]=0x04;

	sendbuf[6]=0x00;
	sendbuf[7]=0x02;
	
	sendbuf[8]=0x01;
	sendbuf[9]=0x00;
	
	sendbuf[10]=0x01;
	sendbuf[11]=0x00;
	sendbuf[12]=0;
	for(i=0;i<12;i++)
		{
			sendbuf[12]+=sendbuf[i];
		}
	printf("read qqlicense send data 1 =  ");
	for(i=0;i<13;i++)
		{
			printf(" %02x",sendbuf[i]);
		}
	printf("\n");

	
	projector_info.get_qq_license_return_flag=0;
	//usleep(200000);
	uart_send_data(sendbuf,13);
	//usleep(200000);
	sleep(1);
	if(projector_info.get_qq_license_return_flag==0)
		{
			printf("in get_qq_license_from_uart projector_info.get_qq_license_return_flag = %d\n",projector_info.get_qq_license_return_flag);
			sendbuf[0]=0xaa;
			sendbuf[1]=0x55;
			sendbuf[2]=0x00;
			sendbuf[3]=0x01;
			sendbuf[4]=0x80;
			sendbuf[5]=0x80;
			sendbuf[6]=0x04;
			sendbuf[7]=0x00;
			return -1;
		}
	else
		{
			memmove(base_info.qqlicense,projector_info.get_qq_license,256);
			sendbuf[0]=0xaa;
			sendbuf[1]=0x55;
			sendbuf[2]=0x00;
			sendbuf[3]=0x01;
			sendbuf[4]=0x80;
			sendbuf[5]=0x04;
			sendbuf[6]=0x02;
			sendbuf[7]=0x00;
			for(i=0;i<7;i++)
				{
					sendbuf[7]+=sendbuf[i];
				}
			printf("read qqlicense send data 1 =  ");
			for(i=0;i<8;i++)
				{
					printf(" %02x",sendbuf[i]);
				}
			printf("\n");
			//usleep(200000);
			uart_send_data(sendbuf,8);
			usleep(200000);
		}
	//sleep(10000);
	projector_info.get_qq_license_busy=0;
	return 1;
}

int send_qq_license_to_mcu_from_uart(unsigned char *license_qq)
{
	unsigned char sendbuf[525];
	int run_flag=0;
	int i;


	projector_info.get_qq_license_busy=1;






	
	sendbuf[0]=0xaa;
	sendbuf[1]=0x55;
	sendbuf[2]=0x00;
	sendbuf[3]=0x06;
	sendbuf[4]=0x00;
	sendbuf[5]=0x05;

	sendbuf[6]=0x00;
	sendbuf[7]=0x02;
	sendbuf[8]=0x01;
	sendbuf[9]=0x00;
	sendbuf[10]=0x01;
	sendbuf[11]=0x00;
	sendbuf[12]=0;
	for(i=0;i<12;i++)
		{
			sendbuf[12]+=sendbuf[i];
		}
	projector_info.get_qq_license_return_flag=0;
	printf("send qqlicense  send data 1 =  ");
	for(i=0;i<13;i++)
		{
			printf(" %02x",sendbuf[i]);
		}
	printf("\n");
	//usleep(200000);
	uart_send_data(sendbuf,13);
	usleep(200000);

	sleep(1);
	if(projector_info.get_qq_license_return_flag==0)
		{
			printf("send_qq_license_to_mcu_from_uart 1 false\n");
			sendbuf[0]=0xaa;
			sendbuf[1]=0x55;
			sendbuf[2]=0x00;
			sendbuf[3]=0x01;
			sendbuf[4]=0x80;
			sendbuf[5]=0x80;
			sendbuf[6]=0x04;
			sendbuf[7]=0x00;
			return -1;
		}
	else
		{
			if(projector_info.get_qq_license[0]==0)
				{
					printf("projector_info.get_qq_license[0]==0\n");
					sendbuf[0]=0xaa;
					sendbuf[1]=0x55;
					
					sendbuf[2]=0x01;
					sendbuf[3]=0x01;
					
					sendbuf[4]=0x80;
					sendbuf[5]=0x05;
					
					sendbuf[6]=0x01;
					sendbuf[263]=0;
					for(i=0;i<256;i++)
						{
							sendbuf[7+i]=license_qq[i];
						}
					for(i=0;i<263;i++)
						{
							sendbuf[263]+=sendbuf[i];
						}
					projector_info.get_qq_license_return_flag=0;
					printf("send qqlicense  send data 2 =  ");
					for(i=0;i<264;i++)
						{
							printf(" %02x",sendbuf[i]);
						}
					printf("\n");
					//usleep(200000);
					uart_send_data(sendbuf,264);
					//usleep(200000);
					sleep(1);
					if(projector_info.get_qq_license_return_flag!=0)
						{
							
							sendbuf[0]=0xaa;
							sendbuf[1]=0x55;
							sendbuf[2]=0x00;
							sendbuf[3]=0x01;
							sendbuf[4]=0x80;
							sendbuf[5]=0x05;
							sendbuf[6]=0x02;
							sendbuf[7]=0x00;
							for(i=0;i<7;i++)
								{
									sendbuf[7]+=sendbuf[i];
								}
							printf("send qqlicense  send data 3 =  ");
							for(i=0;i<8;i++)
								{
									printf(" %02x",sendbuf[i]);
								}
							printf("\n");
							//usleep(200000);
							uart_send_data(sendbuf,8);
							//usleep(200000);
							sleep(1);
							if(projector_info.get_qq_license[0]==0)
								{
									
									return 1;
								}
							else
								{
									return -1;
								}
							
						}
					else
						{
							printf("send_qq_license_to_mcu_from_uart 2 false\n");
							sendbuf[0]=0xaa;
							sendbuf[1]=0x55;
							sendbuf[2]=0x00;
							sendbuf[3]=0x01;
							sendbuf[4]=0x80;
							sendbuf[5]=0x05;
							sendbuf[6]=0x02;
							sendbuf[7]=0x00;
							for(i=0;i<7;i++)
								{
									sendbuf[7]+=sendbuf[i];
								}
							printf("send qqlicense  send data 3 =  ");
							for(i=0;i<8;i++)
								{
									printf(" %02x",sendbuf[i]);
								}
							printf("\n");
							//usleep(200000);
							uart_send_data(sendbuf,8);
							usleep(200000);
							return -1;
						}
				}
			else
				{
					return -1;
				}
		}
	projector_info.get_qq_license_busy=0;
	return 1;
}

/****************************************************************************
初始化音量


******************************************************************************/

int init_audio_card(void)
{
	int playback_volume_min,playback_volume_max;
	int capture_volume_min,capture_volume_max;
	int capture_volume;
	snd_mixer_t *mixer;
	snd_mixer_elem_t *pcm_element;
	int i;
	int ret=0;
	int count=0;
	ret=snd_mixer_open(&mixer,0);
	if(ret!=0)
		{
			printf("snd_mixer_open false\n");
			return -1;
		}
	ret=snd_mixer_attach(mixer, "default");
	if(ret!=0)
		{
			printf("snd_mixer_attach false\n");
			return -1;
		}
	ret=snd_mixer_selem_register(mixer, NULL, NULL);
	if(ret!=0)
		{
			printf("snd_mixer_selem_register false\n");
			return -1;
		}
	ret=snd_mixer_load(mixer);
	if(ret!=0)
		{
			printf("snd_mixer_load false\n");
			return -1;
		}
	pcm_element = snd_mixer_first_elem(mixer);
	if(pcm_element==NULL)
		{
			printf("snd_mixer_first_elem false\n");
			return -1;
		}
	count=snd_mixer_get_count(mixer);
	if(count==0)
		{
			printf("snd_mixer_get_count count = 0");
			return -1;
		}
	printf("snd_mixer_get_count count = %d\n",count);
	snd_mixer_selem_get_playback_volume_range(pcm_element,&playback_volume_min,&playback_volume_max);
	snd_mixer_selem_set_playback_volume(pcm_element,SND_MIXER_SCHN_FRONT_LEFT,playback_volume_max);
	snd_mixer_selem_set_playback_volume(pcm_element,SND_MIXER_SCHN_FRONT_RIGHT,playback_volume_max);
	/*for(i=0;i<count;i++)
		{
			//调音量到最大
			if((i==0)||(i==1)||(i==2)||(i==3)||(i==4)||(i==5)||(i==13))
				{
					snd_mixer_selem_get_playback_volume_range(pcm_element,&playback_volume_min,&playback_volume_max);
					snd_mixer_selem_set_playback_volume(pcm_element,SND_MIXER_SCHN_FRONT_LEFT,playback_volume_max);
					snd_mixer_selem_set_playback_volume(pcm_element,SND_MIXER_SCHN_FRONT_RIGHT,playback_volume_max);
				}
			if(i==14)
				{
					snd_mixer_selem_get_playback_volume_range(pcm_element,&playback_volume_min,&playback_volume_max);
					snd_mixer_selem_set_playback_volume(pcm_element,SND_MIXER_SCHN_FRONT_LEFT,playback_volume_min);
					snd_mixer_selem_set_playback_volume(pcm_element,SND_MIXER_SCHN_FRONT_RIGHT,playback_volume_min);
				}
			//打开通道  不静音
			if((i==0)||(i==1)||(i==2)||(i==3)||(i==5)||(i==13))
				{
					snd_mixer_selem_set_playback_switch(pcm_element,SND_MIXER_SCHN_FRONT_LEFT,1);
					snd_mixer_selem_set_playback_switch(pcm_element,SND_MIXER_SCHN_FRONT_RIGHT,1);
				}
			//打开话筒设置话筒音量
			if(i==9)
				{
					
					snd_mixer_selem_set_capture_switch(pcm_element,SND_MIXER_SCHN_FRONT_LEFT,1);
					snd_mixer_selem_set_capture_switch(pcm_element,SND_MIXER_SCHN_FRONT_RIGHT,1);
					//snd_mixer_selem_get_capture_volume(pcm_element,SND_MIXER_SCHN_FRONT_LEFT,&capture_volume);
					snd_mixer_selem_get_capture_volume_range(pcm_element,&capture_volume_min,&capture_volume_max);
					snd_mixer_selem_set_capture_volume(pcm_element,SND_MIXER_SCHN_FRONT_LEFT,capture_volume_max);
					snd_mixer_selem_set_capture_volume(pcm_element,SND_MIXER_SCHN_FRONT_RIGHT,capture_volume_max);
				}
			//选择输入音源
			if(i==11)
				{
					snd_mixer_selem_set_enum_item(pcm_element,SND_MIXER_SCHN_FRONT_LEFT,1);
					snd_mixer_selem_set_enum_item(pcm_element,SND_MIXER_SCHN_FRONT_RIGHT,1);
				}
			if(i<14)
				{
					printf("now seelk to next element\n");
					pcm_element=snd_mixer_elem_next(pcm_element);
					if(pcm_element==NULL)
						{
							printf("in for snd_mixer_first_elem false\n");
							return -1;
						}
				}
		}*/
	return 1;
}


int init_speex_socket(void)
{
	if ((speex_basedata.dtsclient=socket(AF_INET, SOCK_DGRAM, 0)) <0)
    {
        perror("ERROR--speex_read_thread:  open client_socket fail: ");
        return -1;
    }
	return 1;
}
int set_dsp_parameters(void)
{
	int wavfd; /* wav文件的描述符 */
    int arg;   /* ioctl参数 */
    int ret;   /* 返回值 */
    int parm;
    parm=0x02<<16+0x08;
    ret = ioctl(speex_basedata.fd, SNDCTL_DSP_SETFRAGMENT, &parm);
    if (ret==-1)
    {
        printf("ERROR--init_dsp:  set format %d fail\n",speex_basedata.format);
        close(speex_basedata.fd);
        return -1;
    }
    ret = ioctl(speex_basedata.fd, SOUND_PCM_WRITE_BITS, &speex_basedata.format);
    if (ret==-1)
    {
        printf("ERROR--init_dsp:  set format %d fail\n",speex_basedata.format);
        close(speex_basedata.fd);
        return -1;
    }
    ret = ioctl(speex_basedata.fd, SOUND_PCM_WRITE_CHANNELS, &speex_basedata.channel);
    if (ret==-1)
    {
        printf("ERROR--init_dsp:  set channel %d fail\n",speex_basedata.channel);
        close(speex_basedata.fd);
        return -1;
    }
    ret = ioctl(speex_basedata.fd, SOUND_PCM_WRITE_RATE, &speex_basedata.rate);
    if (ret==-1)
    {
        printf("ERROR--init_dsp:  set rate %d fail\n",speex_basedata.rate);
        close(speex_basedata.fd);
        return -1;
    }
    return 1;
}
int init_dsp_device(void)
{
	int wavfd; /* wav文件的描述符 */
    int arg;   /* ioctl参数 */
    int ret;   /* 返回值 */
    speex_basedata.fd=open("/dev/dsp", O_WRONLY);
    if (speex_basedata.fd < 0) {
        printf("open of /dev/dsp failed\n");
        return -1;
    }
    if(set_dsp_parameters()<0)
    {
        close(speex_basedata.fd);
        return -1;
    }

    return 1;
}
int init_speex(void)
{
	speex_basedata.inflag=0;
	speex_basedata.port=8735;
	
	speex_basedata.rate=16000;
	speex_basedata.channel=2;
	speex_basedata.format=16;
	speex_basedata.nbytes=1280;
	speex_basedata.sockaddrlength=sizeof(struct sockaddr_in);

	pthread_mutexattr_init(&speex_basedata.mutex);
	
	if(init_speex_socket()<0)
		{
			printf("ERROR--init_speex:  init_speex_socket fail\n");
			return -1;
		}
	if(init_dsp_device()<0)
		{
			printf("ERROR--init_speex:  init_dsp_device fail\n");
			return -1;
		}
	//if(init_dsp_parameers()<0)
	////	{
	//		printf("ERROR--init_speex:  init_dsp_paramers fail\n");
	//		return -1;
	//	}
	return 1;
}

void *speex_socketsend_thread(void *ptr)
{
	char sendbuf[512]={0};
	strcpy(sendbuf,"brxydts");
	strcpy((sendbuf+7),base_info.id);
	strcpy((sendbuf+19),base_info.name);
	while(1)
		{
			
			speex_basedata.serveraddr.sin_family = AF_INET;
			speex_basedata.serveraddr.sin_port = htons(speex_basedata.port);
			speex_basedata.serveraddr.sin_addr.s_addr = inet_addr(base_info.ip);
			if (speex_basedata.serveraddr.sin_addr.s_addr == INADDR_NONE)
				{
					printf("ERROR--speex_read_thread: serveraddr.sin_addr.s_addr == INADDR_NONE\n  ");
					
				}
			else
				{
					memset(sendbuf,0,512);
					strcpy(sendbuf,"brxydts");
					strcpy((sendbuf+7),base_info.id);
					strcpy((sendbuf+19),base_info.name);
					sendto(speex_basedata.dtsclient, sendbuf, strlen(sendbuf), 0, (struct sockaddr *)&speex_basedata.serveraddr,speex_basedata.sockaddrlength);
					
				}
			sleep(3);
		}
}
void *speex_socketreceive_thread(void *ptr)
{
	SpeexBits speexbits;
    SpeexStereoState stereo = SPEEX_STEREO_STATE_INIT;
	void *st;
	char recervibuf[2000];
	short decodebuf[2000]={0};
	spx_int32_t frame_size;
	int dataheaderlength;
	int n,ret;
	struct sockaddr_in receiveaddr;
	char *inbufpointer;

	dataheaderlength=sizeof(speexdataheader);
	speex_bits_init(&speexbits);
	st = speex_decoder_init(speex_lib_get_mode(SPEEX_MODEID_WB));
	speex_encoder_ctl(st, SPEEX_GET_FRAME_SIZE, &frame_size);
	
	while(1)
		{
			n=recvfrom(speex_basedata.dtsclient,recervibuf,512,0,(struct sockaddr*)&receiveaddr,&speex_basedata.sockaddrlength);
        	if(n<0)
        		{
            		printf("ERROR--speex_socketreceive_thread:  recvfrom fail\n");
					
            		continue;
        		}
			memmove((void *)&speex_basedata.sdh,recervibuf,dataheaderlength);
            inbufpointer=recervibuf+dataheaderlength;
            if((speex_basedata.sdh.nbytes+dataheaderlength)!=n)
            {
                printf("WARNING--speex_socketreceive_thread:  (sdh.nbytes+dataheaderlength)!=n\n");
                continue;
            }
			speex_bits_read_from(&speexbits,inbufpointer,speex_basedata.sdh.nbytes);
        	ret=speex_decode_int(st,&speexbits,decodebuf);
        	speex_bits_reset(&speexbits);
        	if (speex_basedata.sdh.channel==2)
        	{
            	speex_decode_stereo_int(decodebuf, frame_size, &stereo);
        	}
			pthread_mutex_lock(&speex_basedata.mutex);
			speex_basedata.nbytes=frame_size*speex_basedata.sdh.channel*2;
			memmove(speex_basedata.inbuf,decodebuf,speex_basedata.nbytes);
			speex_basedata.inflag=1;
			pthread_mutex_unlock(&speex_basedata.mutex);
			
		}
	speex_decoder_destroy(st);
    speex_bits_destroy(&speexbits);
}
void *speex_dsp_write_thread(void *ptr)
{
	int inflag=0;
	int nbytes;
	while(1)
		{
			while(1)
				{
					pthread_mutex_lock(&speex_basedata.mutex);
					if(speex_basedata.inflag==1)
						{
							memmove(speex_basedata.playbuf,speex_basedata.inbuf,speex_basedata.nbytes);
							speex_basedata.inflag=0;
							nbytes=speex_basedata.nbytes;
							inflag=1;
						}
					else
						{
							inflag=0;
						}
					pthread_mutex_unlock(&speex_basedata.mutex);
					if(inflag==1)
						{
							break;
						}
					usleep(1000);
				}
			write(speex_basedata.fd,speex_basedata.playbuf,nbytes);
		}
	return;
}
/****************************************************************************
主函数



******************************************************************************/
int main(int argc,char **argv)
{
	int ret;
	//printf("now initdevice //////////////////////////////////////////////////////\n");

	if(init_first()<0)
		{
			printf("init first false\n");
			return -1;
		}
	//printf("init first success\n");
	//sleep(1);
	if(init_audio_card()<0)
		{
			printf("init init_audio_card false\n");
			return -1;
		}
	if(init_uci()<0)
		{
			printf("cp file brxy to retry init_uci\n");
			system("rm /usr/share/brxydata/brxy");
			system("cp /usr/share/brxydata/filesback/brxy /usr/share/brxydata/brxy");
			system("chmod 777 /usr/share/brxydata/brxy");
			if(init_uci()<0)
				{
					printf("init uci false\n");
					return -1;
				}
		}
	//printf("init uci success\n");

	/*if(qq_sn_to_license()<0)
			{
				printf("qq_sn_to_license false\n");
				return -1;
			}*/


	
	//sleep(1);
	if(init_rfid_uci()<0)
		{
			printf("cp file card to retry init_rfid_uci\n");
			system("rm /usr/share/brxydata/card");
			system("cp /usr/share/brxydata/filesback/card /usr/share/brxydata/card");
			system("chmod 777 /usr/share/brxydata/card");
			if(init_rfid_uci()<0)
				{
					printf("init rfid_uci false\n");
					return -1;
				}
			
		}
	//printf("init rfid uci success\n");
	//sleep(2);
	/*init_broadcast_socket();
	init_wifi_internettings_header();
	if(get_internetthings_mac_from_uci()<0)
		{
			printf("get wifi mac from uci false\n");
			return -1;
		}*/
	init_wifi_internettings_header();
	libcurl_global_init();
	if(luci_socket_init()<0)
		{
			printf("init luci socket false\n");
			return -1;
		}
	/*if(i2c_init()<0)
		{
			printf("init i2c false\n");
			return -1;
		}*/
	if(init_serial()<0)
		{
			printf("init serial false\n");
			return -1;
		}
	printf("now to init device on i2c\n");
	/*if(init_wm_tas_rfid_tca()<0)
		{
			printf("init init_wm_tas_rfid_tca false\n");
			return -1;
		}*/
	/*if(init_led_struct()<0)
		{
			printf("init key button fail\n");
			return -1;
		}*/
	if((ret=sem_init(&base_info.qq_init_sem_t,0,0))!=0)
		{
			perror("init semphare key_button_info.wake_key_deal fail");
			return -1;
		}
	
	




	
	/*if(init_key_button()<0)
		{
			printf("init key button fail\n");
			return -1;
		}*/
	/*if(init_schema_play()<0)
		{
			printf("init init_schema_play  fail\n");
			return -1;
		}
	
	if(init_broadcast_mpe_play()<0)
		{
			printf("init init_broadcast_mpe_play  fail\n");
			return -1;
		}*/
	if(pc_det_init()<0)
		{
			printf("init  pc_det_init fail\n");
			return -1;
		}


	if(init_speex()<0)
		{
			printf("init  init_speex fail\n");
			return -1;
		}
	







	
	pthread_mutexattr_init(&power_control_status.mutex);
	/*pthread_mutexattr_settype(&power_control_status.attr,PTHREAD_MUTEX_RECURSIVE_NP);//嵌套锁，允许同一个线程对同一个锁成功获得多次，并通过多次unlock解锁。如果是不同线程请求，则在加锁线程解锁时重新竞争。
	if (pthread_mutex_init(&power_control_status.mutex,&power_control_status.attr)!= 0) {
		printf("pthread mutex init ERROR\n");
		return -1;
	} */
	pthread_mutexattr_init(&local_play_threadheader.mutex);
	/*pthread_mutexattr_settype(&local_play_threadheader.attr,PTHREAD_MUTEX_RECURSIVE_NP);//嵌套锁，允许同一个线程对同一个锁成功获得多次，并通过多次unlock解锁。如果是不同线程请求，则在加锁线程解锁时重新竞争。
	if (pthread_mutex_init(&local_play_threadheader.mutex,&local_play_threadheader.attr)!= 0) {
		printf("pthread mutex init ERROR\n");
		return -1;
	} */
	pthread_mutexattr_init(&i2c_mutex);
	/*pthread_mutexattr_settype(&i2c_attr,PTHREAD_MUTEX_RECURSIVE_NP);//嵌套锁，允许同一个线程对同一个锁成功获得多次，并通过多次unlock解锁。如果是不同线程请求，则在加锁线程解锁时重新竞争。
	if (pthread_mutex_init(&i2c_mutex,&i2c_attr)!= 0) {
		printf("pthread mutex init ERROR\n");
		return -1;
	} */
	pthread_mutexattr_init(&action_mutex);
	/*pthread_mutexattr_settype(&action_attr,PTHREAD_MUTEX_RECURSIVE_NP);//嵌套锁，允许同一个线程对同一个锁成功获得多次，并通过多次unlock解锁。如果是不同线程请求，则在加锁线程解锁时重新竞争。
	if (pthread_mutex_init(&action_mutex,&action_attr)!= 0) {
		printf("pthread mutex init ERROR\n");
		return -1;
	} */
	pthread_mutexattr_init(&key__mutex);
	/*pthread_mutexattr_settype(&key__attr,PTHREAD_MUTEX_RECURSIVE_NP);//嵌套锁，允许同一个线程对同一个锁成功获得多次，并通过多次unlock解锁。如果是不同线程请求，则在加锁线程解锁时重新竞争。
	if (pthread_mutex_init(&key__mutex,&key__attr)!= 0) {
		printf("pthread mutex init ERROR\n");
		return -1;
	} */

	
	pthread_mutexattr_init(&rfid_card_uci_mutex);
	/*pthread_mutexattr_settype(&rfid_card_uci_attr,PTHREAD_MUTEX_RECURSIVE_NP);//嵌套锁，允许同一个线程对同一个锁成功获得多次，并通过多次unlock解锁。如果是不同线程请求，则在加锁线程解锁时重新竞争。
	if (pthread_mutex_init(&rfid_card_uci_mutex,&rfid_card_uci_attr)!= 0) {
		printf("pthread mutex init ERROR\n");
		return -1;
	} */
	pthread_mutexattr_init(&projector_info.mutex);
	/*pthread_mutexattr_settype(&projector_info.attr,PTHREAD_MUTEX_RECURSIVE_NP);//嵌套锁，允许同一个线程对同一个锁成功获得多次，并通过多次unlock解锁。如果是不同线程请求，则在加锁线程解锁时重新竞争。
	if (pthread_mutex_init(&projector_info.mutex,&projector_info.attr)!= 0) {
		printf("pthread mutex init ERROR\n");
		return -1;
	} */
	pthread_mutexattr_init(&nopoll_info.mutex);
	/*pthread_mutexattr_settype(&nopoll_info.attr,PTHREAD_MUTEX_RECURSIVE_NP);//嵌套锁，允许同一个线程对同一个锁成功获得多次，并通过多次unlock解锁。如果是不同线程请求，则在加锁线程解锁时重新竞争。
	if (pthread_mutex_init(&nopoll_info.mutex,&nopoll_info.attr)!= 0) {
		printf("pthread mutex init ERROR\n");
		return -1;
	} */
	
	/*ret=pthread_create(&add_wifi_things_thread, NULL, &add_internetthings_from_broadcast, NULL);//wifi加网
	if(ret!=0)	
	{  
		printf("Create add_internetthings_from_broadcast pthread error!\n");	
		return -1;	
	}
	ret=pthread_create(&select_wifi_things_thread, NULL, &select_wifi_internetthings_status, NULL);//wifi查询设备
	if(ret!=0)	
	{  
		printf("Create select_wifi_internetthings_status pthread error!\n");	
		return -1;	
	}
	ret=pthread_create(&recive_rfid_data_thread, NULL, &recive_wifi_rfid, NULL);
	if(ret!=0)	
	{  
		printf("Create recive_wifi_rfid pthread error!\n");	
		return -1;	
	}*/
	ret=pthread_create(&local_play_threadheader.play_thread_free, NULL, &local_play_thread_t_free, NULL);
	if(ret!=0)	
		{  
			printf("Create local_play_thread_t_free pthread error!\n");  
			return -1;	
		}
	
	

	ret=pthread_create(&projector_info.serial_read_pthread, NULL, &Serial_Read_Thread, NULL);//建立串口读取线程
		if(ret!=0)	
		{  
			printf("Create Serial pthread error!\n");  
			return -1;	
		}
	read_all_data_from_mcu();
	//sleep(1);
	//read_all_data_from_mcu();
	//sleep(2);
	apply_net_work_to_mcu_and_openpermision();
	//usleep(200000);
	ret=pthread_create(&luci_socket_info.luci_socket_thread, NULL, &luci_socket_thread, NULL);
	if(ret!=0)	
		{  
			printf("Create luci_socket_thread pthread error!\n");  
			return -1;	
		}
	get_lisence_from_mcu_to_check();
	base_info.qq_init_success=1;


	
	/*ret=pthread_create(&led_glint.led_standy_pthread, NULL, &machine_standy_thread, NULL);
	if(ret!=0)	
		{  
			printf("Create machine_standy_thread pthread error!\n");  
			return -1;	
		}*/
	

	/*if(!initDevice())
		{
			printf("qq  initDevice false\n");
			return -1;
		}*/

	/*ret=pthread_create(&rfid_info.play_thread_free, NULL, &rfid_read_thread, NULL);
	if(ret!=0)	
		{  
			printf("Create rfid_read_thread pthread error!\n");  
			return -1;	
		}*/
	ret=pthread_create(&nopoll_info.creat_connection_thread, NULL, &creat_nopoll_connection, NULL);
	if(ret!=0)	
		{  
			printf("Create creat_nopoll_connection pthread error!\n");  
			return -1;	
		}
	ret=pthread_create(&nopoll_info.report_server_thread, NULL, &report_to_server_thread, NULL);
	if(ret!=0)	
		{  
			printf("Create rfid_read_thread pthread error!\n");  
			return -1;	
		}

	
	
	/*ret=pthread_create(&key_button_info.key_deal_thread, NULL, &deal_with_button_event, NULL);
	if(ret!=0)	
		{  
			printf("Create deal_with_button_event pthread error!\n");  
			return -1;	
		}*/
	/*ret=pthread_create(&key_button_info.key_read_thread, NULL, &read_button_event, NULL);
	if(ret!=0)	
		{  
			printf("Create read_button_event pthread error!\n");  
			return -1;	
		}*/
	/*ret=pthread_create(&rfid_info.rfid_close_thread, NULL, &close_machine_thread, NULL);
	if(ret!=0)	
		{  
			printf("Create close_machine_thread pthread error!\n");  
			return -1;	
		}*/


	
	/*ret=pthread_create(&schema_play_info.recive_pthread, NULL, &schema_recive_from_server, NULL);
		if(ret!=0)	
			{  
				printf("Create close_machine_thread pthread error!\n");  
				return -1;	
			}
	ret=pthread_create(&schema_play_info.write_pthread, NULL, &schema_write_to_pcm, NULL);
	if(ret!=0)	
		{  
			printf("Create close_machine_thread pthread error!\n");  
			return -1;	
		}

	ret=pthread_create(&broadcast_play_mp3_info.recive_pthread, NULL, &broadcast_play_mp3_recive_from_server, NULL);
		if(ret!=0)	
			{  
				printf("Create close_machine_thread pthread error!\n");  
				return -1;	
			}
	ret=pthread_create(&broadcast_play_mp3_info.write_pthread, NULL, &broadcast_play_mp3_write_to_pcm, NULL);
	if(ret!=0)	
		{  
			printf("Create close_machine_thread pthread error!\n");  
			return -1;	
		}*/
	ret=pthread_create(&speex_basedata.playthread, NULL, &speex_dsp_write_thread, NULL);
	if(ret!=0)	
		{  
			printf("Create speex_dsp_write_thread pthread error!\n");  
			return -1;	
		}
	ret=pthread_create(&speex_basedata.recivethread, NULL, &speex_socketreceive_thread, NULL);
	if(ret!=0)	
		{  
			printf("Create speex_socketreceive_thread pthread error!\n");  
			return -1;	
		}
	ret=pthread_create(&speex_basedata.sendthread, NULL, &speex_socketsend_thread, NULL);
	if(ret!=0)	
		{  
			printf("Create speex_socketsend_thread pthread error!\n");  
			return -1;	
		}
	



	
	//qq_init_decision();
	ret=pthread_create(&pc_det.pc_detect_thread, NULL, &output_run_pulse, NULL);
	if(ret!=0)	
		{  
			printf("Create pc_det_thread pthread error!\n");  
			return -1;	
		}
	sleep(1);
	base_info.qq_lisence_get_from_mcu_is_ok=1;

	

	
	

	//sleep(40);

	ret=pthread_create(&base_info.qq_pthread, NULL, &qq_thread, NULL);
	if(ret!=0)	
		{  
			printf("Create local_play_thread_t_free pthread error!\n");  
			return -1;	
		}

	
	//qq_init_decision();
	
	//sleep(15);
	//base_info.qq_init_flag=1;
	//sem_post(&base_info.qq_init_sem_t);
	while(1)
	{
		sleep(100);
	}

	mad_synth_finish(&schema_play_info.synth);
	mad_frame_finish(&schema_play_info.frame);
	mad_stream_finish(&schema_play_info.stream); 

	return 0;


	
}









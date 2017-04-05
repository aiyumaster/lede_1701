#!/bin/sh  
host_dir=`echo ~`                                # 当前用户根目录  
proc_name="/usr/k4"                     # 进程名  
file_name="/Candy/log/cron.log"                         # 日志文件  
pid=0  
  
proc_num()                                              # 计算进程数  
{  
    num=`ps | grep $proc_name | grep -v grep | wc -l`  
    return $num  
}  
  
proc_id()                                               # 进程号  
{  
    pid=`ps | grep $proc_name | grep -v grep | awk '{print $2}'`  
}  
  
proc_num  
number=$?  
if [ $number -eq 0 ]                                    # 判断进程是否存在  
then 
	if [ -f /tmp/websocketstartlog ];then
	
    echo "Starting service ..."  
    /etc/init.d/websocket restart >/dev/null 2>&1    # 重启进程的命令，请相应修改  
	fi
    #proc_id                                         # 获取新进程号  
    #echo ${pid}, `date` >> $host_dir$file_name      # 将新进程号和重启时间记录  
fi 
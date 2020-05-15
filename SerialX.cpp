#include <cstdio>      /*标准输入输出定义*/
#include <cstdlib>     /*标准函数库定义*/
#include <unistd.h>     /*Unix 标准函数定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <string.h>
#include <time.h>
#include "SerialX.h"
using namespace std;

//全局变量
    const int m_BaudRateArr[] = {B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300 };         // 波特率数组
    const int m_SpeedArr[] = {115200,  57600, 38400, 19200, 9600, 4800, 2400, 1200, 300 };           			// 波特率数组
    struct termios m_Setting;           // 串口配置的结构体
    int fd = -1;                             // 打开串口设备后返回的文件描述符

    unsigned long GetTickCount()
    {
        struct timespec ts;

        clock_gettime(CLOCK_MONOTONIC, &ts);

        return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }

    bool InitSerial(const char *devName,int BaudRate,int DataBits,int StopBits,char ParityBit, BYTE readTimeOuts)
    {
        /*open函数打开串口
        O_RDWR :串口可读写
        O_NOCTTY：可以告诉Linux这个程序不会成为这个端口上的“控制终端”.如果不这样做的话,所有的输入,比如键盘上过来的Ctrl+C中止信号等等,会影响到你的进程。
        O_NDELAY：标志则是告诉Linux,这个程序并不关心DCD信号线的状态——也就是不关心端口另一端是否已经连接（不阻塞）。
        */
        fd = open( devName, O_RDWR | O_NOCTTY |O_NDELAY);   //O_NDELAY表明打开串口时是非阻塞模式

        if(fd < 0)
        {
            fd = -1;
            printf("Can't Open the %s device.\n", devName);
            return false;
        }

        return SetSerialOption(BaudRate, DataBits, StopBits, ParityBit, readTimeOuts);
    }

    bool SetSerialOption(int BaudRate,int DataBits,int StopBits,char ParityBit, BYTE readTimeOuts)
    {

        if(fd < 0)
        {
            fd = -1;
            perror("Can't Open the serial device!");
            return false;
        }

        bzero(&m_Setting, sizeof(m_Setting));
         /*重新将串口设置为阻塞模式，即执行read函数时，如果没有数据就会阻塞等待，不往下执行，
         如果设置为非阻塞模式为fcntl(fd, F_SETFL, FNDELAY)，此时执行read函数时，如果没有数据，
         则返回-1，程序继续往下执行*/
        fcntl(fd, F_SETFL, 0);

        if( -1 == fd)
            return false;

        if( 0!= tcgetattr (fd,&m_Setting))
        {
            printf("InitCSerialX tcgetattr() line:%d failed\n",__LINE__);
            return false;
        }

        // 设置波特率
        for(int i = 0 ; i< sizeof(m_SpeedArr)/sizeof(int); i++)
        {
            if( BaudRate == m_SpeedArr[i])
            {
                tcflush(fd, TCIOFLUSH); // 清空发送接收缓冲区
                cfsetispeed(&m_Setting,m_BaudRateArr[i]);  // 设置输入波特率
                cfsetospeed(&m_Setting,m_BaudRateArr[i]);  // 设置输出波特率
                break;
            }
            if(i == sizeof(m_SpeedArr) / sizeof(int))
                return false;
        }

        m_Setting.c_cflag |= CLOCAL;//控制模式, 保证程序不会成为端口的占有者
        m_Setting.c_cflag |= CREAD; //控制模式, 使能端口读取输入的数据
        m_Setting.c_cflag &= ~CRTSCTS;//控制模式，关闭硬件流控制

        // 设置数据位
        m_Setting.c_cflag &= ~CSIZE;
        switch(DataBits)
        {
            case 6:m_Setting.c_cflag |= CS6 ; break;  //6位数据位
            case 7:m_Setting.c_cflag |= CS7 ; break;  //7位数据位
            case 8:m_Setting.c_cflag |= CS8 ; break;  //8位数据位
            default:
                    fprintf(stderr,"unsupported dataBits\n");
                    return false;
        }

        // 设置停止位
        switch(StopBits)
        {
            case 1: m_Setting.c_cflag &= ~CSTOPB;break;  //1位停止位
            case 2: m_Setting.c_cflag |= CSTOPB; break;  //2位停止位
            default:
                    return false;
        }

        // 设置奇偶校验位
        switch(ParityBit)
        {
            case 'n':
            case 'N':
                    m_Setting.c_cflag &= ~PARENB;  // 关闭c_cflag中的校验位使能标志PARENB）
                    m_Setting.c_iflag &= ~INPCK;   // 关闭输入奇偶检测
                    break;
            case 'o':
            case 'O':
                    m_Setting.c_cflag |= (PARODD | PARENB);//激活c_cflag中的校验位使能标志PARENB，同时进行奇校验
                    m_Setting.c_iflag |= INPCK;  // 开启输入奇偶检测
                    break;

            case 'e':
            case 'E':
                    m_Setting.c_cflag |= PARENB;//激活c_cflag中的校验位使能标志PARENB
                    m_Setting.c_cflag &= ~PARODD;// 使用偶校验
                    m_Setting.c_iflag |= INPCK;// 开启输入奇偶检测
                    break;
            case 's':
            case 'S':
                    m_Setting.c_cflag &= ~PARENB; // 关闭c_cflag中的校验位使能标志PARENB）
                    m_Setting.c_cflag &= ~CSTOPB; // 设置停止位位一位
                    break;
            default:
                    fprintf(stderr,"unsupported parityBit\n");
                    return false;
        }


        m_Setting.c_oflag &= ~OPOST;// 设置为原始输出模式
        m_Setting.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // 设置为原始输入模式
        /*所谓标准输入模式是指输入是以行为单位的，可以这样理解，输入的数据最开始存储在一个缓冲区里面（但并未真正发送出去），
        可以使用Backspace或者Delete键来删除输入的字符，从而达到修改字符的目的，当按下回车键时，输入才真正的发送出去，这样终端程序才能接收到。
        通常情况下我们都是使用的是原始输入模式，也就是说输入的数据并不组成行。在标准输入模式下，系统每次返回的是一行数据，在原始输入模式下，
        系统又是怎样返回数据的呢？如果读一次就返回一个字节，那么系统开销就会很大，但在读数据的时候，我们也并不知道一次要读多少字节的数据，
        解决办法是使用c_cc数组中的VMIN和VTIME，如果已经读到了VMIN个字节的数据或者已经超过VTIME时间，系统立即返回。*/

        m_Setting.c_cc[VTIME] = readTimeOuts; //单位0.1秒
        m_Setting.c_cc[VMIN] = 0;  ;


        /*刷新串口数据
        TCIFLUSH:刷新收到的数据但是不读(清空接收缓冲区)
        TCOFLUSH:刷新写入的数据但是不传送(清空发送缓冲区)
        TCIOFLUSH:同时刷新收到的数据但是不读，并且刷新写入的数据但是不传送。 (同时清空接收和发送缓冲区)*/
        tcflush(fd, TCIFLUSH);

        // 激活配置
        if( 0 != tcsetattr(fd,TCSANOW,&m_Setting))
        {
            printf("InitCSerialX tecsetattr() %d failed\n",__LINE__);
            return false;
        }
        return true;
    }


    // 关闭串口
    bool CloseSerial()
    {
        if( -1 == fd)
            return false;

        close(fd);
        fd = -1;

        return true;
    }


    //从串口读取数据
    int Read(unsigned char *readBuffer,const int bufferSize)
    {
    	int totalNums = 0;
        if( -1 == fd)
            return -1;
        unsigned long tick1 = GetTickCount();
        int nRet = read(fd,readBuffer,bufferSize);
        unsigned long tickSpan = GetTickCount() - tick1;
        //printf("Read Time is %d ms.\n", (int)tickSpan);
        if(nRet == 0)
            return  0;
        totalNums = nRet;
        if(bufferSize > totalNums)
        {
        	int subNums = 0;
        	int nRet2 = 0;
        	for(;;)
        	{
				subNums = bufferSize - totalNums;
				nRet2 = read(fd,readBuffer+totalNums, subNums);
				if(nRet2 == 0)
					break;
				totalNums+=nRet2;
				if(totalNums >= bufferSize)
					break;
        	}
        }
        return totalNums;
    }

    // 往串口写入数据
    int Write(unsigned char *writeBuffer,const int bufferSize)
    {
        if( -1 == fd)
            return -1;
        /*刷新串口数据
        TCIFLUSH:刷新收到的数据但是不读(清空接收缓冲区)
        TCOFLUSH:刷新写入的数据但是不传送(清空发送缓冲区)
        TCIOFLUSH:同时刷新收到的数据但是不读，并且刷新写入的数据但是不传送。 (同时清空接收和发送缓冲区)*/
        tcflush(fd, TCIOFLUSH);
        return write(fd,writeBuffer,bufferSize);


    }

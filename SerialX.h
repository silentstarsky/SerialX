/*
 * SerialX.h
 *
 *  Created on: 2020年1月6日
 *      Author: bill
 */

#ifndef SRC_SERIALX_H_
#define SRC_SERIALX_H_


	typedef unsigned char BYTE;

    bool InitSerial(const char *DevName, int BaudRate,int DataBits,int StopBits,char ParityBit,BYTE readTimeOuts); // 初始化串口
    bool CloseSerial();// 关闭串口
    bool SetSerialOption(int BaudRate,int DataBits,int StopBits,char ParityBit, BYTE readTimeOuts);	// 设置串口参数
    int Write(BYTE *Buff, const int Len);  // 向串口写入数据
    int Read(BYTE *Buff,  const int Len);  // 从串口中读取数据




#endif /* SRC_SERIALX_H_ */

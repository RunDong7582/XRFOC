#ifndef _FocDebug_H
#define _FocDebug_H
#include <stdio.h>
typedef struct
{
		float graph[5];
		unsigned char sendBuff[50];
	  int sendFlag;
}VofaProtocol_t;
void GraphDataCopy(float *buff,int count);
void VofaGraphSend(int graphCount);
#define debug_log(fmt, ...) printf(fmt, ##__VA_ARGS__)
#endif

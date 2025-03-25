#include "FocDebug.h"
#include "usart.h"
#include "string.h"
VofaProtocol_t gVofa;
/*******************************************************************************
* @brief   日志 曲线初始化
 * @param   
 * @return  
 ******************************************************************************/
void DebugLogInit(void)
{
		
}

unsigned char SumCheck(unsigned char data[], int count)
{
		int sum = 0;
		for (int i = 0; i < count; i++)
		{
				sum += data[i];
		}
		return (unsigned char)(sum & 0xFF);
}
/*******************************************************************************
*  @brief   拷贝曲线信息
 * @param   
 * @return  
 ******************************************************************************/
void GraphDataCopy(float *buff,int count)
{
	 gVofa.sendFlag = 1;
	 for(int i = 0;i < count;i++)
	{
		gVofa.graph[i] = buff[i];
	}
}
/*******************************************************************************
*  @brief   发送曲线信息
 * @param   
 * @return  
 ******************************************************************************/
void VofaGraphSend(int graphCount)
{
	 int idx = 0;
	 float buff[5];
	 unsigned char tbuff[10];
	 unsigned char *p;
	 if(gVofa.sendFlag == 1)
	 {
		 gVofa.sendFlag = 0;
//		 p = (unsigned char*)(&gVofa.graph[0]);
//		 tbuff[0] = 0xFF;
//		 tbuff[1] = 0x01;
//		 tbuff[2] = p[0];
//		 tbuff[3] = p[1];
//		 tbuff[4] = p[2];
//		 tbuff[5] = p[3];
//		 tbuff[6] = SumCheck(tbuff,6);
//		 HAL_UART_Transmit(&huart1,tbuff,7,800);
		 switch(graphCount)
		 {
			 case 1:
				 printf("%0.2f\n",gVofa.graph[0]);
				 break;
			 case 2:
				 printf("%0.2f,%.2f\n",gVofa.graph[0],gVofa.graph[1]);
				 break;
			 case 3:
				 printf("%0.2f,%0.2f,%.2f\n",gVofa.graph[0],gVofa.graph[1],gVofa.graph[2]);
				 break;
		 }
		 
//		 memcpy(buff,gVofa.graph,graphCount * 4);
//		 char *p = (char*)(buff);
//		 for(idx= 0;idx < graphCount * 4;idx++)
//		 {
//			 gVofa.sendBuff[idx] = p[idx];
//		 }
//		 HAL_UART_Transmit(&huart1,gVofa.sendBuff,idx,100);
//		 const char tail[] = {0x00,0x00,0x80,0x7f};
//		 gVofa.sendBuff[idx++] = 0x00;
//		 gVofa.sendBuff[idx++] = 0x00;
//		 gVofa.sendBuff[idx++] = 0x80;
//		 gVofa.sendBuff[idx++] = 0x7f;
//		 HAL_UART_Transmit(&huart1,(unsigned char *)tail,4,100);
		 
	 }

}
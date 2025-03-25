#ifndef  _FOCCONTROL_H
#define  _FOCCONTROL_H
#define FOC_STATE_IDLE     0
#define FOC_STATE_ALIGN    1
#define FOC_STATE_STARTUP  2
#define FOC_STATE_RUN      3
#define FOC_STATE_STOP     4
typedef struct
{
	 int machineState;
	 int sw;
	 float target;
	 float openVoltageTheta;
	 float alginID;      //对其操作时的D轴电流
}control_t;
void ControlInit(void);
void FocMain(void);
void BspInit(void);
void PwmStop(void);
void PwmStart(void);
void DoControl(control_t *pHandle);
#endif

/**
 *  @file:      XRFOC.h
 *  @brief:     Motor run task
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 14:40
 *  @version:   XRFOC v0.1
 */

int xfoc_module_init ( void );
void serialReceiveUserCommand ( void );
float serial_motor_target ( void );
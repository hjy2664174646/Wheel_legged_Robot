#ifndef __ROS_COMMON_H
#define __ROS_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "main.h"
#include "chassisR_task.h"
#include "ins_task.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
	
/*
	**************  数据发送格式 ***************
	            **** 里程计数据 *****
						[0x55 0xAA]  (2 bytes header)
						x     (4 bytes float)
						y     (4 bytes float)
						yaw   (4 bytes float)
						v     (4 bytes float)
						w     (4 bytes float)
	
							***** imu数据 ******
						Gyro[3] 	(4*3=12 bytes float)
						Accel[3]	(4*3=12 bytes float)
						Pitch 		(4 bytes float)
						Roll;	  	(4 bytes float)		
						Yaw;			(4 bytes float)	
						checksum 	(2 bytes uint16)
	*/
typedef struct  __attribute__((packed))odomStruct_s
{
	/* 帧头 */
	uint16_t head;				
	
	/* odom 数据 */
	float x;				/* 位置x (m)*/
	float y;				/* 位置y (m)*/
	float yaw;				/* 偏航角（朝向） rad*/
	float v;			/* 线速度（m/s） */
	float w;			/* 角速度（rad/s） */
	
	/* imu 数据 */
	float Gyro[3];  /* 角速度 */
  float Accel[3]; /* 加速度 */
	float Pitch; 		/* 俯仰角 */
	float Roll;	 		/* 横滚角 */
	float Yaw;			/* 偏航角 */
	
	/* 校验和 */
	uint16_t checksum;			
}odomStruct_t;
extern float ros_vx ;
extern float ros_wz ;

void ParseByte(uint8_t byte);
void odom_update(chassis_t *chassis,INS_t *ins,float dt);
void odom_msg_task(void);
void odom_update_task(void);


// 轮距 & 半径（单位：米）
#define WHEEL_BASE   0.286f   // 28.6 cm
#define WHEEL_RADIUS 0.06f    // 12 cm 直径

#endif




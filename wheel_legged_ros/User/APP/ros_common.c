#include "ros_common.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
extern odomStruct_t odom;
// 全局变量定义
odomStruct_t odom;
extern INS_t INS;
extern chassis_t chassis_move;
//ros下发的 速度&角速度 控制信息
float ros_vx = 0;
float ros_wz = 0;

osMutexId odom_mutex;

/* 
					*******有限状态机*****
	* FSM（有限状态机）按顺序等待帧头、长度、类型
	* buf 存 payload（vx 与 wz）
	* index 记录当前 payload 写到哪里
*/
typedef enum {
    STATE_WAIT_HEAD1,
    STATE_WAIT_HEAD2,
    STATE_WAIT_LEN,
    STATE_WAIT_TYPE,
    STATE_WAIT_PAYLOAD,
    STATE_WAIT_CHECKSUM
} ParseState;

typedef struct {
    ParseState state;
    uint8_t length;
    uint8_t type;
    uint8_t buf[32];
    uint8_t index;
} FrameParser;

FrameParser parser = { STATE_WAIT_HEAD1, 0, 0, {0}, 0 };


void HandleFrame(void)
{
    if(parser.type == 0x01)   // 速度指令
    {
        float temp_vx[4], temp_wz[4];//临时变量
        
        // 先把 buf 数据拷贝到临时变量（无警告）
        memcpy(&temp_vx, &parser.buf[0], 4);
        memcpy(&temp_wz, &parser.buf[4], 4);
        
        // 再把临时变量的值赋值给 volatile 变量（直接赋值，符合 volatile 语义）
        ros_vx = *(float*)(temp_vx);
        ros_wz = *(float*)(temp_wz);
				memset(temp_vx,0,sizeof(temp_vx));//清空缓存数组
				memset(temp_wz,0,sizeof(temp_wz));//清空缓存数组
    }
}
void check_sum(uint8_t byte)
{
	uint8_t checksum = 0;
	checksum ^= parser.length;
	checksum ^= parser.type;
	for(int i = 0; i < parser.index - 1; i++)
			checksum ^= parser.buf[i];

	if(checksum == byte)
	{
			HandleFrame();                          // 解析成功
	}
	parser.state = STATE_WAIT_HEAD1;            // 重置等待下一帧
}
void ParseByte(uint8_t byte)
{
    switch(parser.state)
    {
			case STATE_WAIT_HEAD1:
					if(byte == 0xAA)
							parser.state = STATE_WAIT_HEAD2;
					break;

			case STATE_WAIT_HEAD2:
					if(byte == 0x55)
							parser.state = STATE_WAIT_LEN;
					else
							parser.state = STATE_WAIT_HEAD1;
					break;

			case STATE_WAIT_LEN:
					parser.length = byte;
					parser.index = 0;
					memset(parser.buf, 0, sizeof(parser.buf));   // 必须清空
					parser.state = STATE_WAIT_TYPE;
					break;

			case STATE_WAIT_TYPE:
					parser.type = byte;
					parser.state = STATE_WAIT_PAYLOAD;
					break;

            case STATE_WAIT_PAYLOAD:
                    if(parser.index < sizeof(parser.buf))       // 保护防溢出
                    {
                            parser.buf[parser.index++] = byte;
                    }
                    if(parser.index >= parser.length - 1)        // payload_len = length - 1
                    {
                            parser.state = STATE_WAIT_CHECKSUM;
                    }
                    break;

            case STATE_WAIT_CHECKSUM:
                    check_sum(byte);
                    break;
            default:break;
    }
}



/**
 * @brief 数据发送任务（50Hz）
 */
void odom_msg_task(void)
{
    odom.head = 0xAA55;  // 帧头
		odom.v = 0;
		odom.w = 0;
		odom.x = 0;
		odom.y = 0;
    uint16_t i;
    uint32_t checksum = 0;
		
    uint8_t txBuffer[sizeof(odom)];  // 缓冲区大小等于结构体大小
    
    while(1)
    {    

			// === 调试：检查原始数据 ===
        // 计算校验和前先将最新数据复制到数组
				osMutexWait(odom_mutex, osWaitForever); //互斥锁加锁
        memcpy(txBuffer, &odom, sizeof(odom));
        
			
        // 重新计算校验和（跳过2字节头和2字节校验和）
        checksum = 0;
        for(i = 2; i < sizeof(odom) - 2; i++)
        {
            checksum += txBuffer[i]; 
        }
        odom.checksum = checksum;  // 更新结构体中的校验和
        
        // 数据更新至发送缓冲区
        memcpy(txBuffer, &odom, sizeof(odom));
				osMutexRelease(odom_mutex); //解锁
				#if 0     //此处printf会与dma冲突，若需打印，需要换其他串口
				static float count = 0;
				if(count++ >= 25)
				{
					count = 0;
//					printf(" odom.v:%.3f, odom.w:%.3f, odom.yaw:%.3f, odom.x:%.3f, odom.y:%.3f\n",odom.v,odom.w,odom.yaw,odom.x,odom.y);
					printf(" odom.acc:%.3f, %.3f,%.3f, odom.gyro:%.3f, %.3f,%.3f,  odom.ins:%.3f,%.3f,%.3f\n"
					,odom.Accel[0],odom.Accel[1],odom.Accel[2],odom.Gyro[0],odom.Gyro[1],odom.Gyro[2],odom.Pitch,odom.Roll,odom.Yaw);
				}
				#else
        HAL_UART_Transmit_DMA(&huart1, txBuffer, sizeof(odom));
        #endif
        osDelay(20);
    }
}

/*
*brief 里程计数据更新
*/
void odom_update(chassis_t *chassis,INS_t *ins,float dt)
{
	float vR = -chassis->wheel_motor[0].para.vel * WHEEL_RADIUS;  // right wheel m/s
	float vL =  chassis->wheel_motor[1].para.vel * WHEEL_RADIUS;  // left wheel m/s
	float v = 0.5f * (vR + vL);                                   // forward speed m/s

	if(v > -0.01f && v < 0.01f)
	{
		v = 0.0f;
	}

	// Keep odom internally self-consistent: gyro drives yaw continuity, wheel speed drives x/y.
	odom.w = ins->Gyro[2];
	odom.yaw += odom.w * dt;
	odom.v = v;

	float v_cos = arm_cos_f32(odom.yaw);
	float v_sin = arm_sin_f32(odom.yaw);
	odom.x += odom.v * v_cos * dt;
	odom.y += odom.v * v_sin * dt;
}
/*
*brief IMU 数据更新
*/
void imu_update(void)
{
	memcpy(&odom.Gyro,&INS.Gyro,sizeof(odom.Gyro));	// imu角速度
	memcpy(&odom.Accel,&INS.Accel,sizeof(odom.Accel));	//imu加速度
	odom.Pitch = INS.Pitch;
	odom.Roll = INS.Roll;
	odom.Yaw = INS.Yaw;  //此处的yaw是需要跳变的（范围在-π~π）
}
/**
 * @brief 数据更新任务（必须保证高精度 50 Hz）
 */
void odom_update_task(void)
{
	while(1)
    {  
			osMutexWait(odom_mutex, osWaitForever);  //加锁
			odom_update(&chassis_move,&INS,0.02f);//更新里程计数据（任务周期20ms）
			imu_update();	//imu数据更新
			osMutexRelease(odom_mutex); //解锁
			osDelay(20);
		}
}






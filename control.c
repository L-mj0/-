/*------------------------------------------------------*/
/*                      头文件加载                       */
/*======================================================*/
#include "zf_gtm_pwm.h"
#include "zf_gpio.h"
#include "SEEKFREE_ICM20602.h"
#include "SEEKFREE_IPS114_SPI.h"
#include "CAM.h"
#include "pid.h"
#include "control.h"
/*------------------------------------------------------*/
/*                       类型定义                       */
/*======================================================*/
enum PIN_INFO_Enum
{
    // 舵机
    STEERING_PIN = ATOM0_CH1_P33_9, // 舵机引脚
    STEERING_FREQUENCY = 50,        // 舵机频率
    STEERING_INITIAL_DUTY = 624,    // 舵机初始占空比 | 同时也是中值占空比

    // 电机
    MOTOR_ENABLE_DEFAULT = 0,         //? 默认电机开启或关闭
    MOTOR_LEFT_PWM = ATOM1_CH4_P02_4, // 左电机占空比
    MOTOR_LEFT_DIR = P02_5,           // 左电机方向
    MOTOR_LEFT_ENABLE = P33_10,       // 左电机使能
    MOTOR_LEFT_DIR_FRONT = 1,         // 左电机正转

    MOTOR_RIGHT_PWM = ATOM1_CH6_P02_6, // 右电机占空比
    MOTOR_RIGHT_DIR = P02_7,           // 右电机方向
    MOTOR_RIGHT_ENABLE = P33_11,       // 右电机使能
    MOTOR_RIGHT_DIR_FRONT = 1,         // 右电机正转

    MOTOR_INITIAL_DUTY = 2500, // 电机初始占空比
} PIN_INFO_Enum;
/*------------------------------------------------------*/
/*                       函数声明                       */
/*======================================================*/
static void runInStraightaway(void);
static void runInCrossroadLoop(void);
/*------------------------------------------------------*/
/*                       变量定义                       */
/*======================================================*/
parameterPID_t *steering;
parameterPID_t *gyroscope;
float gyroZ;
void (*racerRunInWay[])(void) = {runInStraightaway, runInCrossroadLoop};
float tracerError; // 循迹误差
int traceRow = 50; // 循迹行
/*------------------------------------------------------*/
/*                       函数定义                       */
/*======================================================*/
/*------------------------------*/
/*          基础循迹模块        */
/*==============================*/
// 传入参数 | cameraError：摄像头误差
static inline void basicTracer(float cameraError)
{
    get_icm20602_gyro_spi();
    tracerError = cameraError;
    gyroZ = (float)icm_gyro_z;
    gyroscope->runPID(gyroscope);
    steering->setPoint = gyroscope->controllerOutput; // 目标角速度
    steering->runPID(steering);
    pwm_duty(STEERING_PIN, STEERING_INITIAL_DUTY + steering->controllerOutput);
}
/*------------------------------*/
/*        十字回环循迹模块       */
/*==============================*/
static void runInCrossroadLoop(void)
{
    switch (actionState)
    {
    case crossroadLoopEnterR:
        break;
    case crossroadLoopInsideR:
        basicTracer(tracer.getCenterError(traceRow));
        break;
    case crossroadLoopExitR:
        pwm_duty(STEERING_PIN, STEERING_INITIAL_DUTY + 10);
        break;
    }
}
/*------------------------------*/
/*      直道、弯道循迹模块       */
/*==============================*/
static void runInStraightaway(void)
{
    get_icm20602_gyro_spi();
    basicTracer(tracer.getCenterError(traceRow));
}
/*------------------------------*/
/*        控制设备初始化        */
/*==============================*/
void controlInit(void)
{
    //? 舵机初始化
    gtm_pwm_init(STEERING_PIN, 50, STEERING_INITIAL_DUTY);
    //! PID初始化
    // steering = _parameterPID(2, 1, 0.01, 100, &tracerError, POSITION);
    gyroscope = _parameterPID(270, 1, 0.01, 10000, &tracerError, POSITION);
    steering = _parameterPID(0.01, 1, 0.01, 100, &gyroZ, POSITION);
    //? 电机初始化
    gtm_pwm_init(MOTOR_LEFT_PWM, 17000, MOTOR_INITIAL_DUTY);
    gpio_init(MOTOR_LEFT_DIR, GPO, MOTOR_LEFT_DIR_FRONT, PUSHPULL);
    gpio_init(MOTOR_LEFT_ENABLE, GPO, MOTOR_ENABLE_DEFAULT, PUSHPULL);

    gtm_pwm_init(MOTOR_RIGHT_PWM, 17000, MOTOR_INITIAL_DUTY);
    gpio_init(MOTOR_RIGHT_DIR, GPO, MOTOR_RIGHT_DIR_FRONT, PUSHPULL);
    gpio_init(MOTOR_RIGHT_ENABLE, GPO, MOTOR_ENABLE_DEFAULT, PUSHPULL);
    //? 陀螺仪初始化
    icm20602_init_spi();
}

//?
//?          _    _    _   __     __   _
//?         / _  / )  / )  ) )    ) ) )_)\_) (
//?        (__( (_/  (_/  /_/    /_/ / /  /  o
//?
//?
/*------------------------------------------------------*/
/*                      头文件加载                       */
/*======================================================*/
#include "stdlib.h"
#include "math.h"
#include "string.h"
#include "pid.h"
/*------------------------------------------------------*/
/*                       类型定义                       */
/*======================================================*/

/*------------------------------------------------------*/
/*                       函数声明                       */
/*======================================================*/

/*------------------------------------------------------*/
/*                       变量定义                       */
/*======================================================*/
// static parameterPID_t *instance;
/*------------------------------------------------------*/
/*                       函数定义                       */
/*======================================================*/
static void _errorHandler(void) // 错误处理函数 | 不会写，先写个while(1)吧
{
    while (1)
        ;
}

void updateSetPoint(struct parameterPID_t *param, object_t value)
{
    param->setPoint = value; // 更新目标值
}

object_t getControllerOutput(struct parameterPID_t *param)
{
    return param->controllerOutput; // 返回PID输出
}
/*------------------------------*/
/*           构造函数           */
/*==============================*/
parameterPID_t *_parameterPID(float Kp, float Ki, float Kd, object_t threshold, object_t *PV, PID_SELECT_Enum mode)
{
    // 申请内存空间
    parameterPID_t *p;
    p = (parameterPID_t *)malloc(sizeof(parameterPID_t));
    if (p == 0)
    {
        _errorHandler(); // *没有申请到空间，直接用的话会炸 | 启用错误处理函数
    }
    memset(p, 0, sizeof(parameterPID_t)); // 初始化内存空间
    // 函数初始化
    switch (mode)
    {
    case POSITION: // 选择的是位置式的PID
        p->runPID = positionPID;
        break;
    case AUGMENTED_TYPEC: // 选择的是增量式的PID | TypeC
        p->runPID = augmentedPIDTypeC;
        break;
    case AUGMENTED_TYPEB:
        p->runPID = augmentedPIDTypeB;
        break;
    }
    // 参数初始化
    p->Kp = Kp;
    p->Ki = Ki;
    p->Kd = Kd;
    p->alpha = 0.15;
    p->threshold = threshold;
    p->processValue = PV;
    return p;
}

/*----------------------*/
/*		增量PID模块		*/
/*======================*/
void augmentedPIDTypeB(struct parameterPID_t *param)
{
    //	变量定义
    object_t yn;
    //	保存和计算误差
    param->e3 = param->e2;
    param->e2 = param->e1;
    param->e1 = param->setPoint - *param->processValue;
    yn = param->I; // 保存上一次积分项计算值
    //	PID公式
    param->I = param->Ki * param->e1;
    //	一阶低通滤波（积分项
    param->I = param->alpha * param->I + (1.f - param->alpha) * yn;
    param->controllerOutput += param->Kp * (param->e1 - param->e2) + param->I + param->Kd * (param->e1 - 2 * param->e2 + param->e3);
    //	阈值限定
    if (fabs(param->controllerOutput) > param->threshold)
    {
        param->controllerOutput = (param->controllerOutput > 0) ? param->threshold : -param->threshold;
    }
}

/*------------------------------*/
/*           增量PID            */
/*==============================*/
void augmentedPIDTypeC(struct parameterPID_t *param)
{
    // 保存实际值、计算误差
    param->PV2 = param->PV1;
    param->PV1 = *param->processValue;
    param->e1 = param->setPoint - *param->processValue;
    // PID公式计算 | Type C
    param->controllerOutput += -param->Kp * (*param->processValue - param->PV1) + param->Ki * param->e1 - param->Kd * (*param->processValue - 2 * param->PV1 + param->PV2);
    // 阈值限制
    if (fabs(param->controllerOutput) > param->threshold)
    {
        param->controllerOutput = (param->controllerOutput > 0) ? param->threshold : -param->threshold;
    }
}

/*------------------------------*/
/*           位置式PID          */
/*==============================*/
void positionPID(struct parameterPID_t *param)
{
    param->e2 = param->e1;
    param->e1 = param->setPoint - *param->processValue;
    //	PID公式
    param->controllerOutput = (param->Kp) * param->e1 + param->Kd * (param->e1 - param->e2);
    //	阈值限定
    if (fabs(param->controllerOutput) > param->threshold)
    {
        param->controllerOutput = (param->controllerOutput > 0) ? param->threshold : -param->threshold;
    }
}

#ifndef _PID_H
#define _PID_H
/*------------------------------------------------------*/
/*                       类型定义                       */
/*======================================================*/
typedef enum PID_SELECT_Enum
{
    AUGMENTED_TYPEB = 2, // 增量式 | TypeB
    AUGMENTED_TYPEC = 1, // 增量式 | TypeC
    POSITION = 0,        // 位置式
} PID_SELECT_Enum;

typedef float object_t;

typedef struct parameterPID_t
{
    // 指针
    object_t (*getCO)(); // 获取计算结果
    void (*runPID)(struct parameterPID_t *);
    // 地址存储
    object_t *processValue; // 实际值
    // PID参数
    float Kp;
    float Ki;
    float Kd;
    float alpha; // 一阶低通滤波系数 | 数值越小输出变化越小
    // 计算相关
    object_t e1, e2, e3;
    object_t I; // 积分项
    object_t controllerOutput;
    object_t threshold; // 阈值
    object_t PV1;
    object_t PV2;
    object_t setPoint; // 目标值
} parameterPID_t;
/*------------------------------------------------------*/
/*                       外部声明                       */
/*======================================================*/

parameterPID_t *_parameterPID(float Kp, float Ki, float Kd, object_t threshold, object_t *PV, PID_SELECT_Enum mode);
void augmentedPIDTypeB(struct parameterPID_t *param);
void augmentedPIDTypeC(struct parameterPID_t *param);
void positionPID(struct parameterPID_t *param);

object_t getControllerOutput(struct parameterPID_t *param);
void updateSetPoint(struct parameterPID_t *param, object_t value);

#endif

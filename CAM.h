#ifndef _CAM_H
#define _CAM_H
/*------------------------------------------------------*/
/*                       类型定义                       */
/*======================================================*/
typedef enum ELEMENT_Enum // 元素枚举
{
    STRAIGHTAWAY = 0, // 直道（弯道）
    CROSSROAD_LOOP,   // 十字回环
} ELEMENT_Enum;
typedef enum STATE_Enum // 状态枚举
{
    straightAway, // 直道
    bend,         // 弯道

    crossroadLoopEnterR,  // 十字回环入口 | 右
    crossroadLoopInsideR, // 十字回环内 | 右
    crossroadLoopExitR,   // 十字回环出口 | 右
} STATE_Enum;
typedef struct tracer_t
{
    void (*otsu)();
    void (*binary)();
    void (*display)();
    void (*searchRightBorder)();
    void (*searchLeftBorder)();
    void (*stateDetection)();
    // 循迹相关
    int (*getCenterError)(int);
} tracer_t;
/*------------------------------------------------------*/
/*                       外部声明                       */
/*======================================================*/
extern tracer_t tracer;
extern ELEMENT_Enum elementFlag;
extern STATE_Enum stateFlag;
extern STATE_Enum actionState;

void cameraInit(void);
#endif

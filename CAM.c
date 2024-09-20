/*------------------------------------------------------*/
/*                      头文件加载                       */
/*======================================================*/
#include "stdint.h"
#include "stdbool.h"
#include "math.h"
#include "string.h"
#include "SEEKFREE_MT9V03X.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"
#include "SEEKFREE_IPS114_SPI.h"
#include "CAM.h"
/*------------------------------------------------------*/
/*                       类型定义                       */
/*======================================================*/
#define camImage (mt9v03x_image)                      // 图片数组宏定义
#define pointAt binaryImage[i][columnPosition]        // 边界搜索当前点
#define pointRight binaryImage[i][columnPosition + 1] // 边界搜索点右边的点

enum IAMGE_PROPERTIES_Enum
{
    imageColumn = MT9V03X_W,     // 图像列数 | x
    imageRow = MT9V03X_H,        // 图像行数 | y
    imageCenter = MT9V03X_W / 2, // 图像中心线
    grayscale = 256,             // 灰阶

    OTSU_SCALE_LEFT = 30,   // 大津法最优阈值范围 - 左边界 | 可以略去一些最优阈值不可能出现的地方，取值范围0~255
    OTSU_SCALE_RIGHT = 220, // 大津法最有阈值范围 - 右边界 | 同上

    CENTER_IMAGE_BIAS = 2,    // 图像居中对齐偏置
    CUTOFF_IMAGE_NUMBERS = 4, // 剪切掉的图像宽度（左右裁剪）
    COMPRESS_BITS = 8,        // 压缩至的数据大小 | byte = 8bit

    binaryImageColumn = (imageColumn - CUTOFF_IMAGE_NUMBERS) / COMPRESS_BITS, // 二值化图像列数 | 经过压缩的二值化
    BORDER_RIGHT_MOST = imageColumn - CUTOFF_IMAGE_NUMBERS - 1,               // 边界最右边
    BORDER_LEFT_MOST = 0,                                                     // 边界最左边

    DATUM_START_RIGHT = binaryImageColumn / 2,      // 压缩过后的图像搜索边界的起点（列）
    DATUM_START_LEFT = (binaryImageColumn / 2) - 2, // 搜索范围为16个位，故-2(2*8bit)

    HORIZONTAL_BORDER_SKIP_ROW = 10, // 水平边线跳过检测的行数
    HORIZONTAL_BORDER_HEIGHT = 5,    // 水平边线限制的行高
    HORIZONTAL_UPPER_LIMIT = 10,     // 水平边线搜索范围 | 距离上边界的偏移量
    HORIZONTAL_LOWER_LIMIT = 10,     // 水平边线搜索范围 | 距离下边界的偏移量

    STRAIGHTAWAY_EXTENSION_LENGTH = 10, // 直道延伸长度判断基准

    BEND_JUDGE_LIMIT = 5, // 弯道判断限制

    CROSSROAD_LOOP_UPPER_BIAS = 8, // 十字回环出口上边界端点偏置
    CROSSROAD_LOOP_LOWER_BIAS = 5, // 十字回环出口下边界端点偏置
};

enum SHOW_COLOR_Enum // 显示的颜色
{
    WHITE_COLOR = 0xFFFF,
    BLACK_COLOR = 0x0000,
    BLUE_001 = 0xAEFC,
    BLUE_002 = 0x292B,
    ORANGE_001 = 0xFD10,
    GREEN_001 = 0xA6AE,
    PINK_001 = 0xB20E,
};

// 大津法
typedef float otsu_t;
typedef struct image_t
{
    // 大津法计算值
    otsu_t P[grayscale];
    otsu_t PK[grayscale];
    otsu_t MK[grayscale];

    otsu_t imageSize;  // 图像尺寸
    uint8_t threshold; // 二值化阈值
} image_t;

// 图像边界
typedef enum DATUM_MARK_FLAG_Enum
{
    DATUM_BOTTOM_NORMAL,
    DATUM_BOTTOM_IN_CENTER,
    DATUM_ABOVE,
} DATUM_MARK_FLAG_Enum;
typedef struct vertical_t // 竖直边界
{
    int datumMarkRight[2]; // 右基准点行列
    int datumMarkLeft[2];  // 左基准点行列

    int borderRight[imageRow]; // 竖直右边界 | 行为索引，元素为列
    int cutRowRight;           // 竖直右边界截止行

    int borderLeft[imageRow]; // 竖直左边界 | 行为索引，元素为列
    int cutRowLeft;           // 竖直左边界截止行
    DATUM_MARK_FLAG_Enum datumFlag;
} vertical_t;
typedef struct horizontal_t
{
    bool exist;    // 是否存在出口
    int topRow;    // 白到黑跳变行
    int bottomRow; // 黑到白跳变行

    int upper[BORDER_RIGHT_MOST + 1]; // 水平上边界
    int upperCut;                     // 上边界截止
    int lower[BORDER_RIGHT_MOST + 1]; // 水平下边界
    int lowerCut;                     // 下边界截止
} horizontal_t;
/*------------------------------------------------------*/
/*                       函数声明                       */
/*======================================================*/
static void otsu(void);
static void stateMachineEnter(void);
static void stateMachineCrossroadLoop(void);
/*------------------------------------------------------*/
/*                       变量定义                       */
/*======================================================*/
static image_t cam;                               // 摄像头结构体
static vertical_t vertical;                       // 垂直边界结构体
horizontal_t horizonLeft;                         // 水平左边界
horizontal_t horizonRight;                        // 水平右边界
tracer_t tracer;                                  // 追踪者
uint8_t binaryImage[imageRow][binaryImageColumn]; // 二值化图像
uint16_t displayColor = BLUE_001;                 // 二值化图像显示颜色
ELEMENT_Enum elementFlag = STRAIGHTAWAY;          // 元素标志位 | 表示车子现在处于哪一个元素内
STATE_Enum stateFlag = straightAway;              // 状态标志位 | 表示车子当前识别出的元素状态
STATE_Enum actionState = straightAway;            // 运行状态标志位 | 表示车子当前工作在的元素状态

static void (*stateMachine[])(void) = {stateMachineEnter, stateMachineCrossroadLoop}; // 状态机 | 对应elementFlag | 在前后检测状态不同时调用
/*------------------------------------------------------*/
/*                       函数定义                       */
/*======================================================*/
#define vbLeft vertical.borderLeft
/*------------------------------*/
/*           直线检测           */
/*==============================*/
// 传入参数 | bottomRow：检测范围左边界 | topRow：右边界 | maxSlope：允许的最大斜率 | minSlope：允许的最小斜率 | diffAllow：允许的最大斜率差
static bool isStraightLine4VertLeft(int bottomRow, int topRow, float maxSlope, float minSlope, float diffAllow) // 左垂直边线的直线检测
{
    // 先判断传入范围是否合法
    if (topRow < vertical.cutRowLeft)
    {
        topRow = vertical.cutRowLeft; // 不符合的时候默认使用左截止行
    }
    if (bottomRow > vertical.datumMarkLeft[0])
    {
        bottomRow = vertical.datumMarkLeft[0]; // 不符合的时候默认使用基准点所在行
    }
    int sp[4];                   // 选中用于判断的点 | 元素为行
    sp[0] = bottomRow;           // 最底下的点
    sp[3] = topRow;              // 最顶上的点
    sp[1] = (sp[0] + sp[3]) / 2; // 中间点
    sp[2] = (sp[1] + sp[3]) / 2; // 中间靠近顶点
    sp[1] = (sp[1] + sp[0]) / 2; // 中间靠近底点
    //? 单调性判断
    bool isMonotone = false;
    if ((vbLeft[sp[0]] < vbLeft[sp[1]]))
    {
        if (vbLeft[sp[1]] < vbLeft[sp[2]])
        {
            if (vbLeft[sp[2]] < vbLeft[sp[3]])
            {
                isMonotone = true;
            }
        }
    }
    if (isMonotone == true) // 符合单调递增
    {
        float slope[2];                                                             // 定义变量，存储斜率
        slope[0] = (float)(vbLeft[sp[3]] - vbLeft[sp[1]]) / (float)(sp[1] - sp[3]); // 注意正负号
        slope[1] = (float)(vbLeft[sp[2]] - vbLeft[sp[0]]) / (float)(sp[0] - sp[2]); // 注意正负号
        float averageSlope = (slope[0] + slope[1]) / 2;                             // 计算平均斜率
        float differenceSlope = fabs(slope[0] - slope[1]);                          // 计算斜率差
        // ips114_showfloat(0, 6, averageSlope, 3, 3);
        // ips114_showfloat(0, 7, differenceSlope, 3, 3);
        if ((averageSlope < maxSlope) && (averageSlope > minSlope)) // 判断平均斜率是否在许可范围内
        {
            if (differenceSlope < diffAllow) // 判断斜率差是否在允许范围内
            {
                return true; // 符合所有条件，是直线
            }
        }
    }
    return false;
}
//?/*------------------------------*/
//?/*           状态检测           */
//?/*==============================*/
static void stateDetection(void)
{
    // 变量定义
    static STATE_Enum stateFlagTemp = straightAway;
    //? 状态初始化
    if (stateFlag != stateFlagTemp) // 前后状态不一致 | 状态机状态转变触发
    {
        stateMachine[elementFlag](); // 调用状态机
    }
    stateFlagTemp = stateFlag;
    stateFlag = straightAway; // 默认为直道（弯道）
    //! 一些特殊状态的退出检测
    switch (actionState)
    {
    case crossroadLoopEnterR:                                                                     // 十字回环入口（右）
        if (vertical.borderLeft[vertical.cutRowLeft + 1] >= BORDER_RIGHT_MOST - BEND_JUDGE_LIMIT) // 判断是否进入弯道
        {
            stateFlag = crossroadLoopInsideR;
            return;
        }
        break;
    case crossroadLoopInsideR: // 十字回环内（右）
        if ((horizonLeft.exist == true) && (horizonRight.exist == true))
        {
            stateFlag = crossroadLoopExitR;
            return;
        }
        break;
    }
    //! 状态检测
    //? 岔道预留

    //? 左右都有出口 | 十字 | 十字回环出环

    //? 只有左边有出口 | 左环 | 十字回环（左）

    //? 只有右边有出口 | 右环 | 十字回环（右）
    if ((horizonLeft.exist == false) && (horizonRight.exist == true))
    {
        if (horizonLeft.topRow - vertical.cutRowRight > STRAIGHTAWAY_EXTENSION_LENGTH) // 判断直道是否延伸 | 右边界截止点要高于左出口顶行
        {
            //? 判断是否为直线
            if (isStraightLine4VertLeft(vertical.datumMarkLeft[0], horizonRight.topRow, 0.65f, 0.45f, 0.15f) == true)
            {
                int upperDiff = abs(horizonRight.upper[horizonRight.upperCut + CROSSROAD_LOOP_UPPER_BIAS] - horizonRight.upper[BORDER_RIGHT_MOST]); // 计算上边界两点高度差
                int lowerDiff = abs(horizonRight.lower[horizonRight.lowerCut + CROSSROAD_LOOP_LOWER_BIAS] - horizonRight.lower[BORDER_RIGHT_MOST]); // 计算下边界两点高度差
                int heightRight = horizonRight.bottomRow - horizonRight.topRow;                                                                     // 计算右边的上下边界高度差
                int heightLeft = horizonRight.lower[horizonRight.lowerCut + CROSSROAD_LOOP_LOWER_BIAS] - horizonRight.upper[horizonRight.upperCut + CROSSROAD_LOOP_UPPER_BIAS];
                if (upperDiff < 4 && lowerDiff < 4)
                {
                    if (abs(heightRight - heightLeft) < 5)
                    {
                        stateFlag = crossroadLoopEnterR;
                    }
                }
                // ips114_showint16(imageColumn, 0, upperDiff);
                // ips114_showint16(imageColumn, 1, lowerDiff);
                // ips114_showint16(imageColumn, 2, heightRight);
                // ips114_showint16(imageColumn, 3, heightLeft);
            }
        }
    }
    //? 显示检测出来的状态
//    switch (actionState)
//    {
//    case straightAway:
//        ips114_showstr(0, 6, "straightAway         ");
//        break;
//    case bend:
//        ips114_showstr(0, 6, "bend                 ");
//        break;
//    case crossroadLoopEnterR:
//        ips114_showstr(0, 6, "crossroadLoopEnterR  ");
//        break;
//    }
//    switch (elementFlag)
//    {
//    case STRAIGHTAWAY:
//        ips114_showstr(0, 7, "STRAIGHTAWAY         ");
//        break;
//    case CROSSROAD_LOOP:
//        ips114_showstr(0, 7, "CROSSROAD_LOOP       ");
//        break;
//    }
}

static void stateMachineCrossroadLoop(void)
{
    switch (actionState)
    {
    case crossroadLoopEnterR:
        if (stateFlag == crossroadLoopInsideR)
        {
            actionState = crossroadLoopInsideR;
            displayColor = BLUE_001;
        }
        break;
    case crossroadLoopInsideR:
        if (stateFlag == crossroadLoopExitR)
        {
            actionState = crossroadLoopExitR;
            displayColor = BLUE_002;
        }
        break;
    }
}

static void stateMachineEnter(void) // 状态机入口
{
    switch (stateFlag)
    {
    case crossroadLoopEnterR: // 十字回环入口
        elementFlag = CROSSROAD_LOOP;
        actionState = crossroadLoopEnterR;
        displayColor = BLUE_002;
        break;
    }
}
/*------------------------------*/
/*         返回边界中点         */
/*==============================*/
static int getCenterError(int row)
{
    if (row > 0 && row < imageRow)
    {
        int center = (float)(vertical.borderLeft[row] + vertical.borderRight[row]) / 2.f;
        return center - imageCenter;
    }
    return 0; // 不在可选范围内
}
#define pointUpper binaryImage[j - 1][i] // 所在点上方的点
#define pointMiddle binaryImage[j][i]    // 水平边线搜索所在点
#define pointLower binaryImage[j + 1][i] // 所在点下方的点
/*------------------------------*/
/*       水平边线搜索（左）      */
/*==============================*/
static void searchHBLeft(void)
{
    // 初始化边界截止点
    horizonLeft.upperCut = BORDER_LEFT_MOST;
    horizonLeft.lowerCut = BORDER_LEFT_MOST;
    int exitRow = (horizonLeft.topRow + horizonLeft.bottomRow) / 2; // 计算出口所在行 | 出口是边缘黑白跳变的中间点
    // 搜索下边界
    for (int i = 0; i < binaryImageColumn - 1; i++)
    {
        uint8_t foundFlag = 0x00; // 搜寻标志，用于标记已经找到的点
        for (int j = exitRow; j < horizonLeft.bottomRow + HORIZONTAL_LOWER_LIMIT; j++)
        {
            uint8_t xorResult = pointMiddle ^ pointLower; // 异或运算结果，用于存储找到的黑白跳变点
            for (int k = COMPRESS_BITS - 1; k > -1; k--)
            {
                if (((foundFlag >> k) & 0x01) == 0x00) // 先判断该位置上的点是否已经找到边界，防止同一列找出两个边界
                {
                    if (((xorResult >> k) & 0x01) == 0x01) // 相应位置找到新的点
                    {
                        int position = (i * COMPRESS_BITS) + COMPRESS_BITS - 1 - k;
                        horizonLeft.lower[position] = j; // 存入水平边线所在行数
                        if (position > horizonLeft.lowerCut)
                        {
                            horizonLeft.lowerCut = position;
                        }
                    }
                }
            }
            foundFlag |= xorResult; // 保存获取结果
            if (foundFlag == 0xFF)  // 所有点都找到时跳转列
            {
                break;
            }
            if (pointLower == 0x00) // 遇到全黑的点时跳转列
            {
                break;
            }
        }
        if (foundFlag != 0xFF) // 找不满所有点时停止搜索
        {
            break;
        }
    }
    // 搜索上边线
    for (int i = 0; i < binaryImageColumn - 1; i++)
    {
        uint8_t foundFlag = 0x00; // 搜寻标志，用于标记已经找到的点
        for (int j = exitRow; j > horizonLeft.topRow - HORIZONTAL_UPPER_LIMIT; j--)
        {
            uint8_t xorResult = pointMiddle ^ pointUpper; // 异或运算结果，用于存储找到的黑白跳变点
            for (int k = COMPRESS_BITS - 1; k > -1; k--)
            {
                if (((foundFlag >> k) & 0x01) == 0x00) // 先判断该位置上的点是否已经找到边界，防止同一列找出两个边界
                {
                    if (((xorResult >> k) & 0x01) == 0x01) // 相应位置找到新的点
                    {
                        int position = (i * COMPRESS_BITS) + COMPRESS_BITS - 1 - k;
                        horizonLeft.upper[position] = j; // 存入水平边线所在行数
                        if (position > horizonLeft.upperCut)
                        {
                            horizonLeft.upperCut = position;
                        }
                    }
                }
            }
            foundFlag |= xorResult; // 保存获取结果
            if (foundFlag == 0xFF)  // 所有点都找到时跳转列
            {
                break;
            }
            if (pointLower == 0x00) // 遇到全黑的点时跳转列
            {
                break;
            }
        }
        if (foundFlag != 0xFF) // 找不满所有点时停止搜索
        {
            break;
        }
    }
}
/*------------------------------*/
/*       水平边线搜索（右）      */
/*==============================*/
static void searchHBRight(void)
{
    // 初始化边界截止点
    horizonRight.upperCut = BORDER_RIGHT_MOST;
    horizonRight.lowerCut = BORDER_RIGHT_MOST;
    int exitRow = (horizonRight.topRow + horizonRight.bottomRow) / 2; // 计算出口所在行 | 出口是边缘黑白跳变的中间点
    // 搜索下边界
    for (int i = binaryImageColumn - 1; i > -1; i--)
    {
        uint8_t foundFlag = 0x00; // 搜寻标志，用于标记已经找到的点
        for (int j = exitRow; j < horizonRight.bottomRow + HORIZONTAL_LOWER_LIMIT; j++)
        {
            uint8_t xorResult = pointMiddle ^ pointLower; // 异或运算结果，用于存储找到的黑白跳变点
            for (int k = COMPRESS_BITS - 1; k > -1; k--)
            {
                if (((foundFlag >> k) & 0x01) == 0x00) // 先判断该位置上的点是否已经找到边界，防止同一列找出两个边界
                {
                    if (((xorResult >> k) & 0x01) == 0x01) // 相应位置找到新的点
                    {
                        int position = (i * COMPRESS_BITS) + COMPRESS_BITS - 1 - k;
                        horizonRight.lower[position] = j; // 存入水平边线所在行数
                        if (position < horizonRight.lowerCut)
                        {
                            horizonRight.lowerCut = position;
                        }
                    }
                }
            }
            foundFlag |= xorResult; // 保存获取结果
            if (foundFlag == 0xFF)  // 所有点都找到时跳转列
            {
                break;
            }
            if (pointLower == 0x00) // 遇到全黑的点时跳转列
            {
                break;
            }
        }
        if (foundFlag != 0xFF) // 找不满所有点时停止搜索
        {
            break;
        }
    }
    // 搜索上边线
    for (int i = binaryImageColumn - 1; i > -1; i--)
    {
        uint8_t foundFlag = 0x00; // 搜寻标志，用于标记已经找到的点
        for (int j = exitRow; j > horizonRight.topRow - HORIZONTAL_UPPER_LIMIT; j--)
        {
            uint8_t xorResult = pointMiddle ^ pointUpper; // 异或运算结果，用于存储找到的黑白跳变点
            for (int k = COMPRESS_BITS - 1; k > -1; k--)
            {
                if (((foundFlag >> k) & 0x01) == 0x00) // 先判断该位置上的点是否已经找到边界，防止同一列找出两个边界
                {
                    if (((xorResult >> k) & 0x01) == 0x01) // 相应位置找到新的点
                    {
                        int position = (i * COMPRESS_BITS) + COMPRESS_BITS - 1 - k;
                        horizonRight.upper[position] = j; // 存入水平边线所在行数
                        if (position < horizonRight.upperCut)
                        {
                            horizonRight.upperCut = position;
                        }
                    }
                }
            }
            foundFlag |= xorResult; // 保存获取结果
            if (foundFlag == 0xFF)  // 所有点都找到时跳转列
            {
                break;
            }
            if (pointLower == 0x00) // 遇到全黑的点时跳转列
            {
                break;
            }
        }
        if (foundFlag != 0xFF) // 找不满所有点时停止搜索
        {
            break;
        }
    }
}
/*------------------------------*/
/*        边界点限制模块        */
/*==============================*/
static void verticalLimit(void)
{
    //! 一段神秘的代码，似乎三个if都执行之后就不调用水平边界搜索函数
    if (vertical.cutRowLeft > 40)
    {
        if (vertical.borderLeft[vertical.cutRowRight - 3] > 10)
        {
            if (vertical.borderRight[vertical.cutRowRight] < 20)
            {
                vertical.cutRowLeft = vertical.cutRowRight + 1;
                for (int i = 0; i < vertical.cutRowLeft; i++)
                {
                    vertical.borderLeft[i] = BORDER_LEFT_MOST;
                    vertical.borderRight[i] = BORDER_LEFT_MOST;
                }
            }
        }
    }
    if (vertical.cutRowRight > 40)
    {
        if (vertical.borderRight[vertical.cutRowLeft - 3] > 10)
        {
            if (vertical.borderLeft[vertical.cutRowLeft] < 20)
            {
                vertical.cutRowRight = vertical.cutRowLeft + 1;
                for (int i = 0; i < vertical.cutRowRight; i++)
                {
                    vertical.borderLeft[i] = BORDER_RIGHT_MOST;
                    vertical.borderRight[i] = BORDER_RIGHT_MOST;
                }
            }
        }
    }
}
/*------------------------------*/
/*	  基准点精准搜寻模块（左） 	 */
/*==============================*/
//? 返回枚举类型
static inline void accuratePointFindLeft(const uint8_t *const p, int const position)
{
    // 黑白转变点在左边八位的情况
    vertical.datumMarkLeft[0] = imageRow - 1; // 基准点所在行
    if ((*p != 0xFF) && (*p != 0x00))
    {
        vertical.datumFlag = DATUM_BOTTOM_NORMAL;
        vertical.datumMarkLeft[1] = position; // 基准点所在列（压缩图像中的所在列）
        return;
    }
    // 右边八位
    if ((*(p + 1) != 0xFF) && (*(p + 1) != 0x00))
    {
        vertical.datumFlag = DATUM_BOTTOM_NORMAL;
        vertical.datumMarkLeft[1] = position + 1; // 基准点所在列，在右边，所以+1
        return;
    }
    // 左黑右白
    vertical.datumFlag = DATUM_BOTTOM_IN_CENTER;
    vertical.datumMarkLeft[1] = position + 1;
}
/*------------------------------*/
/*      基准点寻找模块（左）     */
/*==============================*/
//? 传入二值化图像，返回基准点粗略坐标及寻找模式
static void datumMarkLeft(void)
{
    // 起始点，从图像最底下的行开始搜索边界
    for (int i = DATUM_START_LEFT; i > -1; i--)
    {
        if (binaryImage[imageRow - 1][i] == 0xFF && binaryImage[imageRow - 1][i + 1] == 0xFF) // 全白的情况，持续跳过
        {
            continue;
        }
        accuratePointFindLeft(&binaryImage[imageRow - 1][i], i); // 基准点在当前行，调用函数判断基准点的具体位置
        return;
    }
    // 在当前行搜索不到基点，开始向上搜索
    for (int i = imageRow - 1; i > 0; i--)
    {
        if ((binaryImage[i][0] ^ binaryImage[i - 1][0]) == 0x00)
        {
            continue;
        }
        vertical.datumFlag = DATUM_ABOVE;
        vertical.datumMarkLeft[0] = i; // 记录基准点所在行
        vertical.datumMarkLeft[1] = 0; // 存入基准点所在列（经过压缩的图像）
        return;
    }
}
//?/*------------------------------*/
//?/*          左边界寻找          */
//?/*==============================*/
static bool accurateBorderFindLeft(int i, int columnPosition) // 辅助找边界点 | 找到返回true
{
    if ((pointAt != 0x00) && (pointAt != 0xFF)) // 当前8个点存在边界
    {
        //? 寻找边界准确位置
        for (int j = COMPRESS_BITS - 1; j > 0; j--)
        {
            if (((pointAt >> j) & 0x01) != (pointAt >> (j - 1) & 0x01)) // 遍历八个点，找到黑白转变点
            {
                vertical.borderLeft[i] = columnPosition * COMPRESS_BITS + (COMPRESS_BITS - j); // 计算得到基准点准确位置
                break;
            }
        }
        return true;
    }
    if ((pointRight != 0x00) && (pointRight != 0xFF)) // 当前点右边8个点存在边界
    {
        //? 寻找边界准确位置
        for (int j = COMPRESS_BITS - 1; j > 0; j--)
        {
            if (((pointRight >> j) & 0x01) != (pointRight >> (j - 1) & 0x01)) // 遍历八个点，找到黑白转变点
            {
                vertical.borderLeft[i] = (columnPosition + 1) * COMPRESS_BITS + (COMPRESS_BITS - j); // 计算得到基准点准确位置
                break;
            }
        }
        return true;
    }
    if ((pointAt == 0x00) && (pointRight == 0xFF)) // 左边全黑，右边全白
    {
        vertical.borderLeft[i] = (columnPosition * COMPRESS_BITS + COMPRESS_BITS - 1);
        return true;
    }
    if ((pointAt == 0xFF) && (pointRight == 0x00)) //? 左边全白，右边全黑  | 不太可能出现的情况
    {
        vertical.borderLeft[i] = ((columnPosition + 1) * COMPRESS_BITS);
        return true;
    }
    return false;
}

//? 传入二值化图像，传入边界数组源地址
static void searchLeftBorder(void)
{
    // 定位基准点
    datumMarkLeft();                                // 调用函数，寻找基准点的粗略位置
    int rowPosition = vertical.datumMarkLeft[0];    // 基准点所在行
    int columnPosition = vertical.datumMarkLeft[1]; // 基准点所在列（经过压缩的图像）
    // 数值初始化
    vertical.cutRowLeft = vertical.datumMarkLeft[0] - 1;
    horizonLeft.exist = false;        // 水平边界初始化
    horizonLeft.bottomRow = imageRow; //! horizontal_t 作为参数传入会比较合适
    horizonLeft.topRow = imageRow;
    // 分情况找到基准点具体位置
    switch (vertical.datumFlag)
    {
    case DATUM_BOTTOM_NORMAL: // 一般情况，边界为八个点的某一个点
        for (int i = COMPRESS_BITS - 1; i > 0; i--)
        {
            if (((pointAt >> i) & 0x01) != (pointAt >> (i - 1) & 0x01)) // 遍历八个点，找到黑白转变点
            {
                vertical.borderLeft[rowPosition] = columnPosition * COMPRESS_BITS + (COMPRESS_BITS - i); // 计算得到基准点准确位置
                break;
            }
        }
        break;
    case DATUM_BOTTOM_IN_CENTER: // 基准点在16个点中间的情况
        vertical.borderLeft[rowPosition] = columnPosition * COMPRESS_BITS;
        break;
    case DATUM_ABOVE:
        for (int i = COMPRESS_BITS - 1; i > 0; i--)
        {
            if (((pointAt >> i) & 0x01) != (pointAt >> (i - 1) & 0x01)) // 遍历八个点，找到黑白转变点
            {
                vertical.borderLeft[rowPosition] = columnPosition * COMPRESS_BITS + (COMPRESS_BITS - i); // 计算得到基准点准确位置
                break;
            }
        }
        for (int i = imageRow - 1; i > rowPosition; i--)
        {
            vertical.borderLeft[i] = BORDER_LEFT_MOST;
        }
        break;
    }
    // 水平边界搜索标志位
    bool isEdgeWhite = false;
    bool isEdgeWhiteTemp = false;
    // 向上搜索边界 | 这里直接拿基准点坐标当作搜索点坐标了
    for (int i = rowPosition - 1; i > 0; i--) // 从基准点上方一行开始搜索
    {
        vertical.cutRowLeft = i; // 保存搜索停止行
        //! 判断是否存在边界跳变
        if ((isEdgeWhite != isEdgeWhiteTemp) && (i < imageRow - HORIZONTAL_BORDER_SKIP_ROW)) // 存在黑白跳变且行高高于底部10行
        {
            if (horizonLeft.exist == false) // 尚未找到边界
            {
                if (isEdgeWhiteTemp == false && isEdgeWhite == true) // 前一行是黑，后一行是白
                {
                    horizonLeft.bottomRow = i + 1; // 这里提前换行了，找到的跳变在上一行
                }
                else // 既然有跳变，那不是前黑后白，就是前白后黑了
                {
                    if (horizonLeft.bottomRow != imageRow) // 先判断是否有边界黑到白跳变
                    {
                        horizonLeft.topRow = i + 1;                                                // 存储跳变所在行
                        if (horizonLeft.bottomRow - horizonLeft.topRow > HORIZONTAL_BORDER_HEIGHT) // 判断上下行离得足够远 | 这里因为原点的原因，top坐标会比较小
                        {
                            horizonLeft.exist = true; // 满足所有条件，存在水平边界
                            searchHBLeft();           //! 调用水平边线搜索函数，临时这样写
                        }
                        else // 不满足高度要求，重置
                        {
                            horizonLeft.bottomRow = imageRow;
                            horizonLeft.topRow = imageRow;
                        }
                    }
                }
            }
        }
        // 重置水平搜索标志位
        isEdgeWhiteTemp = isEdgeWhite;
        isEdgeWhite = false;
        //! 寻找边界
        if (accurateBorderFindLeft(i, columnPosition) == true)
        {
            continue; // 找到后换一行搜索
        }
        // 16个点全为黑的情况
        if ((pointAt == 0x00) && (pointRight == 0x00))
        {
            for (columnPosition++; columnPosition < binaryImageColumn - 1; columnPosition++) // 初始右移8bit，之后持续右移，直到找到边界
            {
                if ((pointAt == 0x00) && (pointRight == 0x00)) // 依旧是全黑，继续向右遍历
                {
                    continue;
                }
                else
                {
                    break;
                }
            }
            if (columnPosition == binaryImageColumn - 1) // 抵达最边界，停止搜索
            {
                vertical.cutRowLeft = i;
                //? 图像边缘黑白检测预留
                if ((horizonLeft.exist == false) && (isEdgeWhiteTemp == true))
                {
                    if (horizonLeft.bottomRow != imageRow) // 先判断是否有边界黑到白跳变
                    {
                        horizonLeft.topRow = i;                                                    // 存储跳变所在行 | 这里没有提前换行，所以不+1
                        if (horizonLeft.bottomRow - horizonLeft.topRow > HORIZONTAL_BORDER_HEIGHT) // 判断上下行离得足够远 | 这里因为原点的原因，top坐标会比较小
                        {
                            horizonLeft.exist = true; // 满足所有条件，存在水平边界
                            searchHBLeft();           //! 调用水平边线搜索函数，临时这样写
                        }
                    }
                }
                return;
            }
            //! 找边界具体位置 | 好像有更简单的写法
            if (accurateBorderFindLeft(i, columnPosition) == true)
            {
                continue;
            }
        }
        // 16个点全为白的情况
        if ((pointAt == 0xFF) && (pointRight == 0xFF))
        {
            for (columnPosition--; columnPosition > -1; columnPosition--)
            {
                if ((pointAt == 0xFF) && (pointRight == 0xFF)) // 依旧是全白，继续向左遍历
                {
                    continue;
                }
                else
                {
                    break;
                }
            }
            if (columnPosition <= -1) // 抵达右边界
            {
                isEdgeWhite = true;                        // 标记边缘白色块
                columnPosition += 6;                       // 向右偏移，重新寻找
                vertical.borderLeft[i] = BORDER_LEFT_MOST; // 存入边界
                continue;
            }
            if (accurateBorderFindLeft(i, columnPosition) == true)
            {
                continue; // 找到后换一行搜索
            }
        }
    }
}
/*------------------------------*/
/*	  基准点精准搜寻模块（右） 	 */
/*==============================*/
//? 返回枚举类型
static inline void accuratePointFindRight(const uint8_t *const p, int const position)
{
    // 黑白转变点在左边八位的情况
    vertical.datumMarkRight[0] = imageRow - 1; // 基准点所在行
    if ((*p != 0xFF) && (*p != 0x00))
    {
        vertical.datumFlag = DATUM_BOTTOM_NORMAL;
        vertical.datumMarkRight[1] = position; // 基准点所在列（压缩图像中的所在列）
        return;
    }
    // 右边八位
    if ((*(p + 1) != 0xFF) && (*(p + 1) != 0x00))
    {
        vertical.datumFlag = DATUM_BOTTOM_NORMAL;
        vertical.datumMarkRight[1] = position + 1; // 基准点所在列，在右边，所以+1
        return;
    }
    // 左白右黑
    vertical.datumFlag = DATUM_BOTTOM_IN_CENTER;
    vertical.datumMarkRight[1] = position + 1;
}
/*------------------------------*/
/*      基准点寻找模块（右）     */
/*==============================*/
//? 传入二值化图像，返回基准点粗略坐标及寻找模式
static void datumMarkRight(void)
{
    // 起始点，从图像最底下的行开始搜索边界
    for (int i = DATUM_START_RIGHT; i < binaryImageColumn - 2; i++)
    {
        if (binaryImage[imageRow - 1][i] == 0xFF && binaryImage[imageRow - 1][i + 1] == 0xFF) // 全白的情况，持续跳过
        {
            continue;
        }
        accuratePointFindRight(&binaryImage[imageRow - 1][i], i); // 基准点在当前行，调用函数判断基准点的具体位置
        return;
    }
    // 在当前行搜索不到基点，开始向上搜索
    for (int i = imageRow - 1; i > 0; i--)
    {
        if ((binaryImage[i][binaryImageColumn - 1] ^ binaryImage[i - 1][binaryImageColumn - 1]) == 0x00)
        {
            continue;
        }
        vertical.datumFlag = DATUM_ABOVE;
        vertical.datumMarkRight[0] = i;                     // 记录基准点所在行
        vertical.datumMarkRight[1] = binaryImageColumn - 1; // 存入基准点所在列（经过压缩的图像）
        return;
    }
}
//?/*------------------------------*/
//?/*          右边界寻找          */
//?/*==============================*/
static bool accurateBorderFindRight(int i, int columnPosition) // 辅助找边界点 | 找到返回true
{
    if ((pointAt != 0x00) && (pointAt != 0xFF)) // 当前8个点存在边界
    {
        //? 寻找边界准确位置
        for (int j = COMPRESS_BITS - 1; j > 0; j--)
        {
            if (((pointAt >> j) & 0x01) != (pointAt >> (j - 1) & 0x01)) // 遍历八个点，找到黑白转变点
            {
                vertical.borderRight[i] = columnPosition * COMPRESS_BITS + (COMPRESS_BITS - j); // 计算得到基准点准确位置
                break;
            }
        }
        return true;
    }
    if ((pointRight != 0x00) && (pointRight != 0xFF)) // 当前点右边8个点存在边界
    {
        //? 寻找边界准确位置
        for (int j = COMPRESS_BITS - 1; j > 0; j--)
        {
            if (((pointRight >> j) & 0x01) != (pointRight >> (j - 1) & 0x01)) // 遍历八个点，找到黑白转变点
            {
                vertical.borderRight[i] = (columnPosition + 1) * COMPRESS_BITS + (COMPRESS_BITS - j); // 计算得到基准点准确位置
                break;
            }
        }
        return true;
    }
    if ((pointAt == 0xFF) && (pointRight == 0x00)) // 左边全白，右边全黑
    {
        vertical.borderRight[i] = ((columnPosition + 1) * COMPRESS_BITS);
        return true;
    }
    if ((pointAt == 0x00) && (pointRight == 0xFF)) //? 左边全黑，右边全白 | 不太可能出现的情况
    {
        vertical.borderRight[i] = (columnPosition * COMPRESS_BITS + COMPRESS_BITS - 1);
        return true;
    }
    return false;
}
//? 传入二值化图像，传入边界数组源地址
static void searchRightBorder(void)
{
    // 定位基准点
    datumMarkRight();                                // 调用函数，寻找基准点的粗略位置
    int rowPosition = vertical.datumMarkRight[0];    // 基准点所在行
    int columnPosition = vertical.datumMarkRight[1]; // 基准点所在列（经过压缩的图像）
    // 数值初始化
    vertical.cutRowRight = vertical.datumMarkRight[0] - 1;
    horizonRight.exist = false;        // 水平边界初始化
    horizonRight.bottomRow = imageRow; //! horizontal_t 作为参数传入会比较合适
    horizonRight.topRow = imageRow;
    // 分情况找到基准点具体位置
    switch (vertical.datumFlag)
    {
    case DATUM_BOTTOM_NORMAL: // 一般情况，边界为八个点的某一个点
        for (int i = COMPRESS_BITS - 1; i > 0; i--)
        {
            if (((pointAt >> i) & 0x01) != (pointAt >> (i - 1) & 0x01)) // 遍历八个点，找到黑白转变点
            {
                vertical.borderRight[rowPosition] = columnPosition * COMPRESS_BITS + (COMPRESS_BITS - i); // 计算得到基准点准确位置
                break;
            }
        }
        break;
    case DATUM_BOTTOM_IN_CENTER: // 基准点在16个点中间的情况
        vertical.borderRight[rowPosition] = columnPosition * COMPRESS_BITS;
        break;
    case DATUM_ABOVE:
        for (int i = COMPRESS_BITS - 1; i > 0; i--)
        {
            if (((pointAt >> i) & 0x01) != (pointAt >> (i - 1) & 0x01)) // 遍历八个点，找到黑白转变点
            {
                vertical.borderRight[rowPosition] = columnPosition * COMPRESS_BITS + (COMPRESS_BITS - i); // 计算得到基准点准确位置
                break;
            }
        }
        for (int i = imageRow - 1; i > rowPosition; i--)
        {
            vertical.borderRight[i] = BORDER_RIGHT_MOST;
        }
        break;
    }
    // 在最右边的时候留一个空位，防止后续处理越界
    if (columnPosition >= binaryImageColumn - 1)
    {
        columnPosition--;
    }
    // 水平边界搜索标志位
    bool isEdgeWhite = false;
    bool isEdgeWhiteTemp = false;
    // 向上搜索边界 | 这里直接拿基准点坐标当作搜索点坐标了
    for (int i = rowPosition - 1; i > 0; i--) // 从基准点上方一行开始搜索
    {
        vertical.cutRowRight = i; // 保存搜索停止行
        //! 判断是否存在边界跳变
        if ((isEdgeWhite != isEdgeWhiteTemp) && (i < imageRow - HORIZONTAL_BORDER_SKIP_ROW)) // 存在黑白跳变且行高高于底部10行
        {
            if (horizonRight.exist == false) // 尚未找到边界
            {
                if (isEdgeWhiteTemp == false && isEdgeWhite == true) // 前一行是黑，后一行是白
                {
                    horizonRight.bottomRow = i + 1; // 这里提前换行了，找到的跳变在上一行
                }
                else // 既然有跳变，那不是前黑后白，就是前白后黑了
                {
                    if (horizonRight.bottomRow != imageRow) // 先判断是否有边界黑到白跳变
                    {
                        horizonRight.topRow = i + 1;                                                 // 存储跳变所在行
                        if (horizonRight.bottomRow - horizonRight.topRow > HORIZONTAL_BORDER_HEIGHT) // 判断上下行离得足够远 | 这里因为原点的原因，top坐标会比较小
                        {
                            horizonRight.exist = true; // 满足所有条件，存在水平边界
                            searchHBRight();           //! 调用边界寻找函数，临时用一下
                        }
                        else // 不满足高度要求，重置
                        {
                            horizonRight.bottomRow = imageRow;
                            horizonRight.topRow = imageRow;
                        }
                    }
                }
            }
        }
        // 重置水平搜索标志位
        isEdgeWhiteTemp = isEdgeWhite;
        isEdgeWhite = false;
        //! 寻找边界
        if (accurateBorderFindRight(i, columnPosition) == true)
        {
            continue; // 找到后换一行搜索
        }
        // 16个点全为黑的情况
        if ((pointAt == 0x00) && (pointRight == 0x00))
        {
            for (columnPosition--; columnPosition > -1; columnPosition--) // 初始左移8bit，之后持续左移，直到找到边界
            {
                if ((pointAt == 0x00) && (pointRight == 0x00)) // 依旧是全黑，继续向左遍历
                {
                    continue;
                }
                else
                {
                    break;
                }
            }
            if (columnPosition == -1) // 抵达最边界，停止搜索
            {
                vertical.cutRowRight = i;
                //? 图像边缘黑白检测预留
                if ((horizonRight.exist == false) && (isEdgeWhiteTemp == true)) // 前一行还是白色的，这一行就变黑了
                {
                    if (horizonRight.bottomRow != imageRow) // 先判断是否有边界黑到白跳变
                    {
                        horizonRight.topRow = i;                                                     // 存储跳变所在行 | 这里没换行，所以是当前行
                        if (horizonRight.bottomRow - horizonRight.topRow > HORIZONTAL_BORDER_HEIGHT) // 判断上下行离得足够远 | 这里因为原点的原因，top坐标会比较小
                        {
                            horizonRight.exist = true; // 满足所有条件，存在水平边界
                            searchHBRight();           //! 调用边界寻找函数，临时用一下
                        }
                    }
                }
                return;
            }
            //! 找边界具体位置 | 好像有更简单的写法
            if (accurateBorderFindRight(i, columnPosition) == true)
            {
                continue;
            }
        }
        // 16个点全为白的情况
        if ((pointAt == 0xFF) && (pointRight == 0xFF))
        {
            for (columnPosition++; columnPosition < binaryImageColumn - 1; columnPosition++)
            {
                if ((pointAt == 0xFF) && (pointRight == 0xFF)) // 依旧是全白，继续向右遍历
                {
                    continue;
                }
                else
                {
                    break;
                }
            }
            if (columnPosition >= binaryImageColumn - 1) // 抵达右边界
            {
                isEdgeWhite = true;                          // 标记边缘白色块
                columnPosition -= 6;                         // 向左偏移，重新寻找
                vertical.borderRight[i] = BORDER_RIGHT_MOST; // 存入边界
                continue;
            }
            if (accurateBorderFindRight(i, columnPosition) == true)
            {
                continue; // 找到后换一行搜索
            }
        }
    }
}
//!/*------------------------------*/
//!/*		   图像显示模块 		*/
//!/*==============================*/
//? 从模块化的角度讲应该传入二值化图像地址及图像大小
static void imageDisplayIPS114(void)
{
    bool isWhite;
    //?	二值化图像显示
    ips114_set_region(0, 0, BORDER_RIGHT_MOST, imageRow - 1); // 设置要填充的范围
    for (int i = 0; i < imageRow; i++)
    {
        for (int j = 0; j < binaryImageColumn; j++)
        {
            for (int k = COMPRESS_BITS; k > 0; k--) // 遍历每一个像素点
            {
                isWhite = ((binaryImage[i][j] >> (k - 1) & 0x01) == 0x01) ? true : false; // 判断当前坐标像素的颜色
                if (isWhite == true)
                {
                    ips114_writedata_16bit(WHITE_COLOR); // 填充白色
                }
                else
                {
                    ips114_writedata_16bit(displayColor); // 填充自定义颜色
                }
            }
        }
    }
    //? 显示左右边界
    for (int i = imageRow - 1; i > vertical.cutRowRight; i--)
    {
        ips114_drawpoint(vertical.borderRight[i], i, BLACK_COLOR);
    }
    for (int i = imageRow - 1; i > vertical.cutRowLeft; i--)
    {
        ips114_drawpoint(vertical.borderLeft[i], i, BLACK_COLOR);
    }
    //? 水平边界显示
    if (horizonRight.exist == true) //? 是否检测到水平边界
    {
        // ips114_showstr(0, 6, "true  ");
        for (int i = BORDER_RIGHT_MOST; i > horizonRight.lowerCut; i--)
        {
            ips114_drawpoint(i, horizonRight.lower[i], PINK_001);
            // ips114_drawpoint(i, horizonRight.lower[i] - 1, PINK_001);
        }
        for (int i = BORDER_RIGHT_MOST; i > horizonRight.upperCut; i--)
        {
            ips114_drawpoint(i, horizonRight.upper[i], PINK_001);
            // ips114_drawpoint(i, horizonRight.upper[i] + 1, PINK_001);
        }
    }
    // else
    // {
    //     ips114_showstr(0, 6, "false");
    // }
    if (horizonLeft.exist == true) //? 是否检测到水平边界
    {
        // ips114_showstr(0, 7, "true  ");
        for (int i = BORDER_LEFT_MOST; i < horizonLeft.lowerCut; i++)
        {
            ips114_drawpoint(i, horizonLeft.lower[i], ORANGE_001);
            // ips114_drawpoint(i, horizonLeft.lower[i] - 1, ORANGE_001);
        }
        for (int i = BORDER_LEFT_MOST; i < horizonLeft.upperCut; i++)
        {
            ips114_drawpoint(i, horizonLeft.upper[i], ORANGE_001);
            // ips114_drawpoint(i, horizonLeft.upper[i] + 1, ORANGE_001);
        }
    }
    //     else
    //     {
    //         ips114_showstr(0, 7, "false");
    //     }
}

static void imageDisplayIPS200(void) // 2.0寸液晶屏显示
{
    bool isWhite;
    //?	二值化图像显示
    ips200_address_set(0, 0, BORDER_RIGHT_MOST, imageRow - 1); // 设置要填充的范围
    for (int i = 0; i < imageRow; i++)
    {
        for (int j = 0; j < binaryImageColumn; j++)
        {
            for (int k = COMPRESS_BITS; k > 0; k--) // 遍历每一个像素点
            {
                isWhite = ((binaryImage[i][j] >> (k - 1) & 0x01) == 0x01) ? true : false; // 判断当前坐标像素的颜色
                if (isWhite == true)
                {
                    ips200_wr_data16(WHITE_COLOR); // 填充白色
                }
                else
                {
                    ips200_wr_data16(displayColor); // 填充自定义颜色
                }
            }
        }
    }
    //? 显示左右边界
    for (int i = imageRow - 1; i > vertical.cutRowRight; i--)
    {
        ips200_drawpoint(vertical.borderRight[i], i, BLACK_COLOR);
    }
    for (int i = imageRow - 1; i > vertical.cutRowLeft; i--)
    {
        ips200_drawpoint(vertical.borderLeft[i], i, BLACK_COLOR);
    }
}
/*------------------------------*/
/*          二值化模块          */
/*==============================*/
//? 从模块化的角度应该传入源图像地址、二值化图像地址和二值化阈值
static void imageBinary(void)
{
    // 图像二值化
    for (int i = 0; i < imageRow; i++)
    {
        for (int j = 0; j < binaryImageColumn; j++)
        {
            // 重置图像
            binaryImage[i][j] = 0x00;
            // 二值化
            for (int k = 0; k < COMPRESS_BITS; k++)
            {
                binaryImage[i][j] <<= 1;
                if (camImage[i][(j * COMPRESS_BITS) + k + CENTER_IMAGE_BIAS] > cam.threshold)
                {
                    binaryImage[i][j] |= 0x01; // 大于阈值的部分置为1，表示白色
                }
            }
        }
    }
}
/*------------------------------*/
/*          大津法模块          */
/*==============================*/
static void otsu(void)
//? 从模块化的角度应该传入源图像地址及图像大小，返回阈值
{
    // 统计直方图
    uint16_t hist[grayscale] = {0};
    for (int i = 0; i < imageRow; i++)
    {
        for (int j = 0; j < imageColumn; j++)
        {
            hist[camImage[i][j]]++;
        }
    }
    otsu_t sumPK = 0, sumMK = 0;
    // 求类间方差
    for (int i = 0; i < grayscale; i++)
    {
        cam.P[i] = (float)hist[i] / cam.imageSize; // 计算每个灰度级出现的概率
        cam.PK[i] = sumPK + cam.P[i];              // 概率累计和
        sumPK = cam.PK[i];
        cam.MK[i] = sumMK + i * cam.P[i]; // 灰度值累加均值
        sumMK = cam.MK[i];
    }
    otsu_t var = 0, varTemp = 0;
    for (int i = OTSU_SCALE_LEFT; i <= OTSU_SCALE_RIGHT; i++)
    {
        otsu_t temp = (cam.MK[grayscale - 1] * cam.PK[i] - cam.MK[i]);
        varTemp = (temp * temp) / (cam.PK[i] * (1 - cam.PK[i]));
        if (varTemp > var)
        {
            var = varTemp;
            cam.threshold = i;
        }
    }
}

void cameraInit(void)
{
    //? 摄像头相关初始化
    mt9v03x_init();                         // 初始化摄像头
    memset(&cam, 0, sizeof(image_t));       // 结构体数值重置
    cam.imageSize = imageRow * imageColumn; // 计算图像尺寸
    //? 循迹相关初始化
    tracer.otsu = otsu;                           // 大津法
    tracer.binary = imageBinary;                  // 图像二值化
    tracer.display = imageDisplayIPS114;          // 图像显示
    tracer.searchRightBorder = searchRightBorder; // 右边界寻找
    tracer.searchLeftBorder = searchLeftBorder;   // 左边界寻找
    tracer.stateDetection = stateDetection;       // 状态检测
    tracer.getCenterError = getCenterError;       // 返回边界中点
}

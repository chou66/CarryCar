#include "servo.h"

#include "app_config.h"
#include "competition_config.h"
#include "turnable2.h"

extern TIM_HandleTypeDef htim4;

static uint16_t Servo_ClampUs(uint16_t us)
{
    if (us < SERVO_POSITION_MIN_US)
    {
        return SERVO_POSITION_MIN_US;
    }

    if (us > SERVO_POSITION_MAX_US)
    {
        return SERVO_POSITION_MAX_US;
    }

    return us;
}
/*
 * 270度位置舵机角度转PWM脉宽。
 *
 * 当前设定：
 * 0°   -> 500 us
 * 135° -> 1500 us
 * 270° -> 2500 us
 */
static uint16_t Servo_Angle270ToUs(uint16_t angle,
                                  uint8_t inverse)
{
    uint32_t span =
        (uint32_t)SERVO_POSITION_MAX_US -
        (uint32_t)SERVO_POSITION_MIN_US;

    uint32_t offset;

    if (angle > 270U)
    {
        angle = 270U;
    }

    offset =
        ((uint32_t)angle * span) / 270U;

    if (inverse != 0U)
    {
        return (uint16_t)(
            (uint32_t)SERVO_POSITION_MAX_US -
            offset
        );
    }

    return (uint16_t)(
        (uint32_t)SERVO_POSITION_MIN_US +
        offset
    );
}

static uint16_t Servo_Angle360ToUs(uint16_t angle,
                                   uint8_t inverse)
{
    uint32_t span =
        (uint32_t)SERVO_POSITION_MAX_US -
        (uint32_t)SERVO_POSITION_MIN_US;
    uint32_t offset;

    if (angle > 360U)
    {
        angle = 360U;
    }

    offset =
        ((uint32_t)angle * span) / 360U;

    if (inverse != 0U)
    {
        return (uint16_t)(
            (uint32_t)SERVO_POSITION_MAX_US -
            offset);
    }

    return (uint16_t)(
        (uint32_t)SERVO_POSITION_MIN_US +
        offset);
}

void DiscServo_SetUs(uint16_t us)
{
    __HAL_TIM_SET_COMPARE(
        &htim4,
        TIM_CHANNEL_3,
        Servo_ClampUs(us));
}

void CarouselServo_SetUs(uint16_t us)
{
    __HAL_TIM_SET_COMPARE(
        &htim4,
        TIM_CHANNEL_4,
        Servo_ClampUs(us));
}



/**
  * @brief	初始化控制圆盘绕轴旋转的舵机，即显式地开启TIM4，CH3的PWM
  * @param	无
  * @retval	无
  */
void Servo_Pivot_180_Init()
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}


/**
  * @brief  控制圆盘整体绕轴旋转，改变圆盘前后位置，占用TIM4，CH3。(PD14)
  * @param  angle: 目标角度 (0 ~ 180)
  * @retval 无
  */
void Servo_Pivot_180_SetAngle(uint8_t angle)
{
    // 1. 角度限幅保护，防止超出物理极限损坏舵机
    if (angle > 180) {
        angle = 180;
    }
   
    // 2. 将角度转换为对应的脉冲宽度 (Pulse / CCR值)
    // 0度 -> 500us
    // 180度 -> 2500us
    // 换算公式: CCR = 500 + (角度 / 180) * (2500 - 500)
    uint16_t pulse_width = 500 + (uint16_t)((angle / 180.0) * 2000);
    
    // 3. 修改 TIM4 Channel 3 的 CCR 寄存器，改变占空比
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse_width);
}


/**
  * @brief	初始化控制圆盘内部槽位旋转的舵机，即显式地开启TIM4，CH4的PWM
  * @param	无
  * @retval	无
  */
void Servo_Spin_180_Init()
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}


/**
  * @brief  控制圆盘内部槽位自转，占用TIM4，CH4。(PD15)
  * @param  angle: 目标角度 (0 ~ 180)
  * @retval 无
  */
void Servo_Spin_180_SetAngle(uint8_t angle)
{
    // 1. 角度限幅保护，防止超出物理极限损坏舵机
    if (angle > 180) {
        angle = 180;
    }
    
    // 2. 将角度转换为对应的脉冲宽度 (Pulse / CCR值)
    // 0度 -> 500us
    // 180度 -> 2500us
    // 换算公式: CCR = 500 + (角度 / 180) * (2500 - 500)
    uint16_t pulse_width = 500 + (uint16_t)((angle / 180.0) * 2000);
    
    // 3. 修改 TIM4 Channel 4 的 CCR 寄存器，改变占空比
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulse_width);
}

/**
  * @brief  初始化控制圆盘绕轴旋转的270度舵机，即显式地开启TIM4，CH3的PWM[cite: 1]
  * @param  无
  * @retval 无
  */
void Servo_Pivot_270_Init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

/**
  * @brief  控制圆盘整体绕轴旋转，改变圆盘前后位置，占用TIM4，CH3。(PD14)[cite: 1]
  * @param  angle: 目标角度 (0 ~ 270)
  * @retval 无
  */
void Servo_Pivot_270_SetAngle(uint16_t angle)
{
    // 1. 角度限幅保护，防止超出物理极限损坏舵机
    if (angle > 270) {
        angle = 270;
    }
   
    // 2. 将角度转换为对应的脉冲宽度 (Pulse / CCR值)
    // 0度 -> 500us
    // 270度 -> 2500us
    // 换算公式: CCR = 500 + (angle / 270) * 2000
    // 化简提取公因数以防浮点运算: CCR = 500 + (angle * 2000) / 270 = 500 + (angle * 200) / 27
    uint16_t pulse_width = 500 + ((angle * 200) / 27);
    
    // 3. 修改 TIM4 Channel 3 的 CCR 寄存器，改变占空比[cite: 1]
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse_width);
}


/**
  * @brief  初始化控制圆盘内部槽位旋转的270度舵机，即显式地开启TIM4，CH4的PWM[cite: 1]
  * @param  无
  * @retval 无
  */
void Servo_Spin_270_Init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}


/**
  * @brief  控制圆盘内部槽位自转，占用TIM4，CH4。(PD15)[cite: 1]
  * @param  angle: 目标角度 (0 ~ 270)
  * @retval 无
  */
void Servo_Spin_270_SetAngle(uint16_t angle)
{
    // 1. 角度限幅保护，防止超出物理极限损坏舵机
    if (angle > 270) {
        angle = 270;
    }
    
    // 2. 将角度转换为对应的脉冲宽度 (Pulse / CCR值)
    // 公式同上，CCR = 500 + (angle * 200) / 27
    uint16_t pulse_width = 500 + ((angle * 200) / 27);
    
    // 3. 修改 TIM4 Channel 4 的 CCR 寄存器，改变占空比[cite: 1]
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulse_width);
}

/**
  * @brief  初始化控制圆盘绕轴旋转的舵机，即显式地开启TIM4，CH3的PWM
  * @param
  * @retval
  */
void Servo_Pivot_360_Init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

/**
  * @brief  使用360度位置控制舵机，控制圆盘整体绕轴旋转到指定角度，默认旋转方向，即正传方向是顺时针
  * @param  angle: 目标角度 (0 ~ 360)
  *         0: 对应绝对角度0度 (500us)
  *         360: 对应绝对角度360度 (2500us)
  * @retval 无
  */
void Servo_Pivot_360_SetAngle(uint16_t angle)
{
    // 1. 角度限幅保护，防止输入超出物理极限
    if (angle > 360) {
        angle = 360;
    }
    
    // 2. 将角度转换为对应的脉冲宽度 (Pulse / CCR值)
    // 0度对应500us，360度对应2500us，跨度为2000us
    // 公式: CCR = 500 + (angle / 360) * 2000 
    // 化简提取公因数以防浮点运算: CCR = 500 + (angle * 2000) / 360 = 500 + (angle * 50) / 9
    uint16_t pulse_width = 500 + ((angle * 50) / 9);
    
    // 3. 修改 TIM4 Channel 3 的 CCR 寄存器
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse_width);
}


/**
  * @brief  初始化控制圆盘槽位旋转的舵机，即显式地开启TIM4，CH4的PWM
  * @param
  * @retval
  */
void Servo_Spin_360_Init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}


/**
  * @brief  使用360度位置控制舵机，控制槽位旋转到指定角度,默认旋转方向，即正转方向是顺时针
  * @param  angle: 目标角度 (0 ~ 360)
  *         0: 对应绝对角度0度 (500us)
  *         360: 对应绝对角度360度 (2500us)
  * @retval 无
  */
void Servo_Spin_360_SetAngle(uint16_t angle)
{
    // 1. 角度限幅保护
    if (angle > 360) {
        angle = 360;
    }
    
    // 2. 将角度转换为对应的脉冲宽度 (Pulse / CCR值)
    // 公式同上，CCR = 500 + (angle * 50) / 9
    uint16_t pulse_width = 500 + ((angle * 50) / 9);
    
    // 3. 修改 TIM4 Channel 4 的 CCR 寄存器
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulse_width);
}

/**
  * @brief  使用360度位置控制舵机，控制圆盘整体绕轴旋转到指定角度 (逆时针/反转方向)
  * @param  angle: 目标角度 (0 ~ 360)
  *         0: 对应绝对角度0度 (2500us)
  *         360: 对应绝对角度360度 (500us)
  * @retval 无
  */
void Servo_Pivot_360_SetAngle_Inverse(uint16_t angle)
{
    // 1. 角度限幅保护，防止输入超出物理极限[cite: 1]
    if (angle > 360) {
        angle = 360;
    }
    
    // 2. 将角度转换为对应的脉冲宽度 (Pulse / CCR值)
    // 逆时针映射: 0度对应2500us，360度对应500us，跨度为2000us
    // 公式化简: CCR = 2500 - (angle * 2000) / 360 = 2500 - (angle * 50) / 9
    uint16_t pulse_width = 2500 - ((angle * 50) / 9);
    
    // 3. 修改 TIM4 Channel 3 的 CCR 寄存器[cite: 1]
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse_width);
}


/**
  * @brief  使用360度位置控制舵机，控制槽位旋转到指定角度 (逆时针/反转方向)
  * @param  angle: 目标角度 (0 ~ 360)
  *         0: 对应绝对角度0度 (2500us)
  *         360: 对应绝对角度360度 (500us)
  * @retval 无
  */
void Servo_Spin_360_SetAngle_Inverse(uint16_t angle)
{
    // 1. 角度限幅保护[cite: 1]
    if (angle > 360) {
        angle = 360;
    }
    
    // 2. 将角度转换为对应的脉冲宽度 (Pulse / CCR值)
    // 逆时针映射同上，CCR = 2500 - (angle * 50) / 9
    uint16_t pulse_width = 2500 - ((angle * 50) / 9);
    
    // 3. 修改 TIM4 Channel 4 的 CCR 寄存器[cite: 1]
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulse_width);
}
void DiscServo_ToRear(void)
{
    Servo_Pivot_270_SetAngle(DISC_SERVO_TOP_DEG);
}

void DiscServo_ToFront(void)
{
    Servo_Pivot_270_SetAngle(DISC_SERVO_FRONT_DEG);
}

void DiscServo_ToLinePickup(void)
{
    Servo_Pivot_270_SetAngle(DISC_SERVO_LINE_PICK_DEG);
}

/*
 * The corrected Turnable2 angle table is the single source of truth
 * for all internal carousel positions. Legacy callers remain compatible.
 */
void CarouselServo_GotoIntakeSlot(uint8_t slot)
{
    if (slot < TURNABLE_TASK1_SLOT_COUNT)
    {
        Servo_Spin_360_SetAngle(
            task1_configs[slot].angle_opening);
    }
}

void CarouselServo_GotoReleaseSlot(uint8_t slot)
{
    CarouselServo_GotoIntakeSlot(slot);
}

void CarouselServo_GotoColorSlot(uint8_t slot)
{
    if (slot < TURNABLE_TASK1_SLOT_COUNT)
    {
        Servo_Spin_360_SetAngle(
            task1_configs[slot].angle_to_gy33);
    }
}

void CarouselServo_GotoSlot(uint8_t slot)
{
    CarouselServo_GotoIntakeSlot(slot);
}

void Servo_Init(void)
{
    Servo_Pivot_270_Init();   /* PD14：270°位置舵机 */
    Servo_Spin_360_Init();    /* PD15：内部转盘，保持原来360°逻辑 */

    DiscServo_ToRear();
    CarouselServo_GotoIntakeSlot(0U);
}


void Servo_SetUs(uint16_t us)
{
    DiscServo_SetUs(us);
}

void Servo_Open(void)
{
    DiscServo_ToFront();
    HAL_Delay(300U);
}

void Servo_Close(void)
{
    DiscServo_ToRear();
    HAL_Delay(300U);
}

void Servo_Mid(void)
{
    DiscServo_SetUs(SERVO_MID_US);
    HAL_Delay(300U);
}

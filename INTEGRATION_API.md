# 常用 API

## 路线
```c
RobotApp_GotoNode(5);
RobotApp_GotoReturnNode();
RobotApp_EStop();
```

## 抓夹 PB4
```c
GripperServo_Open();
GripperServo_Close();
GripperServo_Mid();
```

## 三工位圆盘 PE3
```c
CarouselServo_GotoSlot(0);
CarouselServo_GotoSlot(1);
CarouselServo_GotoSlot(2);
```

## 电机5 云台
```c
Manipulator_Gimbal_MovePulses(dir, rpm, acc, pulses);
```

## 电机6 前后滑台
```c
Manipulator_Slide_MovePulses(dir, rpm, acc, pulses);
```

## HWT101
```c
float yaw = HWT101_GetYaw();
uint8_t online = HWT101_IsOnline();
HWT101_ZeroYaw();
```

## K230
```c
K230_Result_t r;
if (K230_GetLatestResult(&r)) { ... }
```

## 扫码
```c
char code[64];
if (QRScanner_GetNewCode(code, sizeof(code))) { ... }
```

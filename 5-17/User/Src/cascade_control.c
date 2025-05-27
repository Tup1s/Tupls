// #include <stdio.h>
 
// // 定义PID结构体用于存放一个PID的数据
// typedef struct
// {
//     float kp, ki, kd;        // 三个系数：比例、积分和微分
//     float error, lastError;  // 当前误差、上次误差
//     float integral, maxIntegral;  // 积分、积分限幅
//     float output, maxOutput; // 输出、输出限幅
//     int32_t setposition;  // 设定位置
// } PID;
 
// // 用于初始化PID参数的函数
// void PID_Init_control(PID *pid, float p, float i, float d, float maxI, float maxOut)
// {
//     pid->kp = p;  // 设置比例系数
//     pid->ki = i;  // 设置积分系数
//     pid->kd = d;  // 设置微分系数
//     pid->maxIntegral = maxI;  // 设置积分限幅
//     pid->maxOutput = maxOut;  // 设置输出限幅
//     pid->error = 0;
//     pid->lastError = 0;
//     pid->integral = 0;
//     pid->output = 0;
// }
 
// // 参数为(pid结构体, 目标值, 反馈值)，计算结果放在pid结构体的output成员中
// void PID_Calc(PID *pid, float reference, float feedback)
// {
//     // 更新数据
//     pid->lastError = pid->error;  // 将旧error存起来
//     pid->error = reference - feedback;  // 计算新error
 
//     // 计算微分项
//     float dout = (pid->error - pid->lastError) * pid->kd;
 
//     // 计算比例项
//     float pout = pid->error * pid->kp;
 
//     // 计算积分项
//     pid->integral += pid->error * pid->ki;
 
//     // 积分限幅
//     if (pid->integral > pid->maxIntegral) 
//         pid->integral = pid->maxIntegral;
//     else if (pid->integral < -pid->maxIntegral) 
//         pid->integral = -pid->maxIntegral;
 
//     // 计算输出
//     pid->output = pout + dout + pid->integral;
 
//     // 输出限幅
//     if (pid->output > pid->maxOutput) 
//         pid->output = pid->maxOutput;
//     else if (pid->output < -pid->maxOutput) 
//         pid->output = -pid->maxOutput;
// }
// void PID_SetPosition(PID_Controller *pid, int32_t setposition)  
// {  
//     pid->setposition = setposition;  
// }

// // 串级PID的结构体，包含两个单级PID
// typedef struct
// {
//     PID inner; // 内环
//     PID outer; // 外环
//     float output; // 串级输出，等于inner.output
// } CascadePID;
 
// // 串级PID的计算函数
// // 参数(PID结构体, 外环目标值, 外环反馈值, 内环反馈值)
// void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb)
// {
//     PID_Calc(&pid->outer, outerRef, outerFdb); // 计算外环
//     PID_Calc(&pid->inner, pid->outer.output, innerFdb); // 计算内环
//     pid->output = pid->inner.output; // 内环输出就是串级PID的输出
// }
 
// // 模拟设定执行器输出大小的函数
// void setActuatorOutput(float output)
// {
//     // 这里是将PID的输出值应用到执行器上的代码
//     // 在实际应用中，这可能是一个控制电机速度、阀门开度等的函数
//     printf("Actuator Output: %f\n", output);
// }
 
// // 模拟获取反馈值的函数
// float getFeedbackValue()
// {
//     // 这里是获取系统当前反馈值的代码
//     // 在实际应用中，这可能是从传感器读取的值
//     // 这里暂时返回一个模拟值
//     static float feedback = 0;
//     feedback += 1;  // 模拟反馈值增加
//     return feedback;
// }
 
// // 模拟获取目标值的函数
// float getTargetValue()
// {
//     // 这里是获取系统目标值的代码
//     // 在实际应用中，这可能是从用户输入或者其他系统计算得到的
//     // 这里暂时返回一个固定目标值
//     return 100;
// }
 
// CascadePID mypid = {0}; // 创建串级PID结构体变量
 
// int main()
// {
//     // ...其他初始化代码
//     // 初始化内环参数：比例系数10，积分系数0，微分系数0，最大积分0，最大输出1000
//     PID_Init(&mypid.inner, 10, 0, 0, 0, 1000);
//     // 初始化外环参数：比例系数5，积分系数0，微分系数5，最大积分0，最大输出100
//     PID_Init(&mypid.outer, 5, 0, 5, 0, 100);
 
//     while (1) // 进入循环运行
//     {
//         float outerTarget = getTargetValue(); // 获取外环目标值
//         float outerFeedback = getFeedbackValue(); // 获取外环反馈值
//         float innerFeedback = getFeedbackValue(); // 获取内环反馈值
//         PID_CascadeCalc(&mypid, outerTarget, outerFeedback, innerFeedback); // 进行PID计算
//         setActuatorOutput(mypid.output); // 设定执行器输出大小
//         // 模拟延时，这里使用sleep函数，单位为秒
//         // 在实际应用中，这个值根据系统需求调整
//         sleep(1); // 等待1秒再开始下一次循环
//     }
 
//     return 0;
// }
 
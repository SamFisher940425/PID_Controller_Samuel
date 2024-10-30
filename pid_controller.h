#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "PID_Controller_global.h"

class PID_CONTROLLER_EXPORT PID_Controller
{
public:
    PID_Controller(double KP,double KI,double KD,double KFF,double KFFB,double KFF_A,double KFFB_B,double Integral_Max,double Integral_Min,double Output_Max,double Output_Min);
    PID_Controller(double KP,double KI,double KD,double Integral_Max,double Integral_Min,double Output_Max,double Output_Min); ///<不采用改进算法，传统PID构造函数
    void Clear();

    double Incremental_PID(double Input,double Feedback); ///<增量式PID算法，结合梯形积分，积分抗饱和，微分先行，不完全前馈，不完全前馈增压等优化算法，当Kff=Kd且Kffb、Kff_Alpha、Kffb_Beta=0时，等效为传统PID
    double Incremental_PID(double Input,double Feedback,double Differential_Error); ///<微分项误差(反馈量一阶导)可被传感器测量值时，微分项误差可用传感器数值代替，例如角度环的微分项误差为角速度，可用陀螺仪测得,使用时注意传感器反馈信号的极性

    double Position_PID(double Input,double Feedback); ///<位置式PID，结合积分抗饱和，微分先行，不完全前馈，不完全前馈增压等优化算法，当Kff=Kd且Kffb、Kff_Alpha、Kffb_Beta=0时，等效为传统PID
    double Position_PID(double Input,double Feedback,double Differential_Error); ///<与增量式注释同理

    double Set_Output(double Output);///<设定输出值，一般用于手动自动平滑切换

    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kff = 0; ///<前馈系数
    double Kffb = 0; ///<前馈增压系数
    double Kff_Alpha = 0; ///<不完全前馈系数，范围0~1，越大延缓作用越明显，为0时，等效不使用不完全前馈
    double Kffb_Beta = 0; ///<不完全前馈增压系数，范围0~1，越大延缓作用越明显，为0时，等效不使用不完全前馈

    double Integ_Max = 0; ///<积分上限，一般取本级PID执行器正向最大输出
    double Integ_Min = 0; ///<积分下限，一般取本级PID执行器负向最大输出

    double Out_Max = 0;///<输出限幅，一般取本级PID执行器正向最大输出
    double Out_Min = 0;///<输出限幅，一般取本级PID执行器负向最大输出

private:

    double Err[3] = {0}; ///<误差存储器，0脚标位置储存本次输入减反馈，1脚标为上次误差，2脚标为上上次误差
    double In[3] = {0}; ///<输入存储器，0脚标位置储存本次输入，1脚标为上次输入，2脚标为上上次输入
    double Fb[3] = {0}; ///<反馈存储器，0脚标位置储存本次反馈，1脚标为上次反馈，2脚标为上上次反馈
    double Position_Integ_Term[3] = {0}; ///<位置式PID积分项储存器，0脚标位置储存输入项累加值，1脚标为本次积分项，2脚标为上次积分项
    double Diff_Term[3] = {0}; ///<微分项储存器，0脚标位置储存本次微分项，1脚标位置储存上次微分项，2脚标位置储存上上次微分项
    double Ff_Term[3] = {0}; ///<前馈项储存器，0脚标位置储存本次前馈项，1脚标位置储存上次前馈项，2脚标位置储存上上次微前馈项
    double Ffb_Term[3] = {0}; ///<前馈增压项储存器，0脚标位置储存本次前馈增压项，1脚标位置储存上次前馈增压项，2脚标位置储存上上次前馈增压项

    double Out = 0;
    double Delta_Out = 0;

};

#endif // PID_CONTROLLER_H

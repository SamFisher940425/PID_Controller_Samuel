#include "pid_controller.h"

/**
 * @brief PID控制器构造函数
 * @details 改进型PID控制器构造函数
 * @param KP:比例系数
 * @param KI:积分系数
 * @param KD:微分系数
 * @param KFF:前馈系数，与微分系数相等时，等效为传统的无微分先行和前馈PID
 * @param KFFB:前馈增压系数，为0时，等效不使用前馈增压
 * @param KFF_A:不完全前馈系数，范围0~1，越大延缓作用越明显，为0时，等效不使用不完全前馈
 * @param KFFB_B:不完全前馈增压系数，范围0~1，越大延缓作用越明显，为0时，等效不使用不完全前馈
 * @param Integral_Max:积分项上限，一般取本级PID执行器正向最大输出
 * @param Integral_Min:积分项下限，一般取本级PID执行器负向最大输出
 * @param Output_Max:输出上限，本级PID执行器正向最大输出
 * @param Output_Min:输出下限，本级PID执行器负向最大输出
 *
 * @note
 *  关于微分先行与前馈改进，由于传统PID在输入量突变和反馈量突变时，都会引起微分项突变
 *  但输入量突变引起的微分项变化是帮助系统执行动作（前馈效应），反馈量突变引起的微分项变化是抑制系统动作（阻尼效应）
 *  因此将传统微分项拆开为前馈项和微分项，并分别赋予两个不同的系数，实现互相解耦的调整系数，前馈负责系统的快速跟随，微分负责系统稳定阻尼
 * @note
 *  参考资料：
 *  https://blog.csdn.net/foxclever/article/details/80633275
 *  https://blog.csdn.net/foxclever/article/details/81048086
 * @note
 *  关于不完全前馈，由于对阶跃类输入，前馈项仅在输入跳变的瞬间有输出，其他计算过程均无数输出
 *  对于有惯性（不能突变）的系统而言，这种转瞬即逝的输出既无效果，又有害处（损伤元件）
 *  因此在前馈项后串接一阶惯性环节（一阶滤波器）延缓前馈作用，使其产生效果，平滑前馈项输出
 * @note
 *  参考资料：
 *  https://blog.csdn.net/foxclever/article/details/80551919
 * @note
 *  关于前馈增压，是对于前馈项的一个补充，给控制器输出增添一个与输入的二阶导数成比例关系的项
 *  以位置控制为例，前馈增压等效为与期望位置变化的二阶导成比例的项，即与加速度成比例的项，进一步的提高了系统的跟随快速性
 * @note
 *  参考资料：https://github.com/betaflight/betaflight/wiki/Feed-Forward-2.0
 * @note
 *  关于不完全前馈增压，与不完全前馈类似，为解决前馈增压项的瞬间输出问题所串接的一阶惯性环节
 */
PID_Controller::PID_Controller(double KP,double KI,double KD,double KFF,double KFFB,double KFF_A,double KFFB_B,double Integral_Max,double Integral_Min,double Output_Max,double Output_Min)
{
    PID_Controller::Kp = KP;
    PID_Controller::Kd = KD;
    PID_Controller::Ki = KI;
    PID_Controller::Kff = KFF;
    PID_Controller::Kffb = KFFB;
    PID_Controller::Kff_Alpha = KFF_A;
    PID_Controller::Kffb_Beta = KFFB_B;
    PID_Controller::Integ_Max = Integral_Max;
    PID_Controller::Integ_Min = Integral_Min;
    PID_Controller::Out_Max = Output_Max;
    PID_Controller::Out_Min = Output_Min;
}

PID_Controller::PID_Controller(double KP,double KI,double KD,double Integral_Max,double Integral_Min,double Output_Max,double Output_Min)
{
    PID_Controller::Kp = KP;
    PID_Controller::Kd = KD;
    PID_Controller::Ki = KI;
    PID_Controller::Kff = KD;
    PID_Controller::Kffb = 0;
    PID_Controller::Kff_Alpha = 0;
    PID_Controller::Kffb_Beta = 0;
    PID_Controller::Integ_Max = Integral_Max;
    PID_Controller::Integ_Min = Integral_Min;
    PID_Controller::Out_Max = Output_Max;
    PID_Controller::Out_Min = Output_Min;
}

void PID_Controller::Clear()
{
    for(int i=0;i<3;i++)
    {
        PID_Controller::Err[i] = 0;
        PID_Controller::In[i] = 0;
        PID_Controller::Fb[i] = 0;
        PID_Controller::Position_Integ_Term[i] = 0;
        PID_Controller::Diff_Term[i] = 0;
        PID_Controller::Ff_Term[i] = 0;
        PID_Controller::Ffb_Term[i] = 0;
    }
    PID_Controller::Integ_Max = 0;
    PID_Controller::Integ_Min = 0;
    PID_Controller::Out_Max = 0;
    PID_Controller::Out_Min = 0;
    PID_Controller::Out = 0;
    PID_Controller::Delta_Out = 0;
}

double PID_Controller::Position_PID(double Input,double Feedback)
{
    PID_Controller::In[2] = PID_Controller::In[1];
    PID_Controller::In[1] = PID_Controller::In[0];
    PID_Controller::In[0] = Input;

    PID_Controller::Fb[2] = PID_Controller::Fb[1];
    PID_Controller::Fb[1] = PID_Controller::Fb[0];
    PID_Controller::Fb[0] = Feedback;

    PID_Controller::Err[2] = PID_Controller::Err[1];
    PID_Controller::Err[1] = PID_Controller::Err[0];
    PID_Controller::Err[0] = PID_Controller::In[0] - PID_Controller::Fb[0];

    double Prop_Term = PID_Controller::Kp * PID_Controller::Err[0];

    PID_Controller::Position_Integ_Term[2] = PID_Controller::Position_Integ_Term[1];
    PID_Controller::Position_Integ_Term[1] = PID_Controller::Ki * (PID_Controller::Err[0]+PID_Controller::Err[1]) * 0.5;
    if(PID_Controller::Position_Integ_Term[1] > PID_Controller::Integ_Max)PID_Controller::Position_Integ_Term[1] = PID_Controller::Integ_Max;
    if(PID_Controller::Position_Integ_Term[1] < PID_Controller::Integ_Min)PID_Controller::Position_Integ_Term[1] = PID_Controller::Integ_Min;
    PID_Controller::Position_Integ_Term[0] += PID_Controller::Position_Integ_Term[1];
    if(PID_Controller::Position_Integ_Term[0] > PID_Controller::Integ_Max)PID_Controller::Position_Integ_Term[0] = PID_Controller::Integ_Max;
    if(PID_Controller::Position_Integ_Term[0] < PID_Controller::Integ_Min)PID_Controller::Position_Integ_Term[0] = PID_Controller::Integ_Min;

    PID_Controller::Diff_Term[2] = PID_Controller::Diff_Term[1];
    PID_Controller::Diff_Term[1] = PID_Controller::Diff_Term[0];
    PID_Controller::Diff_Term[0] = PID_Controller::Kd * (PID_Controller::Fb[0] - PID_Controller::Fb[1]);

    PID_Controller::Ff_Term[2] = PID_Controller::Ff_Term[1];
    PID_Controller::Ff_Term[1] = PID_Controller::Ff_Term[0];
    PID_Controller::Ff_Term[0] = (1 - PID_Controller::Kff_Alpha) * PID_Controller::Kff * (PID_Controller::In[0] - PID_Controller::In[1]) + PID_Controller::Kff_Alpha * PID_Controller::Ff_Term[1];

    PID_Controller::Ffb_Term[2] = PID_Controller::Ffb_Term[1];
    PID_Controller::Ffb_Term[1] = PID_Controller::Ffb_Term[0];
    PID_Controller::Ffb_Term[0] = (1 - PID_Controller::Kffb_Beta) * PID_Controller::Kffb * (PID_Controller::In[0] - 2.0 * PID_Controller::In[1] + PID_Controller::In[2]) + PID_Controller::Kffb_Beta * PID_Controller::Ffb_Term[1];

    PID_Controller::Out = Prop_Term + PID_Controller::Position_Integ_Term[0] - PID_Controller::Diff_Term[0] + PID_Controller::Ff_Term[0] + PID_Controller::Ffb_Term[0];
    if(PID_Controller::Out > PID_Controller::Out_Max)PID_Controller::Out = PID_Controller::Out_Max;
    if(PID_Controller::Out < PID_Controller::Out_Min)PID_Controller::Out = PID_Controller::Out_Min;

    return PID_Controller::Out;
}

double PID_Controller::Position_PID(double Input,double Feedback,double Differential_Error)
{
    PID_Controller::In[2] = PID_Controller::In[1];
    PID_Controller::In[1] = PID_Controller::In[0];
    PID_Controller::In[0] = Input;

    PID_Controller::Fb[2] = PID_Controller::Fb[1];
    PID_Controller::Fb[1] = PID_Controller::Fb[0];
    PID_Controller::Fb[0] = Feedback;

    PID_Controller::Err[2] = PID_Controller::Err[1];
    PID_Controller::Err[1] = PID_Controller::Err[0];
    PID_Controller::Err[0] = PID_Controller::In[0] - PID_Controller::Fb[0];

    double Prop_Term = PID_Controller::Kp * PID_Controller::Err[0];

    PID_Controller::Position_Integ_Term[2] = PID_Controller::Position_Integ_Term[1];
    PID_Controller::Position_Integ_Term[1] = PID_Controller::Ki * (PID_Controller::Err[0]+PID_Controller::Err[1]) * 0.5;
    if(PID_Controller::Position_Integ_Term[1] > PID_Controller::Integ_Max)PID_Controller::Position_Integ_Term[1] = PID_Controller::Integ_Max;
    if(PID_Controller::Position_Integ_Term[1] < PID_Controller::Integ_Min)PID_Controller::Position_Integ_Term[1] = PID_Controller::Integ_Min;
    PID_Controller::Position_Integ_Term[0] += PID_Controller::Position_Integ_Term[1];
    if(PID_Controller::Position_Integ_Term[0] > PID_Controller::Integ_Max)PID_Controller::Position_Integ_Term[0] = PID_Controller::Integ_Max;
    if(PID_Controller::Position_Integ_Term[0] < PID_Controller::Integ_Min)PID_Controller::Position_Integ_Term[0] = PID_Controller::Integ_Min;

    PID_Controller::Diff_Term[2] = PID_Controller::Diff_Term[1];
    PID_Controller::Diff_Term[1] = PID_Controller::Diff_Term[0];
    PID_Controller::Diff_Term[0] = PID_Controller::Kd * Differential_Error;

    PID_Controller::Ff_Term[2] = PID_Controller::Ff_Term[1];
    PID_Controller::Ff_Term[1] = PID_Controller::Ff_Term[0];
    PID_Controller::Ff_Term[0] = (1 - PID_Controller::Kff_Alpha) * PID_Controller::Kff * (PID_Controller::In[0] - PID_Controller::In[1]) + PID_Controller::Kff_Alpha * PID_Controller::Ff_Term[1];

    PID_Controller::Ffb_Term[2] = PID_Controller::Ffb_Term[1];
    PID_Controller::Ffb_Term[1] = PID_Controller::Ffb_Term[0];
    PID_Controller::Ffb_Term[0] = (1 - PID_Controller::Kffb_Beta) * PID_Controller::Kffb * (PID_Controller::In[0] - 2.0 * PID_Controller::In[1] + PID_Controller::In[2]) + PID_Controller::Kffb_Beta * PID_Controller::Ffb_Term[1];

    PID_Controller::Out = Prop_Term + PID_Controller::Position_Integ_Term[0] - PID_Controller::Diff_Term[0] + PID_Controller::Ff_Term[0] + PID_Controller::Ffb_Term[0];
    if(PID_Controller::Out > PID_Controller::Out_Max)PID_Controller::Out = PID_Controller::Out_Max;
    if(PID_Controller::Out < PID_Controller::Out_Min)PID_Controller::Out = PID_Controller::Out_Min;

    return PID_Controller::Out;
}

double PID_Controller::Incremental_PID(double Input, double Feedback)
{
    PID_Controller::In[2] = PID_Controller::In[1];
    PID_Controller::In[1] = PID_Controller::In[0];
    PID_Controller::In[0] = Input;

    PID_Controller::Fb[2] = PID_Controller::Fb[1];
    PID_Controller::Fb[1] = PID_Controller::Fb[0];
    PID_Controller::Fb[0] = Feedback;

    PID_Controller::Err[2] = PID_Controller::Err[1];
    PID_Controller::Err[1] = PID_Controller::Err[0];
    PID_Controller::Err[0] = PID_Controller::In[0] - PID_Controller::Fb[0];

    double Prop_Term = PID_Controller::Kp * (PID_Controller::Err[0] - PID_Controller::Err[1]);

    double Integ_Term = PID_Controller::Ki * (PID_Controller::Err[0]+PID_Controller::Err[1]) * 0.5;
    if(Integ_Term > PID_Controller::Integ_Max)Integ_Term = PID_Controller::Integ_Max;
    if(Integ_Term < PID_Controller::Integ_Min)Integ_Term = PID_Controller::Integ_Min;

    PID_Controller::Diff_Term[2] = PID_Controller::Diff_Term[1];
    PID_Controller::Diff_Term[1] = PID_Controller::Diff_Term[0];
    PID_Controller::Diff_Term[0] = PID_Controller::Kd * (PID_Controller::Fb[0] - PID_Controller::Fb[1]);

    PID_Controller::Ff_Term[2] = PID_Controller::Ff_Term[1];
    PID_Controller::Ff_Term[1] = PID_Controller::Ff_Term[0];
    PID_Controller::Ff_Term[0] = (1 - PID_Controller::Kff_Alpha) * PID_Controller::Kff * (PID_Controller::In[0] - PID_Controller::In[1]) + PID_Controller::Kff_Alpha * PID_Controller::Ff_Term[1];

    PID_Controller::Ffb_Term[2] = PID_Controller::Ffb_Term[1];
    PID_Controller::Ffb_Term[1] = PID_Controller::Ffb_Term[0];
    PID_Controller::Ffb_Term[0] = (1 - PID_Controller::Kffb_Beta) * PID_Controller::Kffb * (PID_Controller::In[0] - 2.0 * PID_Controller::In[1] + PID_Controller::In[2]) + PID_Controller::Kffb_Beta * PID_Controller::Ffb_Term[1];

    PID_Controller::Delta_Out = Prop_Term + Integ_Term - (PID_Controller::Diff_Term[0] - PID_Controller::Diff_Term[1]) + (PID_Controller::Ff_Term[0] - PID_Controller::Ff_Term[1]) + (PID_Controller::Ffb_Term[0] - PID_Controller::Ffb_Term[1]);
    if(PID_Controller::Delta_Out > PID_Controller::Integ_Max)PID_Controller::Delta_Out = PID_Controller::Integ_Max;
    if(PID_Controller::Delta_Out < PID_Controller::Integ_Min)PID_Controller::Delta_Out = PID_Controller::Integ_Min;

    PID_Controller::Out += PID_Controller::Delta_Out;
    if(PID_Controller::Out > PID_Controller::Out_Max)PID_Controller::Out = PID_Controller::Out_Max;
    if(PID_Controller::Out < PID_Controller::Out_Min)PID_Controller::Out = PID_Controller::Out_Min;

    return PID_Controller::Out;
}

double PID_Controller::Incremental_PID(double Input,double Feedback,double Differential_Error)
{
    PID_Controller::In[2] = PID_Controller::In[1];
    PID_Controller::In[1] = PID_Controller::In[0];
    PID_Controller::In[0] = Input;

    PID_Controller::Fb[2] = PID_Controller::Fb[1];
    PID_Controller::Fb[1] = PID_Controller::Fb[0];
    PID_Controller::Fb[0] = Feedback;

    PID_Controller::Err[2] = PID_Controller::Err[1];
    PID_Controller::Err[1] = PID_Controller::Err[0];
    PID_Controller::Err[0] = PID_Controller::In[0] - PID_Controller::Fb[0];

    double Prop_Term = PID_Controller::Kp * (PID_Controller::Err[0] - PID_Controller::Err[1]);

    double Integ_Term = PID_Controller::Ki * (PID_Controller::Err[0]+PID_Controller::Err[1]) * 0.5;
    if(Integ_Term > PID_Controller::Integ_Max)Integ_Term = PID_Controller::Integ_Max;
    if(Integ_Term < PID_Controller::Integ_Min)Integ_Term = PID_Controller::Integ_Min;

    PID_Controller::Diff_Term[2] = PID_Controller::Diff_Term[1];
    PID_Controller::Diff_Term[1] = PID_Controller::Diff_Term[0];
    PID_Controller::Diff_Term[0] = PID_Controller::Kd * Differential_Error;

    PID_Controller::Ff_Term[2] = PID_Controller::Ff_Term[1];
    PID_Controller::Ff_Term[1] = PID_Controller::Ff_Term[0];
    PID_Controller::Ff_Term[0] = (1 - PID_Controller::Kff_Alpha) * PID_Controller::Kff * (PID_Controller::In[0] - PID_Controller::In[1]) + PID_Controller::Kff_Alpha * PID_Controller::Ff_Term[1];

    PID_Controller::Ffb_Term[2] = PID_Controller::Ffb_Term[1];
    PID_Controller::Ffb_Term[1] = PID_Controller::Ffb_Term[0];
    PID_Controller::Ffb_Term[0] = (1 - PID_Controller::Kffb_Beta) * PID_Controller::Kffb * (PID_Controller::In[0] - 2.0 * PID_Controller::In[1] + PID_Controller::In[2]) + PID_Controller::Kffb_Beta * PID_Controller::Ffb_Term[1];

    PID_Controller::Delta_Out = Prop_Term + Integ_Term - (PID_Controller::Diff_Term[0] - PID_Controller::Diff_Term[1]) + (PID_Controller::Ff_Term[0] - PID_Controller::Ff_Term[1]) + (PID_Controller::Ffb_Term[0] - PID_Controller::Ffb_Term[1]);
    if(PID_Controller::Delta_Out > PID_Controller::Integ_Max)PID_Controller::Delta_Out = PID_Controller::Integ_Max;
    if(PID_Controller::Delta_Out < PID_Controller::Integ_Min)PID_Controller::Delta_Out = PID_Controller::Integ_Min;

    PID_Controller::Out += PID_Controller::Delta_Out;
    if(PID_Controller::Out > PID_Controller::Out_Max)PID_Controller::Out = PID_Controller::Out_Max;
    if(PID_Controller::Out < PID_Controller::Out_Min)PID_Controller::Out = PID_Controller::Out_Min;

    return PID_Controller::Out;
}

double PID_Controller::Set_Output(double Output)
{
    PID_Controller::Out = Output;
    if(PID_Controller::Out > PID_Controller::Out_Max)PID_Controller::Out = PID_Controller::Out_Max;
    if(PID_Controller::Out < PID_Controller::Out_Min)PID_Controller::Out = PID_Controller::Out_Min;

    return PID_Controller::Out;
}

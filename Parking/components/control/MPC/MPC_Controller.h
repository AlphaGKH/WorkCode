#pragma once
//#include<stdio.h>

#include "MPC_light_matrix.h"
#include "memorypool.h"
#include "osqp.h"
 
 
/*******************************************************************************
* 结构体定义 : VehiclePosition 车辆状态信息
  结构体内容： x,y,theta 车辆坐标系下的位置信息
                       v 当前车速
					   kappa 弯道曲率半径
					   t 参考时间
*******************************************************************************/
typedef struct {
	float x;
	float y;
	float theta;
	float v;
	float a;
	float kappa;
	float t;
}VehiclePosition;



/*******************************************************************************
* 结构体定义 : Trajectory 路径信息
  结构体内容： pointsNum 路径点个数
                       pathPoint 各路径点车辆状态信息
*******************************************************************************/
typedef struct {
	VehiclePosition* pathPoint;
	int pointsNum;
	int showNum;
}Trajectory;



/*******************************************************************************
* 结构体定义 : MpcOutput 控制输出信息
  结构体内容： Xcell 二次规划求解器原始输出结果
                       acceleration  输出加速度
					   frontWheelAngle  输出前轮偏角
*******************************************************************************/
typedef struct {
	MatCT Xcell;  
	float acceleration;
	float frontWheelAngle;
	float velocity;
}MpcOutput;


/*******************************************************************************
* 结构体定义 : 稀疏矩阵格式结构体
说明  : QP解算库输入为稀疏矩阵格式
*******************************************************************************/
typedef struct {
	c_float * CSC_value; //CSC format: None zero values 
	c_int * CSC_i;  //CSC format: Number of None zero elements in each columns
	c_int * CSC_p;//CSC format: The row number for None zero elements
	int counter;
}  CSCValue;



/*******************************************************************************
* 函数名  : MpcVehicleControlInitial
* 描  述  : 算法初始化
* 输  入  : 前后轮间距，预测时域，控制采样周期
* 输  出  :  
* 返回值  :  
*******************************************************************************/
//void MpcVehicleControlInitial(float vehicleShaftDis, int predictHorizon, float sampleTime);



/*******************************************************************************
* 函数名  : MpcVehicleControlProcess
* 描  述  : 算法处理，每帧调用。输入参考路径信息和初始位置，输出控制结果
* 输  入  : VehiclePosition 初始位置结构体，Trajectory 参考路径信息结构体
* 输  出  : MpcOutput 控制输出信息 
* 返回值  : MpcOutput 控制输出信息
*******************************************************************************/
//MpcOutput MpcVehicleControlProcess(VehiclePosition vehiclePosition, Trajectory trajectory);




/*******************************************************************************
注意事项：
（1）在Linux或者Mac环境下运行，请修改QP/osqp_configure.h 中的环境变量 改为：#define IS_LINUX 或 #define IS_MAC
*******************************************************************************/


/*******************************************************************************
Reference: 
[1] "Model Predictive Control of a Mobile Robot Using Linearization" by Kuhne.F et al [2004],
[2] "无人驾驶模型预测控制"龚建伟，第3-3-3节 [2014],       

   基本思路：
1. 状态量X：车辆信息（x,y,theta,v） 控制量U：加速度（a）,前轮偏角（FWA）
2. 根据二自由度车辆运动学模型建立状态量与控制量之间的关系方程，
3. 根据参考路径获得“参考状态量 Xref”和“参考控制量 Uref”，实际车辆状态定义为 “实际状态量 Xreal”，实际控制量“Ureal”  
4. 状态变化量Xerror= Xreal-Xout， 类似有控制变化量Uerror，将第二步中的关系方程转换为Xerror与Uerror之间的关系方程  
5. 路径跟踪前首先找到离初始状态最近的轨迹点，该轨迹点由原始路径中的点线性插值得到，
6. 跟踪路径的目标是使mpcMatrices.Xerror尽可能的小，将问题转化为等价的二次规划问题，  
7. 使用OSQP.org库求解二次规划问题，得到使mpcMatrices.Xerror尽可能小时的mpcMatrices.Uerror值，获得对应的控制量  
*******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
	void MpcVehicleControlInitial(float vehicleShaftDis, int predictHorizon, float sampleTime);

	MpcOutput MpcVehicleControlProcess(VehiclePosition vehiclePosition, Trajectory trajectory);


#ifdef __cplusplus
};
#endif



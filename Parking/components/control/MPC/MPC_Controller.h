#pragma once
//#include<stdio.h>

#include "MPC_light_matrix.h"
#include "memorypool.h"
#include "osqp.h"
 
 
/*******************************************************************************
* �ṹ�嶨�� : VehiclePosition ����״̬��Ϣ
  �ṹ�����ݣ� x,y,theta ��������ϵ�µ�λ����Ϣ
                       v ��ǰ����
					   kappa ������ʰ뾶
					   t �ο�ʱ��
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
* �ṹ�嶨�� : Trajectory ·����Ϣ
  �ṹ�����ݣ� pointsNum ·�������
                       pathPoint ��·���㳵��״̬��Ϣ
*******************************************************************************/
typedef struct {
	VehiclePosition* pathPoint;
	int pointsNum;
	int showNum;
}Trajectory;



/*******************************************************************************
* �ṹ�嶨�� : MpcOutput ���������Ϣ
  �ṹ�����ݣ� Xcell ���ι滮�����ԭʼ������
                       acceleration  ������ٶ�
					   frontWheelAngle  ���ǰ��ƫ��
*******************************************************************************/
typedef struct {
	MatCT Xcell;  
	float acceleration;
	float frontWheelAngle;
	float velocity;
}MpcOutput;


/*******************************************************************************
* �ṹ�嶨�� : ϡ������ʽ�ṹ��
˵��  : QP���������Ϊϡ������ʽ
*******************************************************************************/
typedef struct {
	c_float * CSC_value; //CSC format: None zero values 
	c_int * CSC_i;  //CSC format: Number of None zero elements in each columns
	c_int * CSC_p;//CSC format: The row number for None zero elements
	int counter;
}  CSCValue;



/*******************************************************************************
* ������  : MpcVehicleControlInitial
* ��  ��  : �㷨��ʼ��
* ��  ��  : ǰ���ּ�࣬Ԥ��ʱ�򣬿��Ʋ�������
* ��  ��  :  
* ����ֵ  :  
*******************************************************************************/
//void MpcVehicleControlInitial(float vehicleShaftDis, int predictHorizon, float sampleTime);



/*******************************************************************************
* ������  : MpcVehicleControlProcess
* ��  ��  : �㷨����ÿ֡���á�����ο�·����Ϣ�ͳ�ʼλ�ã�������ƽ��
* ��  ��  : VehiclePosition ��ʼλ�ýṹ�壬Trajectory �ο�·����Ϣ�ṹ��
* ��  ��  : MpcOutput ���������Ϣ 
* ����ֵ  : MpcOutput ���������Ϣ
*******************************************************************************/
//MpcOutput MpcVehicleControlProcess(VehiclePosition vehiclePosition, Trajectory trajectory);




/*******************************************************************************
ע�����
��1����Linux����Mac���������У����޸�QP/osqp_configure.h �еĻ������� ��Ϊ��#define IS_LINUX �� #define IS_MAC
*******************************************************************************/


/*******************************************************************************
Reference: 
[1] "Model Predictive Control of a Mobile Robot Using Linearization" by Kuhne.F et al [2004],
[2] "���˼�ʻģ��Ԥ�����"����ΰ����3-3-3�� [2014],       

   ����˼·��
1. ״̬��X��������Ϣ��x,y,theta,v�� ������U�����ٶȣ�a��,ǰ��ƫ�ǣ�FWA��
2. ���ݶ����ɶȳ����˶�ѧģ�ͽ���״̬���������֮��Ĺ�ϵ���̣�
3. ���ݲο�·����á��ο�״̬�� Xref���͡��ο������� Uref����ʵ�ʳ���״̬����Ϊ ��ʵ��״̬�� Xreal����ʵ�ʿ�������Ureal��  
4. ״̬�仯��Xerror= Xreal-Xout�� �����п��Ʊ仯��Uerror�����ڶ����еĹ�ϵ����ת��ΪXerror��Uerror֮��Ĺ�ϵ����  
5. ·������ǰ�����ҵ����ʼ״̬����Ĺ켣�㣬�ù켣����ԭʼ·���еĵ����Բ�ֵ�õ���
6. ����·����Ŀ����ʹmpcMatrices.Xerror�����ܵ�С��������ת��Ϊ�ȼ۵Ķ��ι滮���⣬  
7. ʹ��OSQP.org�������ι滮���⣬�õ�ʹmpcMatrices.Xerror������Сʱ��mpcMatrices.Uerrorֵ����ö�Ӧ�Ŀ�����  
*******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
	void MpcVehicleControlInitial(float vehicleShaftDis, int predictHorizon, float sampleTime);

	MpcOutput MpcVehicleControlProcess(VehiclePosition vehiclePosition, Trajectory trajectory);


#ifdef __cplusplus
};
#endif



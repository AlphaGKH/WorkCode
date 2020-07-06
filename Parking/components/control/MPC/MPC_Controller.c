#define _CRT_SECURE_NO_WARNINGS

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "MPC_Controller.h"

 
mem_size_t max_mem = 80 * MB + 500 * KB;
mem_size_t mem_pool_size = 40 * MB + 500 * KB;

/*******************************************************************************
输入的全局变量Input global values;
*******************************************************************************/

float Tsample;              /* 控制采样周期 samping time*/
float vehicleShaftDistance; /* 前后轮间距 vehicle shaft distance */
int   Tpredict;             /* 预测时域 preidction horizon  */
int   Tcontrol;             /* 控制时域control horizon */
int   ControlSts;           /* 控制量个数 */

float lowerBoundary[2];     // 二次规划控制量上下限 Lower boundary and upper boundary of the control element differences;
float upperBoundary[2];
float lowerDeltaBoundary[2]; //新增控制增量的变化率
float upperDeltaBoundary[2];

float Q0[6];
float R0[2];

typedef struct {

	MatCT KESIref;    //Reference path point state elements + control elements
	MatCT KESIreal;   //Vehicle current real state elements + control elements
	MatCT KESIerror;  //Difference between KESIref and KESIreal
	MatCT Qcell;      //Weighting matrix for state element 
	MatCT Rcell;      //Weighting matrix for control elements
	MatCT Acell;      //Integrated Acell (State space model)
	MatCT Bcell;      //Integrated Bcell (State space model)
	MatCT Hcell;      //Hessian matrix (Input of the QP)
	MatCT Fcell;      //First order state vector (Input of the QP)
	MatCT Bdcell;     //Boundary cell  lb < Bd*x < ub
	MatCT LBcell;     //LowerBoundary
	MatCT UBcell;     //UpperBoundary
	float delta_a_ref;      // Reference incremental acceleration
	float delta_angle_ref;  // Reference incremental front wheel angle
	
}MpcMatrices;

void CalculateAcell(int start, int end, MpcMatrices *mpcMatrices, MatCT *intermediate);

FILE* lcj;
MpcMatrices SetInputMatrices(VehiclePosition vehiclePosition, Trajectory trajectory_in)
{
	static int flag = 1;
	if (flag)
	{
		lcj = fopen("./data/lcj.csv", "w+");
	}
	flag = 0;
	//transfer the input information into the Matrices. "?path" are the intermediate values used to initialize the Matrices 将路径信息从结构体中取出存入特定矩阵

	MpcMatrices mpcMatrices;

	float x_0;
	float y_0;
	float theta_0;
	float v_0;
	float a_0;
	float kappa_0;

	x_0 = vehiclePosition.x;
	y_0 = vehiclePosition.y;
	theta_0 = vehiclePosition.theta;
    if(theta_0>M_PI){
        theta_0=theta_0-2*M_PI;
    }
	v_0 = vehiclePosition.v;
	a_0 = vehiclePosition.a;
	kappa_0 = vehiclePosition.kappa;

    printf("Current location:  %f, %f, %f, %f, %f, %f \n", tan(kappa_0*2.5)*180/3.1415926*16, x_0, y_0, theta_0*180/ 3.1415926, v_0, a_0 );


	float *xpath;
	float *ypath;
	float *thetapath;
	float *vpath;
	float *apath;
	float *kappapath;
	float *tpath;

	// The sample time of the input path might different from the sample time of control. Rehearse the trajectory path points based on Tsample of control using linear interpolation
	// 输入轨迹的采样周期可能与MPC控制器的采样周期可能不同，首先根据控制器采样周期重新排布输入轨迹（使用最近点两点线性插值）。

	float dmin = trajectory_in.pathPoint[1].t - trajectory_in.pathPoint[0].t;
	int tt;
	for (tt = 1; tt < trajectory_in.pointsNum; tt++) {
//		if (trajectory_in.pathPoint[tt].t - trajectory_in.pathPoint[tt - 1].t - dmin > 0.00001) {
//			printf("Warning： The input path trajectory is not time equally distributed!  %d \n", tt);
//		}
	}

	float Tsample_in;
	Tsample_in = dmin; 

	Trajectory trajectory;
	trajectory.pointsNum = floor((trajectory_in.pointsNum-1)*Tsample_in / Tsample);
	trajectory.pathPoint = (VehiclePosition*)MemoryPool_Alloc(mp,sizeof(VehiclePosition)* trajectory.pointsNum);

	trajectory.pathPoint[0].t = trajectory_in.pathPoint[0].t;
	trajectory.pathPoint[0].x = trajectory_in.pathPoint[0].x;
	trajectory.pathPoint[0].y = trajectory_in.pathPoint[0].y;
	trajectory.pathPoint[0].theta = trajectory_in.pathPoint[0].theta;
    if(trajectory.pathPoint[0].theta>M_PI){
        trajectory.pathPoint[0].theta=trajectory.pathPoint[0].theta;
    }
	trajectory.pathPoint[0].v = trajectory_in.pathPoint[0].v;
	trajectory.pathPoint[0].a = trajectory_in.pathPoint[0].a;
	trajectory.pathPoint[0].kappa = trajectory_in.pathPoint[0].kappa;


	int index_start_0;
	int index_end_0;
	float ratio_0;
	int xy;
	for ( xy= 1; xy < trajectory.pointsNum; xy++) {
		trajectory.pathPoint[xy].t = xy*Tsample;

		index_start_0 = floor(xy*Tsample / Tsample_in);
		index_end_0 = index_start_0 + 1;
        ratio_0 = 1;

		trajectory.pathPoint[xy].x = (trajectory_in.pathPoint[index_start_0].x + ratio_0*(trajectory_in.pathPoint[index_end_0].x-trajectory_in.pathPoint[index_start_0].x));
		trajectory.pathPoint[xy].y = (trajectory_in.pathPoint[index_start_0].y + ratio_0*(trajectory_in.pathPoint[index_end_0].y - trajectory_in.pathPoint[index_start_0].y));
		trajectory.pathPoint[xy].theta = trajectory_in.pathPoint[index_start_0].theta + ratio_0*(trajectory_in.pathPoint[index_end_0].theta - trajectory_in.pathPoint[index_start_0].theta);
		trajectory.pathPoint[xy].v = trajectory_in.pathPoint[index_start_0].v + ratio_0*(trajectory_in.pathPoint[index_end_0].v - trajectory_in.pathPoint[index_start_0].v);
		trajectory.pathPoint[xy].a = trajectory_in.pathPoint[index_start_0].a + ratio_0*(trajectory_in.pathPoint[index_end_0].a - trajectory_in.pathPoint[index_start_0].a);
		trajectory.pathPoint[xy].kappa = (trajectory_in.pathPoint[index_start_0].kappa + ratio_0*(trajectory_in.pathPoint[index_end_0].kappa - trajectory_in.pathPoint[index_start_0].kappa));
		//fprintf(lcj, "%f,%f,%f,%f,", vehiclePosition.x, vehiclePosition.y, vehiclePosition.theta, vehiclePosition.kappa);
		//fprintf(lcj, "%f,%f,%f,%f,%f,%f\n", trajectory.pathPoint[xy].x, trajectory.pathPoint[xy].y, trajectory.pathPoint[xy].theta, trajectory.pathPoint[xy].v, trajectory.pathPoint[xy].a,trajectory.pathPoint[xy].kappa);
		//fflush(lcj);
        if(trajectory.pathPoint[xy].theta>M_PI){
            trajectory.pathPoint[xy].theta=trajectory.pathPoint[xy].theta;
        }
	}

	// First find the nearest point on the given trajectory. 首先找到给定轨迹上与当前车辆状态最接近的点。以此点为实际参考路径的起点。
	int pointer;
	int index_start;
	int index_end;
	int nearestPointNr;
	float Distance;
	float min=100000.0;

	for (pointer = 0; pointer < trajectory.pointsNum; pointer++)
	{
		//Distance = (trajectory.pathPoint[pointer].x - x_0)*(trajectory.pathPoint[pointer].x - x_0)+ (trajectory.pathPoint[pointer].y - y_0)*(trajectory.pathPoint[pointer].y - y_0)+ 500*(trajectory.pathPoint[pointer].kappa - kappa_0)*(trajectory.pathPoint[pointer].kappa - kappa_0);
		Distance = (trajectory.pathPoint[pointer].x - x_0)*(trajectory.pathPoint[pointer].x - x_0) + (trajectory.pathPoint[pointer].y - y_0)*(trajectory.pathPoint[pointer].y - y_0);

		if (Distance < min){
			min = Distance;
			nearestPointNr = pointer;
		}
	}

	// Use the linear interpolation to calculate the first matching point 使用线性插点法寻找初始状态值
    printf("Nearest Point No:  %d , %f,  %f,  %f, %f \n", nearestPointNr, (trajectory.pathPoint[nearestPointNr].x - x_0), (trajectory.pathPoint[nearestPointNr].y - y_0), (trajectory.pathPoint[nearestPointNr].v - v_0));

    index_start = nearestPointNr + 8;
	index_end = nearestPointNr;

	float x_start = trajectory.pathPoint[index_start].x;
	float y_start = trajectory.pathPoint[index_start].y;
	float x_end = trajectory.pathPoint[index_end].x;
	float y_end = trajectory.pathPoint[index_end].y;
	float startEndDistance = (y_end - y_start)*(y_end - y_start)+ (x_end - x_start)*(x_end - x_start) ;

	float ratio = 0;

	/*if (ratio > 1 || ratio < 0)
	{
		printf("Warning:  The initial point is not close to the path trajectory, please check! \n ");
		printf("Ratio:  %f  \n", ratio);
	}*/
	
	trajectory.pointsNum = trajectory.pointsNum - nearestPointNr;

	xpath = (float*)MemoryPool_Alloc(mp,trajectory.pointsNum * sizeof(float));
	ypath = (float*)MemoryPool_Alloc(mp,trajectory.pointsNum  * sizeof(float));
	thetapath = (float*)MemoryPool_Alloc(mp,trajectory.pointsNum  * sizeof(float));
	vpath = (float*)MemoryPool_Alloc(mp,trajectory.pointsNum  * sizeof(float));
	apath = (float*)MemoryPool_Alloc(mp,trajectory.pointsNum  * sizeof(float));
	kappapath = (float*)MemoryPool_Alloc(mp,trajectory.pointsNum  * sizeof(float));

	//xpath[0] = trajectory.pathPoint[index_start].x + ratio * (trajectory.pathPoint[index_end].x - trajectory.pathPoint[index_start].x );
	//ypath[0] = trajectory.pathPoint[index_start].y + ratio* (trajectory.pathPoint[index_end].y - trajectory.pathPoint[index_start].y);
	//thetapath[0] = trajectory.pathPoint[index_start].theta + ratio* (trajectory.pathPoint[index_end].theta - trajectory.pathPoint[index_start].theta);
	//vpath[0] = trajectory.pathPoint[index_start].v + ratio* (trajectory.pathPoint[index_end].v - trajectory.pathPoint[index_start].v);
	//apath[0] = trajectory.pathPoint[index_start].a + ratio* (trajectory.pathPoint[index_end].a - trajectory.pathPoint[index_start].a);
	//kappapath[0] = trajectory.pathPoint[index_start].kappa + ratio* (trajectory.pathPoint[index_end].kappa - trajectory.pathPoint[index_start].kappa);


	xpath[0] = trajectory.pathPoint[index_start].x ;
	ypath[0] = trajectory.pathPoint[index_start].y ;
	thetapath[0] = trajectory.pathPoint[index_start].theta ;
	vpath[0] = trajectory.pathPoint[index_start].v ;
	apath[0] = trajectory.pathPoint[index_start].a ;
	kappapath[0] = trajectory.pathPoint[index_start].kappa ;

	int i;

	for (i = 1; i < trajectory.pointsNum ; i++)
	{
		xpath[i] = trajectory.pathPoint[i+ nearestPointNr].x;
		ypath[i] = trajectory.pathPoint[i + nearestPointNr].y;
		thetapath[i] = trajectory.pathPoint[i + nearestPointNr].theta;
		vpath[i] = trajectory.pathPoint[i + nearestPointNr].v;
		apath[i] = trajectory.pathPoint[i + nearestPointNr].a;
		kappapath[i] = trajectory.pathPoint[i + nearestPointNr].kappa;
		//fprintf(lcj, "%f,%f,%f,%f,", vehiclePosition.x, vehiclePosition.y, vehiclePosition.theta, vehiclePosition.kappa);
		//fprintf(lcj, "%f,%f,%f,%f,%f,%f\n", xpath[i], ypath[i], thetapath[i], kappapath[i], vpath[i], apath[i]);
		//fflush(lcj);
	}
	

	//Initialize Kesi_ref Matrix
	MatCT Kesiref;
	MatCreate(&Kesiref, 6, trajectory.pointsNum);
	MatCreate(&mpcMatrices.KESIref, trajectory.pointsNum, 6);

	float* valll;
	valll = (float*)MemoryPool_Alloc(mp, 6 * trajectory.pointsNum * sizeof(float));

	for (i = 0; i < trajectory.pointsNum; i++) {
		valll[i] = xpath[i];
		valll[trajectory.pointsNum + i] = ypath[i];
		valll[2 * trajectory.pointsNum + i] = thetapath[i];
		valll[3 * trajectory.pointsNum + i] = vpath[i];
		valll[4 * trajectory.pointsNum + i] = apath[i];
        valll[5 * trajectory.pointsNum + i] = atan(kappapath[i] * vehicleShaftDistance)*1.2;
	}

	MatTrans(MatSetVal(&Kesiref, valll), &mpcMatrices.KESIref);
	MatDelete(&Kesiref);

	if (mpcMatrices.KESIref.row == 1)
	{
		printf(" Error: The input trajectory or initial point cannot be recognized \n The initial point might be too far away from the path trajectory. \n\n");
	}

	//Initialize KESIreal KESIerror
	MatZeros(MatCreate(&mpcMatrices.KESIreal, mpcMatrices.KESIref.col, 1));
	MatZeros(MatCreate(&mpcMatrices.KESIerror, mpcMatrices.KESIref.col, 1));

	float kesi0[6];
	kesi0[0] = x_0;
	kesi0[1] = y_0;
	kesi0[2] = theta_0;
	kesi0[3] = v_0;
	kesi0[4] = a_0;
	kesi0[5] = atan(kappa_0 * vehicleShaftDistance);

	MatSetVal(&mpcMatrices.KESIreal, kesi0);

	float dkesi0[6];
	dkesi0[0] = x_0 - mpcMatrices.KESIref.element[0][0];
	dkesi0[1] = y_0 - mpcMatrices.KESIref.element[0][1];
	dkesi0[2] = theta_0 - mpcMatrices.KESIref.element[0][2];
	dkesi0[3] = v_0 - mpcMatrices.KESIref.element[0][3];
	dkesi0[4] = a_0 - mpcMatrices.KESIref.element[0][4];
	dkesi0[5] = atan(kappa_0 * vehicleShaftDistance) - mpcMatrices.KESIref.element[0][5];

	MatSetVal(&mpcMatrices.KESIerror, dkesi0);

	//Caculate reference incremental value

	mpcMatrices.delta_a_ref = 0.0;
	mpcMatrices.delta_angle_ref = 0.0;
	if (nearestPointNr > 0) {
		mpcMatrices.delta_a_ref = trajectory.pathPoint[nearestPointNr].a - trajectory.pathPoint[nearestPointNr - 1].a;
		mpcMatrices.delta_angle_ref = atan(trajectory.pathPoint[nearestPointNr].kappa * vehicleShaftDistance) - atan(trajectory.pathPoint[nearestPointNr - 1].kappa * vehicleShaftDistance);
	}

	MemoryPool_Free(mp, valll);
	MemoryPool_Free(mp, trajectory.pathPoint);
	MemoryPool_Free(mp, xpath);
	MemoryPool_Free(mp, ypath);
	MemoryPool_Free(mp, thetapath);
	MemoryPool_Free(mp, vpath);
	MemoryPool_Free(mp, apath);
	MemoryPool_Free(mp, kappapath);
	valll = NULL;
	trajectory.pathPoint = NULL;
	xpath = NULL;
	ypath = NULL;
	thetapath = NULL;
	vpath = NULL;
	apath = NULL;
	kappapath = NULL;

	//Calculate Qcell Rcell (weighting matrices)
	MatZeros(MatCreate(&mpcMatrices.Qcell, mpcMatrices.KESIref.col*Tpredict, mpcMatrices.KESIref.col*Tpredict));

	int i_q, j_q;
	for (i_q = 0; i_q < Tpredict; i_q++) {
		for (j_q = 0; j_q < 6; j_q++) {
			mpcMatrices.Qcell.element[6 * i_q + j_q][6 * i_q + j_q] = Q0[j_q];
		}
	}

	MatZeros(MatCreate(&mpcMatrices.Rcell, ControlSts* Tcontrol, ControlSts * Tcontrol));
	int i_r, j_r;
	for (i_r = 0; i_r < Tcontrol; i_r++) {
		for (j_r = 0; j_r < ControlSts; j_r++) {
			mpcMatrices.Rcell.element[ControlSts * i_r + j_r][ControlSts * i_r + j_r] = R0[j_r];
		}
	}

	//  Calculate Acell
	//  Format of Acell
	//  [         A(0)
	//	        A(1)*A(0)
	//	             ---
	//	       A(N)***A(0)   ]

	MatZeros(MatCreate(&mpcMatrices.Acell, mpcMatrices.KESIref.col*Tpredict, mpcMatrices.KESIref.col));

	CalculateAcell(0, Tpredict, &mpcMatrices, &mpcMatrices.Acell);

	// Calculate Bcell
	//    Format of Bcell: N=Tpredict   Nc=Tcontrol  
	//   [     B(0)                                        0                                ---             0
	//         A(1)*B(0)                              B(1)                              ---             0
	//			 ---                                       ---                                ---            ---
	//	        A(N-1)***A(1)*B(0)              A(N-1)***A(2)*B(1)       ---           A(N-1)***A(Nc)*B(Nc)]
 
	MatZeros(MatCreate(&mpcMatrices.Bcell, mpcMatrices.KESIref.col*Tpredict, ControlSts*Tcontrol));
	int x, y, p, q;
	for (x = 0; x < Tpredict; x++) {
		for (y = 0; y < Tcontrol; y++) {
			if (x >= y) {
				MatCT AAcell;
				if (x == y) {
					MatEye(MatCreate(&AAcell, mpcMatrices.KESIref.col, mpcMatrices.KESIref.col));
				}
				else {
					MatZeros(MatCreate(&AAcell, mpcMatrices.KESIref.col, mpcMatrices.KESIref.col));
					CalculateAcell(y + 1, x + 1, &mpcMatrices, &AAcell);
				}

				MatCT BBcell;
				MatZeros(MatCreate(&BBcell, mpcMatrices.KESIref.col, ControlSts));
				BBcell.element[2][1] = mpcMatrices.KESIref.element[y][3] * Tsample / (cos(mpcMatrices.KESIref.element[y][5])*cos(mpcMatrices.KESIref.element[y][5])) / vehicleShaftDistance;
				BBcell.element[3][0] = Tsample;
				BBcell.element[4][0] = 1.0;
				BBcell.element[5][1] = 1.0;

				MatCT ABcell;
				MatZeros(MatCreate(&ABcell, mpcMatrices.KESIref.col, ControlSts));
				MatMul(&AAcell, &BBcell, &ABcell);

				for (p = 0; p < mpcMatrices.KESIref.col; p++)
				{
					for (q = 0; q < ControlSts; q++)
					{
						mpcMatrices.Bcell.element[p + x*mpcMatrices.KESIref.col][q + y*ControlSts] = ABcell.element[p][q];
					}
				}
				MatDelete(&AAcell);
				MatDelete(&BBcell);
				MatDelete(&ABcell);
			}
		}
	}



	/* calculate Hcell Hessian matrix, it should be positive definite */

	MatCT Btran;
	MatCT BtranQ;
	MatCT BtranQB;
	MatCT BQBplusR;
	MatCreate(&Btran, ControlSts*Tcontrol, mpcMatrices.KESIref.col*Tpredict);
	MatCreate(&BtranQ, ControlSts*Tcontrol, mpcMatrices.KESIref.col*Tpredict);
	MatCreate(&BtranQB, ControlSts*Tcontrol, ControlSts*Tcontrol);
	MatCreate(&BQBplusR, ControlSts*Tcontrol, ControlSts*Tcontrol);

	MatEye(MatCreate(&mpcMatrices.Hcell, ControlSts*Tcontrol, ControlSts*Tcontrol));

	MatTrans(&mpcMatrices.Bcell, &Btran);

	MatMul(&Btran, &mpcMatrices.Qcell, &BtranQ);
	MatMul(&BtranQ, &mpcMatrices.Bcell, &BtranQB);
	MatAdd(&BtranQB, &mpcMatrices.Rcell, &BQBplusR);

	int ij, ji, hh;
	for (ij = 0; ij < BQBplusR.row; ij++)
	{
		for (ji = 0; ji < BQBplusR.col; ji++)
		{
			mpcMatrices.Hcell.element[ij][ji] = BQBplusR.element[ij][ji];
		}
	}

	//for (hh = 0; hh < ControlSts * Tcontrol; hh++) {
	//	mpcMatrices.Hcell.element[ControlSts * Tcontrol+hh][ControlSts * Tcontrol + hh] = 1000;     //Relax factor: Not sure what it means to QP.
	//}


	/*Calculate fcell*/

	MatCT AKesierror;
	MatCT AKesierrorTran;
	MatCT AKesierrorTranQ;
	MatCT AKesierrorTranQB;

	MatCreate(&AKesierror, mpcMatrices.KESIref.col*Tpredict, 1);
	MatCreate(&AKesierrorTran, 1, mpcMatrices.KESIref.col*Tpredict);
	MatCreate(&AKesierrorTranQ, 1, mpcMatrices.KESIref.col*Tpredict);
	MatCreate(&AKesierrorTranQB, 1, ControlSts * Tcontrol);

	MatZeros(MatCreate(&mpcMatrices.Fcell, 1, ControlSts * Tcontrol)); //暂时不考虑 relax factor

	MatMul(&mpcMatrices.Acell, &mpcMatrices.KESIerror, &AKesierror);
	MatTrans(&AKesierror, &AKesierrorTran);
	MatMul(&AKesierrorTran, &mpcMatrices.Qcell, &AKesierrorTranQ);
	MatMul(&AKesierrorTranQ, &mpcMatrices.Bcell, &AKesierrorTranQB);

	int ff;
	for (ff = 0; ff<AKesierrorTranQB.col; ff++)
	{
		mpcMatrices.Fcell.element[0][ff] = 2 * AKesierrorTranQB.element[0][ff];
	}

	MatDelete(&Btran);
	MatDelete(&BtranQ);
	MatDelete(&BtranQB);
	MatDelete(&BQBplusR);
	MatDelete(&AKesierror);
	MatDelete(&AKesierrorTran);
	MatDelete(&AKesierrorTranQ);
	MatDelete(&AKesierrorTranQB);



	/* Calculate Bdcell: Include the boundary information for (u) and delta(u)*/

	MatZeros(MatCreate(&mpcMatrices.Bdcell, 2 * ControlSts * Tcontrol, ControlSts * Tcontrol));
	mpcMatrices.Bdcell.element[0][0] = 1;
	mpcMatrices.Bdcell.element[1][1] = 1;
	mpcMatrices.Bdcell.element[2][2] = 1;
	mpcMatrices.Bdcell.element[3][3] = 1;
	mpcMatrices.Bdcell.element[4][4] = 1;
	mpcMatrices.Bdcell.element[5][5] = 1;

	mpcMatrices.Bdcell.element[6][0] = 1;
	mpcMatrices.Bdcell.element[7][1] = 1;
	mpcMatrices.Bdcell.element[8][2] = 1;
	mpcMatrices.Bdcell.element[9][3] = 1;
	mpcMatrices.Bdcell.element[10][4] = 1;
	mpcMatrices.Bdcell.element[11][5] = 1;

	mpcMatrices.Bdcell.element[2][0] = 1;
	mpcMatrices.Bdcell.element[3][1] = 1;
	mpcMatrices.Bdcell.element[4][2] = 1;
	mpcMatrices.Bdcell.element[5][3] = 1;

	mpcMatrices.Bdcell.element[4][0] = 1;
	mpcMatrices.Bdcell.element[5][1] = 1;
	
	//Caculate LBcell
	MatZeros(MatCreate(&mpcMatrices.LBcell, 2 * ControlSts * Tcontrol, 1));
	mpcMatrices.LBcell.element[0][0] = lowerBoundary[0] - mpcMatrices.KESIreal.element[4][0];
	mpcMatrices.LBcell.element[1][0] = lowerBoundary[1] - mpcMatrices.KESIreal.element[5][0];
	mpcMatrices.LBcell.element[2][0] = lowerBoundary[0] - mpcMatrices.KESIreal.element[4][0];
	mpcMatrices.LBcell.element[3][0] = lowerBoundary[1] - mpcMatrices.KESIreal.element[5][0];
	mpcMatrices.LBcell.element[4][0] = lowerBoundary[0] - mpcMatrices.KESIreal.element[4][0];
	mpcMatrices.LBcell.element[5][0] = lowerBoundary[1] - mpcMatrices.KESIreal.element[5][0];
	mpcMatrices.LBcell.element[6][0] = lowerDeltaBoundary[0];
	mpcMatrices.LBcell.element[7][0] = lowerDeltaBoundary[1];
	mpcMatrices.LBcell.element[8][0] = lowerDeltaBoundary[0];
	mpcMatrices.LBcell.element[9][0] = lowerDeltaBoundary[1];
	mpcMatrices.LBcell.element[10][0] = lowerDeltaBoundary[0];
	mpcMatrices.LBcell.element[11][0] = lowerDeltaBoundary[1];
	
	//Caculate UBcell
	MatZeros(MatCreate(&mpcMatrices.UBcell, 2 * ControlSts * Tcontrol, 1));
	mpcMatrices.UBcell.element[0][0] = upperBoundary[0] - mpcMatrices.KESIreal.element[4][0];
	mpcMatrices.UBcell.element[1][0] = upperBoundary[1] - mpcMatrices.KESIreal.element[5][0];
	mpcMatrices.UBcell.element[2][0] = upperBoundary[0] - mpcMatrices.KESIreal.element[4][0];
	mpcMatrices.UBcell.element[3][0] = upperBoundary[1] - mpcMatrices.KESIreal.element[5][0];
	mpcMatrices.UBcell.element[4][0] = upperBoundary[0] - mpcMatrices.KESIreal.element[4][0];
	mpcMatrices.UBcell.element[5][0] = upperBoundary[1] - mpcMatrices.KESIreal.element[5][0];
	mpcMatrices.UBcell.element[6][0] = upperDeltaBoundary[0];
	mpcMatrices.UBcell.element[7][0] = upperDeltaBoundary[1];
	mpcMatrices.UBcell.element[8][0] = upperDeltaBoundary[0];
	mpcMatrices.UBcell.element[9][0] = upperDeltaBoundary[1];
	mpcMatrices.UBcell.element[10][0] = upperDeltaBoundary[0];
	mpcMatrices.UBcell.element[11][0] = upperDeltaBoundary[1];

    char h[6] = "error";
    printf("%s ", h);
    MatDump(&mpcMatrices.KESIerror);

    char gr[6] = "real";
    printf("%s ", gr);
    MatDump(&mpcMatrices.KESIreal);

	return mpcMatrices;

}

void CalculateAcell(int start, int end, MpcMatrices *mpcMatrices, MatCT *intermediate)
{
	int kk;
	int xx, yy;
	MatCT AA;
	MatCT AAA;
	MatCT AAAA;
	MatEye(MatCreate(&AA, mpcMatrices->KESIref.col, mpcMatrices->KESIref.col));
	MatEye(MatCreate(&AAA, mpcMatrices->KESIref.col, mpcMatrices->KESIref.col));
	MatEye(MatCreate(&AAAA, mpcMatrices->KESIref.col, mpcMatrices->KESIref.col));

	if (end <= start)
	{
		printf(" Error: The input of CalculateAcell function is wrong");
	}

	for (kk = start; kk < end; kk++) {

		AA.element[0][2] = (-mpcMatrices->KESIref.element[kk][3] * sin(mpcMatrices->KESIref.element[kk][2])*Tsample);

		AA.element[1][2] = (mpcMatrices->KESIref.element[kk][3] * cos(mpcMatrices->KESIref.element[kk][2])*Tsample);

		AA.element[0][3] = cos(mpcMatrices->KESIref.element[kk][2])* Tsample;

		AA.element[1][3] = sin(mpcMatrices->KESIref.element[kk][2])* Tsample;

		AA.element[2][3] = tan(mpcMatrices->KESIref.element[kk][5]) * Tsample / vehicleShaftDistance;

		AA.element[2][5] = mpcMatrices->KESIref.element[kk][3] * Tsample / (cos(mpcMatrices->KESIref.element[kk][5])*cos(mpcMatrices->KESIref.element[kk][5])) / vehicleShaftDistance;

		AA.element[3][4] = Tsample;

		if (kk == 0) {
			MatCopy(&AA, &AAA);
		}
		else {
			MatMul(&AA, &AAA, &AAAA);
			MatCopy(&AAAA, &AAA);
		}

		if (start == 0) {
			for (xx = 0; xx < AA.row; xx++) {
				for (yy = 0; yy < AA.col; yy++) {
					intermediate->element[xx + 6 * kk][yy] = AAA.element[xx][yy];
				}
			}
		}
	}

	if (start > 0) {
		for (xx = 0; xx < AA.row; xx++) {
			for (yy = 0; yy < AA.col; yy++) {
				intermediate->element[xx][yy] = AAA.element[xx][yy];
			}
		}
	}

	MatDelete(&AAAA);
	MatDelete(&AAA);
	MatDelete(&AA);
}

void MatrixToCSC(MatCT* Input, CSCValue* Output)
{
	int i, j;
	int tt = 0;
	Output->counter = 0;
	for (i = 0; i < Input->row; i++)
	{
		for (j = 0; j < Input->col; j++)
		{
			if (Input->element[i][j] != 0)
			{
				Output->counter++;
			}
		}
	}
	Output->CSC_value = (c_float*)MemoryPool_Alloc(mp, Output->counter * sizeof(c_float));
	Output->CSC_i = (c_int*)MemoryPool_Alloc(mp, Output->counter * sizeof(c_int));
	Output->CSC_p = (c_int*)MemoryPool_Alloc(mp, (Input->col + 1) * sizeof(c_int));

	for (j = 0; j < Input->col; j++)
	{
		Output->CSC_p[0] = 0;
		for (i = 0; i < Input->row; i++)
		{
			if (Input->element[i][j] != 0)
			{
				Output->CSC_value[tt] = Input->element[i][j];
				Output->CSC_i[tt] = i;
				tt++;
			}
		}
		Output->CSC_p[j + 1] = tt;
	}

}

MpcOutput QPsolve(MpcMatrices mpcMatrices)
{
	//First transfer the normal Hcell matrices into CSC format(Compressed sparse columns format ) 
	//根据OSQP二次规划求解器的输入要求，首先将输入的Hcell矩阵转换为CSC稀疏矩阵格式
	//  n    number of variables n
	//  m    number of constraints m
	//  *P   the upper triangular part of the quadratic cost matrix P in csc format(size n x n).
	//  *A   linear constraints matrix A in csc format(size m x n)
	//  *q   dense array for linear part of cost function(size n)
	//  *l   dense array for lower bound(size m)
	//  *u   dense array for upper bound(size m)


	CSCValue HcellCSC;
	MatrixToCSC(&mpcMatrices.Hcell, &HcellCSC);

	CSCValue BdcellCSC;   //Bdcell 也就是 QP中的A矩阵
	MatrixToCSC(&mpcMatrices.Bdcell, &BdcellCSC);


	//solve the QP problems
	//求解二次规划问题
	//Reference: osqp.org/docs/ 
	c_int   P_nnz = -1;
	c_int   A_nnz = -1;

	c_float *q = (c_float*)MemoryPool_Alloc(mp, mpcMatrices.Fcell.col * sizeof(c_float));

	int ii;
	for (ii = 0; ii < mpcMatrices.Fcell.col; ii++)
	{
		q[ii] = mpcMatrices.Fcell.element[0][ii];
	}


	c_float *l = (c_float*)MemoryPool_Alloc(mp, 2 * ControlSts*Tcontrol * sizeof(c_float));
	c_float *u = (c_float*)MemoryPool_Alloc(mp, 2 * ControlSts*Tcontrol * sizeof(c_float));
	int lu;
	for (lu = 0; lu < 2 * ControlSts*Tcontrol; lu++)
	{
		l[lu] = mpcMatrices.LBcell.element[lu][0];
		u[lu] = mpcMatrices.UBcell.element[lu][0];
	}

	c_int n = mpcMatrices.Hcell.col;
	c_int m = 2 * ControlSts*Tcontrol;

	// Problem settings
	OSQPSettings *settings = (OSQPSettings *)MemoryPool_Alloc(mp, sizeof(OSQPSettings));

	// Structures
	OSQPWorkspace *work; // Workspace
	OSQPData *data;      // OSQPData

						 // Populate data
	data = (OSQPData *)MemoryPool_Alloc(mp, sizeof(OSQPData));
	data->n = n;
	data->m = m;
	data->P = csc_matrix(data->n, data->n, P_nnz, HcellCSC.CSC_value, HcellCSC.CSC_i, HcellCSC.CSC_p);
	data->q = q;
	data->A = csc_matrix(data->m, data->n, A_nnz, BdcellCSC.CSC_value, BdcellCSC.CSC_i, BdcellCSC.CSC_p);
	data->l = l;
	data->u = u;

	// Define Solver settings as default                                                                                                                                    
	osqp_set_default_settings(settings);

	// Setup workspace
	work = osqp_setup(data, settings);

	MpcOutput mpcOutput;

	// Solve Problem
	osqp_solve(work, &mpcOutput.Xcell);

    //MatDump(&mpcOutput.Xcell);

    mpcOutput.acceleration = mpcOutput.Xcell.element[0][0] + mpcMatrices.KESIref.element[0][4] + mpcMatrices.delta_a_ref;

    mpcOutput.frontWheelAngle = mpcOutput.Xcell.element[1][0] + mpcMatrices.KESIref.element[0][5] + mpcMatrices.delta_angle_ref;

    mpcOutput.velocity = mpcMatrices.KESIref.element[0][3] + mpcOutput.acceleration*Tsample;

    if(mpcOutput.acceleration>10){
        mpcOutput.acceleration =  mpcMatrices.KESIref.element[0][4] ;

        mpcOutput.frontWheelAngle =  mpcMatrices.KESIref.element[0][5] ;

        mpcOutput.velocity = mpcMatrices.KESIref.element[0][3] + mpcOutput.acceleration*Tsample;
    }
    printf(" Current/Ref Acceleration: %f ,%f [m/s^2] \n\n", mpcOutput.acceleration, mpcMatrices.KESIref.element[0][4]);

    printf(" Current/Ref FrontWheelAn: %f  ,%f [rad] \n\n", mpcOutput.frontWheelAngle*16*180/3.1415926, mpcMatrices.KESIref.element[0][5]*16*180/3.1415926 );

    printf(" Current/Ref OutputVeloci: %f  ,%f [m/s] \n\n", mpcOutput.velocity, mpcMatrices.KESIref.element[0][3]);

	// Clean workspace
	osqp_cleanup(work);

	//MemoryPool_Free(mp,data->A);
	//MemoryPool_Free(mp,data->P);
	MemoryPool_Free(mp, data);
	MemoryPool_Free(mp, settings);

	MemoryPool_Free(mp, HcellCSC.CSC_value);
	MemoryPool_Free(mp, HcellCSC.CSC_i);
	MemoryPool_Free(mp, HcellCSC.CSC_p);
	MemoryPool_Free(mp, q);
	MemoryPool_Free(mp, BdcellCSC.CSC_value);
	MemoryPool_Free(mp, BdcellCSC.CSC_i);
	MemoryPool_Free(mp, BdcellCSC.CSC_p);
	MemoryPool_Free(mp, l);
	MemoryPool_Free(mp, u);

	data = NULL;
	settings = NULL;
	HcellCSC.CSC_value = NULL;
	HcellCSC.CSC_i = NULL;
	HcellCSC.CSC_p = NULL;
	q = NULL;
	BdcellCSC.CSC_value = NULL;
	BdcellCSC.CSC_i = NULL;
	BdcellCSC.CSC_p = NULL;
	l = NULL;
	u = NULL;

	return mpcOutput;
}


void PrintMpcMatrices(MpcMatrices mpcMatrices)
{
	//char ee[6] = "Qcell";
	//printf("%s ", ee);
	//MatDump(&mpcMatrices.Qcell);

	//char eee[6] = "Rcell";
	//printf("%s ", eee);
	//MatDump(&mpcMatrices.Rcell);

	//char e[6] = "Acell";
	//printf("%s ", e);
	//MatDump(&mpcMatrices.Acell);

	//char f[6] = "Bcell";
	//printf("%s ", f);
	//MatDump(&mpcMatrices.Bcell);

	//char g[6] = "Hcell";
	//printf("%s ", g);
	//MatDump(&mpcMatrices.Hcell);

	//char h[6] = "fcell";
	//printf("%s ", h);
	//MatDump(&mpcMatrices.Fcell);

}

void CleanMpcMatrices(MpcMatrices mpcMatrices)
{
	MatDelete(&mpcMatrices.Acell);
	MatDelete(&mpcMatrices.Bcell);
	MatDelete(&mpcMatrices.Qcell);
	MatDelete(&mpcMatrices.Rcell);
	MatDelete(&mpcMatrices.Hcell);
	MatDelete(&mpcMatrices.Fcell);
	MatDelete(&mpcMatrices.KESIref);
	MatDelete(&mpcMatrices.KESIreal);
	MatDelete(&mpcMatrices.KESIerror);
	MatDelete(&mpcMatrices.Bdcell);
	MatDelete(&mpcMatrices.LBcell);
	MatDelete(&mpcMatrices.UBcell);

}

//Internal State simulator 内置状态估计器 使用状态估计器时，应对SetInputMatrices与QPsolve进行循环

//In reality, the updated states of the vehicle should be acquired 
//from the sensor, In this controller, a kinematic model (Vehicle 2-dof kinematic model)
//is used to simulate the updated vehicle states.

// In this algorithm, vehicle's kinematic model is used.
// The partial differencial equations are solved analytically from Matlab.
// Therefore,  the equations below are directly given.
// If the model changes, the equations used should also be changed.

//void PDEsolvePrep()
//{
//	float vd11;
//	float vd22;
//	float X00;
//	float Y00;
//	float Z00;
//
//	mpcMatrices.Uerror.element[0][0] = Xcell.element[0][0];
//	mpcMatrices.Uerror.element[0][1] = Xcell.element[1][0];
//
//	vd11 = mpcMatrices.Uerror.element[0][0] + mpcMatrices.Uref.element[0][0];
//	vd22 = tan(mpcMatrices.Uerror.element[0][1] + mpcMatrices.Uref.element[0][1]) / vehicleShaftDistance;
//	//printf(" vd11= %f \n", vd11);
//	//printf(" vd22= %f \n", vd22);
//
//	X00 = mpcMatrices.Xreal.element[0][0];
//	Y00 = mpcMatrices.Xreal.element[0][1];
//	Z00 = mpcMatrices.Xreal.element[0][2];
//
//	//printf(" X00= %f \n", X00);
//	//printf(" Y00= %f \n", Y00);
//	//printf(" Z00= %f \n", Z00);
//
//	//Below is the solved equations
//	mpcMatrices.Xreal.element[0+1][0] = X00 - vd11*sin(Z00) / (vd11*vd22) + vd11*sin(Z00 + vd11*vd22*Tsample) / (vd11*vd22);
//	mpcMatrices.Xreal.element[0+1][1] = Y00 + vd11*cos(Z00) / (vd11*vd22) - vd11*cos(Z00 + vd11*vd22*Tsample) / (vd11*vd22);
//	mpcMatrices.Xreal.element[0+1][2] = Z00 + vd11*vd22*Tsample;
//
//
//    //Update all the cells:
//	mpcMatrices.Ureal.element[0][0] = mpcMatrices.Uref.element[0][0] + mpcMatrices.Uerror.element[0][0];
//	mpcMatrices.Ureal.element[0][1] = mpcMatrices.Uref.element[0][1] + mpcMatrices.Uerror.element[0][1];
//	
//	mpcMatrices.Xerror.element[0 + 1][0] = mpcMatrices.Xreal.element[0 + 1][0] - mpcMatrices.Xref.element[0 + 1][0];
//	mpcMatrices.Xerror.element[0 + 1][1] = mpcMatrices.Xreal.element[0 + 1][1] - mpcMatrices.Xref.element[0 + 1][1];
//	mpcMatrices.Xerror.element[0 + 1][2] = mpcMatrices.Xreal.element[0 + 1][2] - mpcMatrices.Xref.element[0 + 1][2];
//
//	char e[9] = "Up mpcMatrices.Xreal";
//	printf("%s ", e);
//	MatDump(&mpcMatrices.Xreal);
//
//	char f[9] = "Up mpcMatrices.Uerror";
//	printf("%s ", f);
//	MatDump(&mpcMatrices.Uerror);
//
//	char g[9] = "Up mpcMatrices.Ureal";
//	printf("%s ", g);
//	MatDump(&mpcMatrices.Ureal);
//}
//
//void GetOutputControl()
//{
//	MpcOutput mpcOutput;
//
//	accel = (float*)malloc(mpcMatrices.Uerror.row * sizeof(float));
//	angle = (float*)malloc(mpcMatrices.Uerror.row * sizeof(float));
//
//	int i, j;
//
//	printf("============== Useful Output Info ============== \n \n");
//	printf("\n Output acceleration [m/s^2] : \n");
//	for (i = 0; i < mpcMatrices.Uerror.row-1;i++)
//	{
//		accel[i] = (mpcMatrices.Uref.element[i+1][0]+ mpcMatrices.Uerror.element[i + 1][0] - mpcMatrices.Uref.element[i][0]- mpcMatrices.Uerror.element[i][0])/ Tsample;
//		printf(" Point%d : %f \n",i+1, accel[i]);
//	}
//	
//	mpcOutput.acceleration = accel[0];
//
//	printf("\n Front wheel angle [rad] : \n");
//	for (j = 0; j < mpcMatrices.Uerror.row-1; j++)
//	{
//		angle[j] = mpcMatrices.Uref.element[j][1]+mpcMatrices.Uerror.element[j][1];
//		printf(" Point%d : %f \n",j+1, angle[j]);
//	}
//
//	mpcOutput.acceleration = angle[0];
//
//}


/*******************************************************************************
* 函数名  : MpcVehicleControlInitial
* 描  述  : 算法初始化
* 输  入  : 前后轮间距，预测时域，控制采样周期
* 输  出  :
* 返回值  :
*******************************************************************************/
void MpcVehicleControlInitial(float vehicleShaftDis, int predictHorizon, float sampleTime)
{
	vehicleShaftDistance = vehicleShaftDis;  //Distance between the vehicle's front shaft and rear shaft （车前后轴间距）
	Tpredict = predictHorizon;
	Tcontrol = floor(predictHorizon / 4);        //  1<=  Control horizon <= Predict horizon; recommend value: 0.2*Tpredict (at least equals to 2-3);

	Tsample = sampleTime;        /*samping time, this should be same as the updated time of the path point*/
	lowerBoundary[0] = -2.0f;   //lower boundary for velocity_control- velocity_path
	lowerBoundary[1] = -0.5454f;  //lower boundary for front/wheel/angel_control- front/wheel/angel_path
	upperBoundary[0] = 1.0f;
	upperBoundary[1] = 0.5454f;
	lowerDeltaBoundary[0] = -0.04f;
    lowerDeltaBoundary[1] = -0.0044f;
	upperDeltaBoundary[0] = 0.02f;
    upperDeltaBoundary[1] = 0.0044f;

	ControlSts = 2;

    Q0[0] = 1.0;
    Q0[1] = 4.0;
    Q0[2] = 4.0;
	Q0[3] = 2.0;
    Q0[4] = 0.5;
    Q0[5] = 0.3;

    R0[0] = 0.5;
    R0[1] = 0.5;

	mp = MemoryPool_Init(max_mem, mem_pool_size, 0); //加入内存池，优化多次malloc造成的计算能力流失 mp定义再 memorypool.h中

	printf(" MPC Initialize finishing \n \n");
}


/*******************************************************************************
* 函数名  : MpcVehicleControlProcess
* 描  述  : 算法处理，每帧调用。输入参考路径信息和初始位置，输出控制结果
* 输  入  : VehiclePosition 初始位置结构体，Trajectory 参考路径信息结构体
* 输  出  : MpcOutput 控制输出信息
* 返回值  : MpcOutput 控制输出信息
*******************************************************************************/
MpcOutput MpcVehicleControlProcess(VehiclePosition vehiclePosition, Trajectory trajectory)
{
	/*FILE* fp = fopen("123.csv", "w");

	for (size_t i = 0; i < trajectory.pointsNum; i++)
	{
		fprintf(fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", trajectory.pathPoint[i].x, trajectory.pathPoint[i].y, trajectory.pathPoint[i].theta, trajectory.pathPoint[i].v, trajectory.pathPoint[i].a, \
			trajectory.pathPoint[i].kappa, trajectory.pathPoint[i].t, vehiclePosition.x, vehiclePosition.y, vehiclePosition.theta, vehiclePosition.v);
		fflush(fp);
	}*/

	MpcMatrices updatedMatrices;

	updatedMatrices = SetInputMatrices(vehiclePosition, trajectory); //return updated Xref Uref Xreal Ureal Xerror Uerror Qcell Rcell Acell Bcell Hcell fcell

	//PrintMpcMatrices(updatedMatrices); 

	MpcOutput updatedOutput;
	updatedOutput= QPsolve(updatedMatrices);

	CleanMpcMatrices(updatedMatrices);
	
	////Following functions are used when the internal state simulator is used.

	//PDEsolvePrep();
	//GetOutputControl();

	mp = MemoryPool_Clear(mp);

	return updatedOutput;

}



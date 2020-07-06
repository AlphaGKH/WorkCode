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

float lowerBoundary[2];     // 二次规划控制量上下限 Lower boundary and upper boundary of the control element differences;
float upperBoundary[2];

typedef struct {
	MatCT Xref;       //Reference path point state elements
	MatCT Uref;       //Reference path point control elements
	MatCT Xreal;      //Vehicle current real state elements
	MatCT Ureal;      //Vehicle current real control elements
	MatCT Xerror;     //Difference between Xref and Xreal
	MatCT Uerror;     //Difference between Uref and Ureal
	MatCT Qcell;      //Weighting matrix for state element 
	MatCT Rcell;      //Weighting matrix for control elements
	MatCT Acell;      //Integrated Acell (State space model)
	MatCT Bcell;      //Integrated Bcell (State space model)
	MatCT Hcell;      //Hessian matrix (Input of the QP)
	MatCT Fcell;      //First order state vector (Input of the QP)
}MpcMatrices;

void CalculateAcell(int start, int end, MpcMatrices *mpcMatrices, MatCT *intermediate);

//FILE* lcj;
MpcMatrices SetInputMatrices(VehiclePosition vehiclePosition, Trajectory trajectory_in)
{
	static int flag = 1;
	if (flag)
	{
//		lcj = fopen("./data/lcj.csv", "w+");
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

    printf("Current location:  %f, %f, %f, %f, %f \n", tan(kappa_0*2.5)*180/3.1415926*16, x_0, y_0, theta_0*180/ 3.1415926, v_0);


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
		if (trajectory_in.pathPoint[tt].t - trajectory_in.pathPoint[tt - 1].t - dmin > 0.00001) {
            //printf("Warning: The input path trajectory is not time equally distributed!  %d \n", tt);
		}
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
		ratio_0 = (trajectory.pathPoint[xy].t - trajectory_in.pathPoint[index_start_0].t) / Tsample_in;

		trajectory.pathPoint[xy].x = (trajectory_in.pathPoint[index_start_0].x + ratio_0*(trajectory_in.pathPoint[index_end_0].x-trajectory_in.pathPoint[index_start_0].x));
		trajectory.pathPoint[xy].y = (trajectory_in.pathPoint[index_start_0].y + ratio_0*(trajectory_in.pathPoint[index_end_0].y - trajectory_in.pathPoint[index_start_0].y));
		trajectory.pathPoint[xy].theta = trajectory_in.pathPoint[index_start_0].theta + ratio_0*(trajectory_in.pathPoint[index_end_0].theta - trajectory_in.pathPoint[index_start_0].theta);
		trajectory.pathPoint[xy].v = trajectory_in.pathPoint[index_start_0].v + ratio_0*(trajectory_in.pathPoint[index_end_0].v - trajectory_in.pathPoint[index_start_0].v);
		trajectory.pathPoint[xy].a = trajectory_in.pathPoint[index_start_0].a + ratio_0*(trajectory_in.pathPoint[index_end_0].a - trajectory_in.pathPoint[index_start_0].a);
		trajectory.pathPoint[xy].kappa = (trajectory_in.pathPoint[index_start_0].kappa + ratio_0*(trajectory_in.pathPoint[index_end_0].kappa - trajectory_in.pathPoint[index_start_0].kappa));
//		fprintf(lcj, "%f,%f,%f,%f,", vehiclePosition.x, vehiclePosition.y, vehiclePosition.theta, vehiclePosition.kappa);
//		fprintf(lcj, "%f,%f,%f,%f,%f,%f\n", trajectory.pathPoint[xy].x, trajectory.pathPoint[xy].y, trajectory.pathPoint[xy].theta, trajectory.pathPoint[xy].v, trajectory.pathPoint[xy].a,trajectory.pathPoint[xy].kappa);
//		fflush(lcj);

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
    printf("Nearest Point No:  %d , %f,  %f, %f, %f \n", nearestPointNr, (trajectory.pathPoint[nearestPointNr].x - x_0), (trajectory.pathPoint[nearestPointNr].y - y_0), (trajectory.pathPoint[nearestPointNr].kappa - kappa_0),(trajectory.pathPoint[nearestPointNr].v - v_0));
    printf("Nearest Point No1:  %d , %f,  %f, %f, %f \n", nearestPointNr, (trajectory.pathPoint[nearestPointNr+1].x - x_0), (trajectory.pathPoint[nearestPointNr+1].y - y_0), (trajectory.pathPoint[nearestPointNr+1].kappa - kappa_0),(trajectory.pathPoint[nearestPointNr+1].v - v_0));

	if (nearestPointNr == 0) {
		index_start = nearestPointNr;
	}
	else {
		index_start = nearestPointNr - 1;
	}
		
	if (nearestPointNr == trajectory.pointsNum) {
		index_end = nearestPointNr;
	}
	else {
		index_end = nearestPointNr + 1;
	}

	float x_start = trajectory.pathPoint[index_start].x;
	float y_start = trajectory.pathPoint[index_start].y;
	float x_end = trajectory.pathPoint[index_end].x;
	float y_end = trajectory.pathPoint[index_end].y;
	float startEndDistance = (y_end - y_start)*(y_end - y_start)+ (x_end - x_start)*(x_end - x_start) ;

	float ratio = ((y_end - y_start)*(vehiclePosition.y - y_start) + (x_end - x_start)*(vehiclePosition.x - x_start)) / startEndDistance;

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
	

	//Initialize Xref Matrix
	MatCT XXref;
	MatCreate(&XXref, 4, trajectory.pointsNum);
	MatCreate(&mpcMatrices.Xref, trajectory.pointsNum, 4);

	float* val;
	val = (float*)MemoryPool_Alloc(mp,4 * trajectory.pointsNum * sizeof(float));

	for (i= 0; i < trajectory.pointsNum; i++) {
		val[i] = xpath[i];
		val[trajectory.pointsNum + i] = ypath[i];
		val[2 * trajectory.pointsNum + i] = thetapath[i];
		val[3 * trajectory.pointsNum + i] = vpath[i];
	}

	MatTrans(MatSetVal(&XXref, val), &mpcMatrices.Xref);
	MatDelete(&XXref);
	MemoryPool_Free(mp,val);
	MemoryPool_Free(mp,trajectory.pathPoint);
	val = NULL;
	trajectory.pathPoint = NULL;

	if (mpcMatrices.Xref.row == 1)
	{
		printf(" Error: The input trajectory or initial point cannot be recognized \n The initial point might be too far away from the path trajectory. \n\n");
	}

	//Initialize Uref Matrix
	MatCT UUref;
	MatCreate(&UUref, 2, trajectory.pointsNum);
	MatCreate(&mpcMatrices.Uref, trajectory.pointsNum, 2);

	float* vall;
	vall = (float*)MemoryPool_Alloc(mp,2 * trajectory.pointsNum * sizeof(float));

	for (i = 0; i < trajectory.pointsNum; i++) {
		vall[i] = apath[i];
		vall[trajectory.pointsNum + i] = atan(kappapath[i] * vehicleShaftDistance);
	}

	MatTrans(MatSetVal(&UUref, vall), &mpcMatrices.Uref);

	MatDelete(&UUref);
	MemoryPool_Free(mp,vall);

	MemoryPool_Free(mp,xpath);
	MemoryPool_Free(mp,ypath);
	MemoryPool_Free(mp,thetapath);
	MemoryPool_Free(mp,vpath);
	MemoryPool_Free(mp,apath);
	MemoryPool_Free(mp,kappapath);

	vall = NULL;
	xpath = NULL;
	ypath = NULL;
	thetapath = NULL;
	vpath = NULL;
	apath = NULL;
	kappapath = NULL;

	//Initialize Xreal Ureal Xerror Uerror
	float X0[4];
	X0[0] = x_0;
	X0[1] = y_0;
	X0[2] = theta_0;
	X0[3] = v_0;

	float U0[2];
	U0[0] = a_0;
	U0[1] = atan(kappa_0 * vehicleShaftDistance);

	//printf("\n current angle: %f\n", U0[1]*15*180/3.141592653);

	MatZeros(MatCreate(&mpcMatrices.Xreal, mpcMatrices.Xref.row, mpcMatrices.Xref.col));
	MatZeros(MatCreate(&mpcMatrices.Ureal, mpcMatrices.Uref.row, mpcMatrices.Uref.col));
	MatZeros(MatCreate(&mpcMatrices.Xerror, mpcMatrices.Xref.row, mpcMatrices.Xref.col));
	MatZeros(MatCreate(&mpcMatrices.Uerror, mpcMatrices.Uref.row, mpcMatrices.Uref.col));

	int col1;
	for (col1 = 0; col1 < mpcMatrices.Xreal.col; col1++)
	{
		mpcMatrices.Xreal.element[0][col1] = X0[col1];
	}

	int col2;
	for (col2 = 0; col2 < mpcMatrices.Ureal.col; col2++)
	{
		mpcMatrices.Ureal.element[0][col2] = U0[col2];
	}
	
	int col3;
	for (col3 = 0; col3 < mpcMatrices.Xerror.col; col3++)
	{
		mpcMatrices.Xerror.element[0][col3] = mpcMatrices.Xreal.element[0][col3] - mpcMatrices.Xref.element[0][col3];
	}

	int col4;
	for (col4 = 0; col4 < mpcMatrices.Uerror.col; col4++)
	{
		mpcMatrices.Uerror.element[0][col4] = mpcMatrices.Ureal.element[0][col4] - mpcMatrices.Uref.element[0][col4];
	}
	
	printf("delta data:  %f, %f, %f, %f \n", mpcMatrices.Xerror.element[0][0], mpcMatrices.Xerror.element[0][1], mpcMatrices.Xerror.element[0][2] * 180 / 3.1415926, mpcMatrices.Xerror.element[0][3]);
	printf("delta angl:  %f, %f \n", mpcMatrices.Xreal.element[0][2] * 180 / 3.1415926, mpcMatrices.Xref.element[0][2] * 180 / 3.1415926);


	//Calculate Qcell Rcell (weighting matrices)
	MatZeros(MatCreate(&mpcMatrices.Qcell, mpcMatrices.Xref.col*Tpredict, mpcMatrices.Xref.col*Tpredict));
	int iii, jjj;
	for (iii = 0; iii < mpcMatrices.Qcell.row; iii++) {
		if (iii % 4 == 0)
		{
			mpcMatrices.Qcell.element[iii][iii] = 1.0f;
		}
		else if (iii % 4 == 1)
		{
			mpcMatrices.Qcell.element[iii][iii] = 2.0f;
		}
		else if (iii % 4 == 2)
		{
			mpcMatrices.Qcell.element[iii][iii] = 3.0f;
		}
		else
		{
			mpcMatrices.Qcell.element[iii][iii] = 2.0f;
		}
	}

	MatZeros(MatCreate(&mpcMatrices.Rcell, mpcMatrices.Uref.col*Tcontrol, mpcMatrices.Uref.col*Tcontrol));
	for (jjj = 0; jjj <mpcMatrices.Rcell.row; jjj++) {
		mpcMatrices.Rcell.element[jjj][jjj] = 1.0f;
	}


	//  Calculate Acell
	//  Format of Acell
	//  [         A(0)
	//	        A(1)*A(0)
	//	             ---
	//	       A(N)***A(0)   ]

	MatZeros(MatCreate(&mpcMatrices.Acell, mpcMatrices.Xref.col*Tpredict, mpcMatrices.Xref.col));

	CalculateAcell(0, Tpredict, &mpcMatrices, &mpcMatrices.Acell);

	// Calculate Bcell
	//    Format of Bcell: N=Tpredict   Nc=Tcontrol  
	//   [     B(0)                                        0                                ---             0
	//         A(1)*B(0)                              B(1)                              ---             0
	//			 ---                                       ---                                ---            ---
	//	        A(N-1)***A(1)*B(0)              A(N-1)***A(2)*B(1)       ---           A(N-1)***A(Nc)*B(Nc)]
 
	MatZeros(MatCreate(&mpcMatrices.Bcell, mpcMatrices.Xref.col*Tpredict, mpcMatrices.Uref.col*Tcontrol));
	int x, y, p, q;
	for (x = 0; x < Tpredict; x++) {
		for (y = 0; y < Tcontrol; y++) {
			if (x >= y) {
				MatCT AAAAcell;
				if (x == y) {
					MatEye(MatCreate(&AAAAcell, mpcMatrices.Xref.col, mpcMatrices.Xref.col));
				}
				else {
					MatZeros(MatCreate(&AAAAcell, mpcMatrices.Xref.col, mpcMatrices.Xref.col));
					CalculateAcell(y+1, x+1 , &mpcMatrices, &AAAAcell);
				}

				MatCT BBcell;
				MatZeros(MatCreate(&BBcell, mpcMatrices.Xref.col, mpcMatrices.Uref.col));
				if(y < mpcMatrices.Xref.row)
					BBcell.element[2][1] = mpcMatrices.Xref.element[y < mpcMatrices.Xref.row ? y : mpcMatrices.Xref.row][3] * Tsample / (cos(mpcMatrices.Uref.element[y][1])*cos(mpcMatrices.Uref.element[y][1]));
				BBcell.element[3][0] = Tsample;

				MatCT ABcell;
				MatZeros(MatCreate(&ABcell, mpcMatrices.Xref.col, mpcMatrices.Uref.col));
				MatMul(&AAAAcell, &BBcell, &ABcell);

				for (p = 0; p < mpcMatrices.Xref.col; p++)
				{
					for (q = 0; q < mpcMatrices.Uref.col; q++)
					{
						mpcMatrices.Bcell.element[p + x*mpcMatrices.Xref.col][q + y*mpcMatrices.Uref.col] = ABcell.element[p][q];
					}
				}
				MatDelete(&AAAAcell);
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
	MatCreate(&Btran, mpcMatrices.Uref.col*Tcontrol, mpcMatrices.Xref.col*Tpredict);
	MatCreate(&BtranQ, mpcMatrices.Uref.col*Tcontrol, mpcMatrices.Xref.col*Tpredict);
	MatCreate(&BtranQB, mpcMatrices.Uref.col*Tcontrol, mpcMatrices.Uref.col*Tcontrol);
	MatCreate(&BQBplusR, mpcMatrices.Uref.col*Tcontrol, mpcMatrices.Uref.col*Tcontrol);

	MatCreate(&mpcMatrices.Hcell, mpcMatrices.Uref.col*Tcontrol, mpcMatrices.Uref.col*Tcontrol);

	MatTrans(&mpcMatrices.Bcell, &Btran);
	MatMul(&Btran, &mpcMatrices.Qcell, &BtranQ);
	MatMul(&BtranQ, &mpcMatrices.Bcell, &BtranQB);
	MatAdd(&BtranQB, &mpcMatrices.Rcell, &BQBplusR);

	int ij, ji;
	for (ij = 0; ij < BQBplusR.row; ij++)
	{
		for (ji = 0; ji < BQBplusR.col; ji++)
		{
			mpcMatrices.Hcell.element[ij][ji] = BQBplusR.element[ij][ji] * 2;
		}
	}


	/*Calculate fcell*/

	MatCT BtranQA;
	MatCT XerrorTran;
	MatCT BQAXerrorTran;

	MatCreate(&BtranQA, mpcMatrices.Uref.col*Tcontrol, mpcMatrices.Xref.col);
	MatCreate(&XerrorTran, mpcMatrices.Xref.col, 1);
	MatCreate(&BQAXerrorTran, mpcMatrices.Uref.col*Tcontrol, 1);

	MatCreate(&mpcMatrices.Fcell, mpcMatrices.Uref.col*Tcontrol, 1);

	int ss;
	for (ss = 0; ss < XerrorTran.row; ss++)
	{
		XerrorTran.element[ss][0] = mpcMatrices.Xerror.element[0][ss];
	}

	MatMul(&BtranQ, &mpcMatrices.Acell, &BtranQA);

	MatMul(&BtranQA, &XerrorTran, &BQAXerrorTran);

	int pp, qq;
	for (pp = 0; pp<BQAXerrorTran.row; pp++)
	{
		for (qq = 0; qq < BQAXerrorTran.col; qq++)
		{
			mpcMatrices.Fcell.element[pp][qq] = BQAXerrorTran.element[pp][qq] * 2;
		}
	}


	MatDelete(&Btran);
	MatDelete(&BtranQ);
	MatDelete(&BtranQB);
	MatDelete(&BQBplusR);
	MatDelete(&BtranQA);
	MatDelete(&XerrorTran);
	MatDelete(&BQAXerrorTran);

	return mpcMatrices;

}

void CalculateAcell(int start, int end, MpcMatrices *mpcMatrices, MatCT *intermediate)
{
	int ii, jj, kk;
	int xx, yy;
	MatCT AA;
	MatCT AAA;
	MatEye(MatCreate(&AA, mpcMatrices->Xref.col, mpcMatrices->Xref.col));
	MatEye(MatCreate(&AAA, mpcMatrices->Xref.col, mpcMatrices->Xref.col));
	
	if (end <= start)
	{
		printf(" Error: The input of CalculateAcell function is wrong");
	}
	for (kk = start; kk < end; kk++) {
		for (ii = 0; ii < AA.row; ii++) {
			for (jj = 0; jj < AA.col; jj++) {

				if ((jj == 0) && (ii % 4 == 0)) {
					AA.element[ii][jj] = 1.0f;
				}
				else if (jj == 1 && ii % 4 == 1) {
					AA.element[ii][jj] = 1.0f;
				}
				else if (jj == 2 && ii % 4 == 2) {
					AA.element[ii][jj] = 1.0f;
				}
				else if (jj == 3 && ii % 4 == 3) {
					AA.element[ii][jj] = 1.0f;
				}
				if (jj == 2 && ii % 4 == 0 && kk < mpcMatrices->Xref.row) {
					AA.element[ii][jj] = (-mpcMatrices->Xref.element[kk][3] * sin(mpcMatrices->Xref.element[kk][2])*Tsample);
				}
				if (jj == 2 && ii % 4 == 1 && kk < mpcMatrices->Xref.row) {
					AA.element[ii][jj] = (mpcMatrices->Xref.element[kk][3] * cos(mpcMatrices->Xref.element[kk][2])*Tsample);
				}
				if (jj == 3 && ii % 4 == 2 && kk < mpcMatrices->Uref.row) {
					AA.element[ii][jj] = tan(mpcMatrices->Uref.element[kk][1]) * Tsample / vehicleShaftDistance;
				}
				if (jj == 3 && ii % 4 == 0 && kk < mpcMatrices->Xref.row) {
					AA.element[ii][jj] = cos(mpcMatrices->Xref.element[kk][2])* Tsample;
				}
				if (jj == 3 && ii % 4 == 1 && kk < mpcMatrices->Xref.row) {
					AA.element[ii][jj] = sin(mpcMatrices->Xref.element[kk][2])* Tsample;
				}
			}
		}
		
		if (kk == 0) {
			MatCopy(&AA, &AAA);
		}
		else {
			MatMul(&AA, &AAA, &AAA);
		}

		if (start == 0) {
			for (xx = 0; xx < AA.row; xx++) {
				for (yy = 0; yy < AA.col; yy++) {
					intermediate->element[xx + 4 * kk][yy] = AAA.element[xx][yy];
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

	MatDelete(&AAA);
	MatDelete(&AA);
}



MpcOutput QPsolve(MpcMatrices mpcMatrices)
{
	//First transfer the normal Hcell matrices into CSC format(Compressed sparse columns format ) 
	//根据OSQP二次规划求解器的输入要求，首先将输入的Hcell矩阵转换为CSC稀疏矩阵格式

	c_float *HcellCSC_value; //CSC format: None zero values 
	c_int *HcellCSC_i;  
	
	
	//CSC format: Number of None zero elements in each columns
	c_int *HcellCSC_p;  //CSC format: The row number for None zero elements

	int i, j;
	int tt = 0;
	int counter = 0;
	for (i = 0; i < mpcMatrices.Hcell.row; i++)
	{
		for (j = 0; j < mpcMatrices.Hcell.col; j++)
		{
			if (mpcMatrices.Hcell.element[i][j] != 0)
			{
				counter++;
			}
		}
	}
	HcellCSC_value = (c_float*)MemoryPool_Alloc(mp,counter*sizeof(c_float));
	HcellCSC_i = (c_int*)MemoryPool_Alloc(mp,counter*sizeof(c_float));
	HcellCSC_p = (c_int*)MemoryPool_Alloc(mp,(mpcMatrices.Hcell.col + 1)*sizeof(c_float));

	for (j = 0; j < mpcMatrices.Hcell.col; j++)
	{
		HcellCSC_p[0] = 0;
		for (i = 0; i < mpcMatrices.Hcell.row; i++)
		{
			if (mpcMatrices.Hcell.element[i][j] != 0)
			{
				HcellCSC_value[tt] = mpcMatrices.Hcell.element[i][j];
				//printf(" %5.2f ", HcellCSC_value[tt]);
				HcellCSC_i[tt] = i;
				tt++;
			}
		}
		HcellCSC_p[j + 1] = tt;
	}
	//int x, y;
	//printf(" \n ");
	//printf(" \n ");
	//for (x = 0; x < counter; x++)
	//{
	//	printf("%5d ", HcellCSC_i[x]);
	//}
	//printf(" \n ");
	//printf(" \n ");
	//for (y = 0; y < (mpcMatrices.Hcell.col + 1); y++)
	//{
	//	printf("%5d ", HcellCSC_p[y]);
	//}
	//printf(" \n ");


	//solve the QP problems
	//求解二次规划问题
	//Reference: osqp.org/docs/ 
	c_int   P_nnz = -1;

	c_float *q = (c_float*)MemoryPool_Alloc(mp,mpcMatrices.Fcell.row*sizeof(c_float));
	
	int ii;
	for (ii = 0; ii < mpcMatrices.Fcell.row; ii++)
	{
		q[ii] = mpcMatrices.Fcell.element[ii][0];
	}

	c_float *A_x = (c_float*)MemoryPool_Alloc(mp,mpcMatrices.Uref.col*Tcontrol*sizeof(c_float));  // fcell 为列向量，直接转化为CSC格式
	c_int *A_i = (c_int*)MemoryPool_Alloc(mp,mpcMatrices.Uref.col*Tcontrol*sizeof(c_int));
	c_int *A_p = (c_int*)MemoryPool_Alloc(mp,(mpcMatrices.Uref.col*Tcontrol + 1)*sizeof(c_int));
	int a;
	for (a = 0; a < mpcMatrices.Uref.col*Tcontrol; a++)
	{
		A_x[a] = 1;
		A_i[a] = a;
		A_p[a] = a;
	}
	A_p[a] = a;
	c_int   A_nnz = -1;

	c_float *l = (c_float*)MemoryPool_Alloc(mp,mpcMatrices.Uref.col*Tcontrol*sizeof(c_float));
	c_float *u = (c_float*)MemoryPool_Alloc(mp,mpcMatrices.Uref.col*Tcontrol*sizeof(c_float));
	int lu;
	for (lu = 0; lu < mpcMatrices.Uref.col*Tcontrol; lu++)
	{
		l[lu] = lowerBoundary[lu%mpcMatrices.Uref.col];
		u[lu] = upperBoundary[lu % mpcMatrices.Uref.col];
	}

	c_int n = mpcMatrices.Hcell.col;
	c_int m = mpcMatrices.Uref.col*Tcontrol;

	// Problem settings
	OSQPSettings *settings = (OSQPSettings *)MemoryPool_Alloc(mp,sizeof(OSQPSettings));

	// Structures
	OSQPWorkspace *work; // Workspace
	OSQPData *data;      // OSQPData

						 // Populate data
	data = (OSQPData *)MemoryPool_Alloc(mp,sizeof(OSQPData));
	data->n = n;
	data->m = m;
	data->P = csc_matrix(data->n, data->n, P_nnz, HcellCSC_value, HcellCSC_i, HcellCSC_p);
	data->q = q;
	data->A = csc_matrix(data->m, data->m, A_nnz, A_x, A_i, A_p);
	data->l = l;
	data->u = u;

	// Define Solver settings as default                                                                                                                                    
	osqp_set_default_settings(settings);

	// Setup workspace
	work = osqp_setup(data, settings);

	MpcOutput mpcOutput;

	// Solve Problem
	osqp_solve(work,&mpcOutput.Xcell);

	//char aa[6] = "Xcell";
	//printf("%s ", aa);
	//MatDump(&mpcOutput.Xcell);

	mpcOutput.acceleration = mpcOutput.Xcell.element[0][0]+ mpcMatrices.Uref.element[0][0];

	//if ( mpcMatrices.Xreal.element[0][3] < 1) {
	//	mpcOutput.acceleration = mpcOutput.acceleration + 0.15;
	//}

	//printf(" Current Output Acceleration: %f  [m/s^2] \n\n", mpcOutput.acceleration);

    mpcOutput.frontWheelAngle = mpcOutput.Xcell.element[1][0] + mpcMatrices.Uref.element[0][1];

    mpcOutput.velocity = mpcMatrices.Xref.element[0][3] + mpcOutput.acceleration*Tsample;

    printf("MPC velocity :%f %f\n", mpcOutput.acceleration, mpcOutput.velocity );

	printf("MPC ref steerOutput :%f %f \n", mpcOutput.Xcell.element[1][0]*180*16/3.14159, mpcMatrices.Uref.element[0][1] * 180 * 16 / 3.14159);

	//printf(" Current Output Front Wheel Angle: %f  [rad] \n\n", mpcOutput.frontWheelAngle);

	// Clean workspace
	osqp_cleanup(work);

	//MemoryPool_Free(mp,data->A);
	//MemoryPool_Free(mp,data->P);
	MemoryPool_Free(mp,data);
	MemoryPool_Free(mp,settings);

	MemoryPool_Free(mp,HcellCSC_value);
	MemoryPool_Free(mp,HcellCSC_i);
	MemoryPool_Free(mp,HcellCSC_p);
	MemoryPool_Free(mp,q);
	MemoryPool_Free(mp,A_x);
	MemoryPool_Free(mp,A_i);
	MemoryPool_Free(mp,A_p);
	MemoryPool_Free(mp,l);
	MemoryPool_Free(mp,u);
	data = NULL;
	settings = NULL;
	HcellCSC_value = NULL;
    HcellCSC_i = NULL;
	HcellCSC_p = NULL;
	q = NULL;
	A_x = NULL;
	A_i = NULL;
	A_p = NULL;
	l = NULL;
	u = NULL;

	return mpcOutput;
}


void PrintMpcMatrices(MpcMatrices mpcMatrices)
{
	char a[5] = "Xref";
	printf("%s ", a);
	MatDump(&mpcMatrices.Xref);

	char mm[5] = "Uref";
	printf("%s ", mm);
	MatDump(&mpcMatrices.Uref);

	//char ff[6] = "Xreal";
	//printf("%s ", ff);
	//MatDump(&mpcMatrices.Xreal);

	//char b[6] = "Ureal";
	//printf("%s ", b);
	//MatDump(&mpcMatrices.Ureal);

	//char c[7] = "Xerror";
	//printf("%s ", c);
	//MatDump(&mpcMatrices.Xerror);

	//char d[7] = "Uerror";
	//printf("%s ", d);
	//MatDump(&mpcMatrices.Uerror);

	//char ee[6] = "Qcell";
	//printf("%s ", ee);
	//MatDump(&mpcMatrices.Qcell);

	//char eee[6] = "Rcell";
	//printf("%s ", eee);
	//MatDump(&mpcMatrices.Rcell);

	char e[6] = "Acell";
	printf("%s ", e);
	MatDump(&mpcMatrices.Acell);

	char f[6] = "Bcell";
	printf("%s ", f);
	MatDump(&mpcMatrices.Bcell);

	char g[6] = "Hcell";
	printf("%s ", g);
	MatDump(&mpcMatrices.Hcell);

	char h[6] = "fcell";
	printf("%s ", h);
	MatDump(&mpcMatrices.Fcell);

}

void CleanMpcMatrices(MpcMatrices mpcMatrices)
{
	MatDelete(&mpcMatrices.Xref);
	MatDelete(&mpcMatrices.Uref);
	MatDelete(&mpcMatrices.Xreal);
	MatDelete(&mpcMatrices.Ureal);
	MatDelete(&mpcMatrices.Xerror);
	MatDelete(&mpcMatrices.Uerror);
	MatDelete(&mpcMatrices.Qcell);
	MatDelete(&mpcMatrices.Rcell);
	MatDelete(&mpcMatrices.Acell);
	MatDelete(&mpcMatrices.Bcell);
	MatDelete(&mpcMatrices.Hcell);
	MatDelete(&mpcMatrices.Fcell);

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
	Tcontrol = floor(predictHorizon/4);              //  1<=  Control horizon <= Predict horizon; recommend value: 0.2*Tpredict (at least equals to 2-3);

	Tsample = sampleTime;        /*samping time, this should be same as the updated time of the path point*/
	lowerBoundary[0] = -0.1f;   //lower boundary for velocity_control- velocity_path
	lowerBoundary[1] = -0.14f;  //lower boundary for front/wheel/angel_control- front/wheel/angel_path
	upperBoundary[0] = 0.1f;
	upperBoundary[1] = 0.14f;

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




//Initial vehicle position 车辆初始状态模拟输入
//Trajectory information 路径点信息模拟输入
// 由函数的调用者赋予并清理MpcOutput的内存空间，Output作为输出值存在
// 等后续调试后看看需要再更改些什么东西
void mpc_controller_box(VehiclePosition vehiclePosition, Trajectory trajectory, MpcOutput Output) {

	mp = MemoryPool_Init(max_mem, mem_pool_size, 0); //加入内存池，优化多次malloc造成的计算能力流失 mp定义再 memorypool.h中
													 //Initial Values 初始量模拟输入
    int predictHorizon = 50;
    float vehicleShaftDistance = 1;     /* vehicle shaft distance */
    float samplingTime = 0.02;


	/*********************************************** functions *************************************************************/
	MpcVehicleControlInitial(vehicleShaftDistance, predictHorizon, samplingTime);

	Output = MpcVehicleControlProcess(vehiclePosition, trajectory);

	/************************************************ Output *****************************************************************/

	//销毁内存池
	MemoryPool_Destroy(mp);

}



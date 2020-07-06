/*
 * Light Matrix: C code implementation for basic matrix operation
 *
 * Copyright (C) 2017 Jiachi Zou
 *
 */

 /*******************************************************************************
  æÿ’Û‘ÀÀ„ø‚ £®«ÎŒÀÊ“‚–ﬁ∏ƒ£©
 *******************************************************************************/
 
#ifndef __LIGHT_MATRIX__
#define __LIGHT_MATRIX__

typedef struct  {
	int row, col;
	float **element;
}MatCT;

MatCT* MatCreate(MatCT* mat, int row, int col);
void MatDelete(MatCT* mat);
MatCT* MatSetVal(MatCT* mat, float* val);
void MatDump(const MatCT* mat);

MatCT* MatZeros(MatCT* mat);
MatCT* MatEye(MatCT* mat);

MatCT* MatAdd(MatCT* src1, MatCT* src2, MatCT* dst);
MatCT* MatSub(MatCT* src1, MatCT* src2, MatCT* dst);
MatCT* MatMul(MatCT* src1, MatCT* src2, MatCT* dst);
MatCT* MatTrans(MatCT* src, MatCT* dst);
float MatDet(MatCT* mat);
MatCT* MatAdj(MatCT* src, MatCT* dst);
MatCT* MatInv(MatCT* src, MatCT* dst);

void MatCopy(MatCT* src, MatCT* dst);

#endif

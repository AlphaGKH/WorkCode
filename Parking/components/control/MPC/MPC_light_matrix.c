/*
 * Light Matrix: C code implementation for basic matrix operation
 *
 * Copyright (C) 2017 Jiachi Zou
 */
 
 /*******************************************************************************
 *  矩阵运算库 （请勿随意修改）
 *******************************************************************************/

#include "MPC_light_matrix.h"
#include <stdio.h>
#include <stdlib.h>
#include "memorypool.h" 

//#define MAT_LEGAL_CHECKING
#define VNAME(name)(#name)

#ifndef min
#define min(a, b) ((a) > (b) ? (b) : (a))
#endif
#define equal(a, b)	((a-b)<1e-7 && (a-b)>-(1e-7))

MemoryPool *mp;

/************************************************************************/
/*                          Private Function                            */
/************************************************************************/

void swap(int *a, int *b)
{
	int m;
	m = *a;
	*a = *b;
	*b = m;
}
 
void perm(int list[], int k, int m, int* p, MatCT* mat, float* det) 
{
	int i;

	if(k > m){
		float res = mat->element[0][list[0]];

		for(i = 1; i < mat->row ; i++){
			res *= mat->element[i][list[i]];
		}

		if(*p%2){
			//odd is negative
			*det -= res;
		}else{
			//even is positive
			*det += res;
		}
	}
	else{
		// if the element is 0, we don't need to calculate the value for this permutation
		if(!equal(mat->element[k][list[k]], 0.0f))
			perm(list, k + 1, m, p, mat, det);
		for(i = k+1; i <= m; i++)
		{
			if(equal(mat->element[k][list[i]], 0.0f))
				continue;
			swap(&list[k], &list[i]);
			*p += 1;
			perm(list, k + 1, m, p, mat, det);
			swap(&list[k], &list[i]);
			*p -= 1; 
		}
	}
}

/************************************************************************/
/*                           Public Function                            */
/************************************************************************/

MatCT* MatCreate(MatCT* mat, int row, int col)
{
	int i;

	mat->element = (float**)MemoryPool_Alloc(mp,row * sizeof(float*));
	if(mat->element == NULL){
		printf("mat create fail!\n");
		return NULL;
	}
	for(i = 0 ; i < row ; i++){
		mat->element[i] = (float*)MemoryPool_Alloc(mp,col * sizeof(float));
		if(mat->element[i] == NULL){
			int j;
			printf("mat create fail!\n");
			for (j = 0; j < i; j++) {
				MemoryPool_Free(mp, mat->element[j]);
				mat->element[j] = NULL;
			}
			MemoryPool_Free(mp,mat->element);
			mat->element = NULL;
			return NULL;
		}
	}

	mat->row = row;
	mat->col = col;

	return mat;
}

void MatDelete(MatCT* mat)
{
	int i;

	for (i = 0; i < mat->row; i++) {
		MemoryPool_Free(mp, mat->element[i]);
		mat->element[i] = NULL;
	}		
	MemoryPool_Free(mp,mat->element);
	mat->element = NULL;
}


MatCT* MatSetVal(MatCT* mat, float* val)
{
	int row,col;

	for(row = 0 ; row < mat->row ; row++){
		for(col = 0 ; col < mat->col ; col++){
			mat->element[row][col] = val[col + row * mat->col];
		}
	}

	return mat;
}

void MatDump(const MatCT* mat)
{
	int row,col;

#ifdef MAT_LEGAL_CHECKING
	if(mat == NULL){
		return ;
	}
#endif

	printf("MatCT %dx%d:\n", mat->row, mat->col);
	for(row = 0 ; row < mat->row ; row++){
		for(col = 0 ; col < mat->col ; col++){
			printf("%.4f\t", mat->element[row][col]);
		}
		printf("\n");
	}
	printf("\n");
}

MatCT* MatZeros(MatCT* mat)
{
	int row,col;

	for(row = 0 ; row < mat->row ; row++){
		for(col = 0 ; col < mat->col ; col++){
			mat->element[row][col] = 0.0f;
		}
	}

	return mat;
}

MatCT* MatEye(MatCT* mat)
{
	int i;
	
	MatZeros(mat);
	for(i = 0 ; i < min(mat->row, mat->col) ; i++){
		mat->element[i][i] = 1.0f;
	}

	return mat;
}

/* dst = src1 + src2 */
MatCT* MatAdd(MatCT* src1, MatCT* src2, MatCT* dst)
{
	int row, col;

#ifdef MAT_LEGAL_CHECKING
	if( !(src1->row == src2->row && src2->row == dst->row && src1->col == src2->col && src2->col == dst->col) ){
		printf("err check, unmatch matrix for MatAdd\n");
		MatDump(src1);
		MatDump(src2);
		MatDump(dst);
		return NULL;
	}
#endif

	for(row = 0 ; row < src1->row ; row++){
		for(col = 0 ; col < src1->col ; col++){
			dst->element[row][col] = src1->element[row][col] + src2->element[row][col];
		}
	}

	return dst;
}

/* dst = src1 - src2 */
MatCT* MatSub(MatCT* src1, MatCT* src2, MatCT* dst)
{
	int row, col;

#ifdef MAT_LEGAL_CHECKING
	if( !(src1->row == src2->row && src2->row == dst->row && src1->col == src2->col && src2->col == dst->col) ){
		printf("err check, unmatch matrix for MatSub\n");
		MatDump(src1);
		MatDump(src2);
		MatDump(dst);
		return NULL;
	}
#endif

	for(row = 0 ; row < src1->row ; row++){
		for(col = 0 ; col < src1->col ; col++){
			dst->element[row][col] = src1->element[row][col] - src2->element[row][col];
		}
	}

	return dst;
}

/* dst = src1 * src2 */ 
// 不要用这个函数进行迭乘！！迭乘时原先那个矩阵在计算过程中本身就会被改变，这样算出来的结果是不多的，把计算结果储存到一个中间矩阵。
MatCT* MatMul(MatCT* src1, MatCT* src2, MatCT* dst)
{
	int row, col;
	int i;
	float temp;

#ifdef MAT_LEGAL_CHECKING
	if( src1->col != src2->row || src1->row != dst->row || src2->col != dst->col ){
		printf("err check, unmatch matrix for MatMul\n");
		MatDump(src1);
		MatDump(src2);
		MatDump(dst);
		return NULL;
	}
#endif

	for(row = 0 ; row < dst->row ; row++){
		for(col = 0 ; col < dst->col ; col++){
			temp = 0.0f;
			for(i = 0 ; i < src1->col ; i++){
				temp += src1->element[row][i] * src2->element[i][col];
			}
			dst->element[row][col] = temp;
		}
	}

	return dst;
}


/* dst = src1 ^times */
MatCT* MatPower(int times, MatCT* src1, MatCT* dst)
{
	int i;

#ifdef MAT_LEGAL_CHECKING
	if ( src1->row != dst->row || src1->col != dst->col) {
		printf("err check, unmatch matrix for MatPower\n");
		MatDump(src1);
		MatDump(dst);
		return NULL;
	}
#endif
	MatEye(dst);
	MatCT temp;
	MatCreate(&temp, dst->row, dst->col);

	for (i = 0; i < times; i++) {
		MatMul(src1, dst, &temp);
		MatCopy(&temp,dst);
	}

	MatDelete(&temp);
	return dst;
}


/* dst = src' */
MatCT* MatTrans(MatCT* src, MatCT* dst)
{
	int row, col;

#ifdef MAT_LEGAL_CHECKING
	if( src->row != dst->col || src->col != dst->row ){
		printf("err check, unmatch matrix for MatTranspose\n");
		MatDump(src);
		MatDump(dst);
		return NULL;
	}
#endif

	for(row = 0 ; row < dst->row ; row++){
		for(col = 0 ; col < dst->col ; col++){
			dst->element[row][col] = src->element[col][row];
		}
	}

	return dst;
}

// return det(mat)
float MatDet(MatCT* mat)
{
	float det = 0.0f;
	int plarity = 0;
	int *list;
	int i;

#ifdef MAT_LEGAL_CHECKING
	if( mat->row != mat->col){
		printf("err check, not a square matrix for MatDetermine\n");
		MatDump(mat);
		return 0.0f;
	}
#endif

	list = (int*)MemoryPool_Alloc(mp,sizeof(int)*mat->col);
	if(list == NULL){
		printf("malloc list fail\n");
		return 0.0f;
	}
	for(i = 0 ; i < mat->col ; i++)
		list[i] = i;

	perm(list, 0, mat->row-1, &plarity, mat, &det);
	MemoryPool_Free(mp,list);
	list = NULL;

	return det;
}

// dst = adj(src)
MatCT* MatAdj(MatCT* src, MatCT* dst)
{
	MatCT smat;
	int row, col;
	int i,j,r,c;
	float det;

#ifdef MAT_LEGAL_CHECKING
	if( src->row != src->col || src->row != dst->row || src->col != dst->col){
		printf("err check, not a square matrix for MatAdj\n");
		MatDump(src);
		MatDump(dst);
		return NULL;
	}
#endif

	MatCreate(&smat, src->row-1, src->col-1);

	for(row = 0 ; row < src->row ; row++){
		for(col = 0 ; col < src->col ; col++){
			r = 0;
			for(i = 0 ; i < src->row ; i++){
				if(i == row)
					continue;
				c = 0;
				for(j = 0; j < src->col ; j++){
					if(j == col)
						continue;
					smat.element[r][c] = src->element[i][j];
					c++;
				}
				r++;
			}
			det = MatDet(&smat);
			if((row+col)%2)
				det = -det;
			dst->element[col][row] = det;
		}
	}

	MatDelete(&smat);

	return dst;
}

// dst = src^(-1)
MatCT* MatInv(MatCT* src, MatCT* dst)
{
	MatCT adj_mat;
	float det;
	int row, col;

#ifdef MAT_LEGAL_CHECKING
	if( src->row != src->col || src->row != dst->row || src->col != dst->col){
		printf("err check, not a square matrix for MatInv\n");
		MatDump(src);
		MatDump(dst);
		return NULL;
	}
#endif
	MatCreate(&adj_mat, src->row, src->col);
	MatAdj(src, &adj_mat);
	det = MatDet(src);

	if(equal(det, 0.0f)){
		printf("err, determinate is 0 for MatInv\n");
		return NULL;
	}
	
	for(row = 0 ; row < src->row ; row++){
		for(col = 0 ; col < src->col ; col++)
			dst->element[row][col] = adj_mat.element[row][col]/det;
	}
	
	MatDelete(&adj_mat);

	return dst;
}

void MatCopy(MatCT* src, MatCT* dst)
{
	int row, col;
	
#ifdef MAT_LEGAL_CHECKING
	if( src->row != dst->row || src->col != dst->col){
		printf("err check, unmathed matrix for MatCopy\n");
		MatDump(src);
		MatDump(dst);
		return ;
	}
#endif
	
	for(row = 0 ; row < src->row ; row++){
		for(col = 0 ; col < src->col ; col++)
			dst->element[row][col] = src->element[row][col];
	}
}

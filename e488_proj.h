#include <stdio.h>
#include <conio.h>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include "stdafx.h"
#include "ensc-488.h"

#define L10 10
#define L30 30
#define L70 70
#define L80 80
#define L130 130
#define L135 135
#define L140 140
#define L142 142
#define L195 195
#define L405 405
#define L410 410
#define PI 3.14159265
#define N 4
#define THETA_CONS_150 150
#define THETA_CONS_100 100
#define D3LOWER_200 -200
#define D3UPPER_100 -100
#define A210 210
#define A150 150

using namespace std;

typedef double TFORM[4][4];
typedef double RFORM[3][3];
typedef double POS[3];

JOINT T{0, 0, L135, 0};
JOINT B{0, 0, L405, 0};
TFORM wrelb, brels, trelw; 
JOINT spt;

void main(void);

// menu stuff
void FwdKinDeg(JOINT &joint_vals, JOINT &spt);
void FwdKinRad(JOINT &joint_vals, JOINT &spt);
void InvKin(JOINT &spt);
void SimpleMove(void);
void check_joints(JOINT &joint_vals, bool &valid);

// functions for class
// Basic Matrix Transformation Procedures
void UTOI(JOINT &pos, TFORM &mat);
void UTOI_FLIP(JOINT &pos, TFORM &mat);
void ITOU(TFORM &mat, JOINT &pos);
void TMULT(TFORM &brela, TFORM &crelb, TFORM &crela);
void TINVERT(TFORM &tmat);

// Forward Kinematics
void KIN(JOINT &joint_vals, TFORM &wrelb);
void WHERE(JOINT &joint_vals, JOINT &spt);

// Inverse Kinematics
void INVKIN(TFORM &wrelb, JOINT &curr_pos, JOINT &near, JOINT &far, bool &sol);
void SOLVE(JOINT &tar_pos, JOINT &curr_pos, JOINT &near, JOINT &far, bool &sol);

// Helper functions
int arr_size(JOINT &arr);
void print(TFORM &mat);
void print(JOINT &arr);
void print(POS &arr);
void get_r(TFORM &tmat, RFORM &rmat);
void get_pos(TFORM &tmat, POS &pos);
void pop_mat(TFORM &vals, TFORM &mat);
void pop_mat(RFORM &vals, RFORM &mat);
void pop_arr(JOINT &vals, JOINT &arr);
void pop_arr(POS &vals, POS &arr);
void rmult(RFORM &mat1, RFORM &mat2, RFORM &res);
void rmult(RFORM &mat, POS &pos, POS &res);
void pmult(POS &pos, double val, POS &res);
void padd(POS &pos1, POS &pos2, POS &res);
void tconst(RFORM &rmat, POS &pos, TFORM &tmat);
void transpose_mat(RFORM &rmat, RFORM &imat);
bool no_sol(bool p_invalid, bool n_invalid);
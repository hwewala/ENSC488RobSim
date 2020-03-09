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
#define THETA1_CONS 150
#define THETA2_CONS 100
#define THETA4_CONS 160
#define D3LOWER_200 -200
#define D3UPPER_100 -100
#define A210 210
#define A150 150
#define FK 666

using namespace std;

typedef double TFORM[4][4];
typedef double RFORM[3][3];
typedef double CFORM[3][4];
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
void ToggleGripper(bool& status);
void TrajPlan(void);

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
void INVKIN(TFORM &wrelb, JOINT &near, JOINT &far, bool &p_val, bool &n_val);
void SOLVE(JOINT &tar_pos, JOINT &curr_joint, JOINT &near, JOINT &far, bool &p_val, bool &n_val);

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

// Demo 2
void CUBCOEF(double theta0, double thetaf, double vel0, double velf, double tf, JOINT &coeff);
void PATHGEN(double t, double vel, TFORM& A, TFORM& B, TFORM& C, TFORM& G);
void get_jv(int idx, JOINT& a_joint, JOINT& b_joint, JOINT& c_joint, JOINT& g_joint, JOINT& joint);
void compute_coeff(JOINT&j, double t, double vel, JOINT&ab, JOINT&bc, JOINT&cg);
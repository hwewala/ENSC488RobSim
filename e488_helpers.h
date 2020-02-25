#pragma once
#ifndef ENSC488
#define ENSC488
#include <stdio.h>
#include <vector> 
#include <math.h>

#include "vec_help.h"
#include "ensc-488.h"

using namespace std;
#endif

#define L10 10
#define L30 30
#define L70 70
#define L80 80
#define L130 130
#define L140 140
#define L142 142
#define L195 195
#define L405 405
#define L410 410
#define PI 3.14159265

#define S {0, 0, 0, 0}
#define B {0, 0, L405, 0}
#define T {140, 0, 0, 0}

// helpers //

// converts deg to rad
double torad(double deg);
double cosd(double deg);
double sind(double deg);
double acosd(double val);
double asind(double val);

///////////////////////////////////////////////
/* Part 1: Basic Matrix Computation Routines */
///////////////////////////////////////////////

//User form TO Internal form
vector<JOINT> UTOI(JOINT pos);

// Internal form TO User form
// JOINT ITOU(vector<JOINT> mat);

// // Transform MULTiplication
// vector<JOINT> TMULT(vector<JOINT> brela, vector<JOINT> crelb);
// JOINT TMULT(JOINT P, vector<JOINT> R);

// // Invert Matrix
// vector<JOINT> TINVERT(vector<JOINT> mat);

// ///////////////////////////////////////////
// /* Part 2: Forward and Inverse Kinematics*/
// ///////////////////////////////////////////

// // Forward Kinematics
// vector<JOINT> KIN(JOINT joint_vals);

// JOINT WHERE(JOINT joint_vals, vector<JOINT> brels, vector<JOINT> trelw);

// // Inverse Kinematics
// void INVKIN(vector<JOINT> wrelb, JOINT curr_pos, JOINT &near, JOINT &far, bool &sol);

// void SOLVE(JOINT tar_pos, JOINT curr_pos, JOINT &near, JOINT &far, bool &sol);


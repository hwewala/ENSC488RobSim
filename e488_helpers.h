#include <stdio.h>
#include <vector> 
#include <math.h>

#include "vec_help.h"

using namespace std;

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
float torad(float deg);

///////////////////////////////////////////////
/* Part 1: Basic Matrix Computation Routines */
///////////////////////////////////////////////

//User form TO Internal form
vector<vector<float>> UTOI(vector<float> pos);

// Internal form TO User form
vector<float> ITOU(vector<vector<float>> mat);

// Transform MULTiplication
vector<vector<float>> TMULT(vector<vector<float>> brela, vector<vector<float>> crelb);
vector<float> TMULT(vector<float> P, vector<vector<float>> R);

// Invert Matrix
vector<vector<float>> TINVERT(vector<vector<float>> mat);

///////////////////////////////////////////
/* Part 2: Forward and Inverse Kinematics*/
///////////////////////////////////////////

// Forward Kinematics
vector<vector<float>> KIN(vector<float> joint_vals);

vector<float> WHERE(vector<float> joint_vals, vector<vector<float>> brels, vector<vector<float>> trelw);

// Inverse Kinematics
void INVKIN(vector<vector<float>> wrelb, vector<float> curr_pos, vector<float> &near, vector<float> &far, bool &sol);

void SOLVE(vector<float> tar_pos, vector<float> curr_pos, vector<float> &near, vector<float> &far, bool &sol);


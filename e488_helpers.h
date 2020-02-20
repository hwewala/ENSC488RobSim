#include <stdio.h>
#include <vector> 
#include <math.h>

#include "vec_help.h"

using namespace std;

float L10 = 10;
float L30 = 30;
float L70 = 70;
float L80 = 80;
float L130 = 130;
float L140 = 140;
float L142 = 142;
float L195 = 195;
float L405 = 405;
float L410 = 410;
float PI = 3.14159265;

// helpers //

// converts deg to rad
float torad(float deg);

///////////////////////////////////////////////
/* Part 1: Basic Matrix Computation Routines */
///////////////////////////////////////////////

//User form TO Internal form
vector<vector<float>> UTOI(vector<float> pos);

// Internal form TO User form
vector<float> ITOU(vector<vector<float>> T);

// Transform MULTiplication
vector<vector<float>> TMULT(vector<vector<float>> brela, vector<vector<float>> crelb);

// Invert Matrix
vector<vector<float>> TINVERT(vector<vector<float>> mat);

///////////////////////////////////////////
/* Part 2: Forward and Inverse Kinematics*/
///////////////////////////////////////////

// Forward Kinematics
vector<vector<float>> KIN(vector<float> theta, vector<vector<float>> wrelb);

vector<vector<float>> WHERE(vector<float> theta, vector<vector<float>> trels);

// Inverse Kinematics
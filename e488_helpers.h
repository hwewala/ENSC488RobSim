#include <stdio.h>
#include <vector> 
#include <math.h>

#include "vec_help.h"

using namespace std;

#define PI 3.14159265;
///////////////////////////////////////////////
/* Part 1: Basic Matrix Computation Routines */
///////////////////////////////////////////////

//User form TO Internal form
vector<vector<float>> UTOI(float x, float y);

// Internal form TO User form
vector<float> ITOU(vector<vector<float>> R);

// Transform MULTiplication
vector<vector<float>> TMULT(vector<vector<float>> brela, vector<vector<float>> crelb);

// Invert Matrix
vector<vector<float>> TINVERT(vector<vector<float>> mat);

///////////////////////////////////////////
/* Part 2: Forward and Inverse Kinematics*/
///////////////////////////////////////////

// Forward Kinematics
void KIN(vector<float> theta, vector<vector<float>> wrelb);

void WHERE(vector<float> theta, vector<vector<float>> trels);

// Inverse Kinematics
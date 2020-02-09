#include <stdio.h>
#include <vector> 
#include <math.h>

using namespace std;

#define PI 3.14159265;

// User form TO Internal form
vector<vector<float>> UTOI(float x, float y);

// Internal form TO User form
vector<float> ITOU(vector<vector<float>> R);

// Transform MULTiplication
vector<vector<float>> TMULT(vector<vector<float>> brela, vector<vector<float>> crelb);

// vector sum
float vSum(vector<float> vec);
#include "e488_helpers.h"

///////////////////////////////////////////////
/* Part 1: Basic Matrix Computation Routines */
///////////////////////////////////////////////

// User form TO Internal form
vector<vector<float>> UTOI(float x, float y) {
    /* We want to allow the user to specify a frame as a 3-tuple (x, y, z)
    Assume 3x3 matrix
    Assume all rotations are around Z-axis
    Input: (x, y) coordinate
    Output: R
    R = | cos(theta)    -sin(theta)     x |
        | sin(theta)    cos(theta)      y |
        | 0             0               1 |
    */
    float theta = atan2f(y, x);
    float s_theta = sinf(theta);
    float c_theta = cosf(theta);

    vector<float> r1{c_theta, -s_theta, x};
    vector<float> r2{s_theta, c_theta, y};
    vector<float> r3{0, 0, 1};
    
    vector<vector<float>> R{r1, r2, r3};

    return R;
}

// Internal form TO User form
vector<float> ITOU(vector<vector<float>> R) {
    /*user will specify a 3x3 rotation matrix, and will output (x, y)
    Assume 3x3 matrix
    Assume all rotations are around Z-axis
    Input: R (rotation matrix)
    Output: (x, y, z) coordinate */

    float c_theta = R[0][0];
    float theta = acosf(c_theta);

    float x = cosf(theta);
    float y = sinf(theta);

    vector<float> unit_vec{x, y};

    return unit_vec;
}

// T MULTiplication
vector<vector<float>> TMULT(vector<vector<float>> brela, vector<vector<float>> crelb) {
    /* multiplies two matrices together. Assumes it's NxN matrix 
    Output: crela, "c relative to a" */
    vector<vector<float>> crela;
    int N = size(brela);

    for(int i = 0; i < N; i++) {        
        // iterate through the rows
        vector<float> vec1;
        for(int j = 0; j < N; j++) {
            // iterate through the columns
            vector<float> vec2;
            float sum = 0;
            for(int k = 0; k < N; k++) {
                // do matrix calculations
                vec2.push_back(brela[i][k]*crelb[k][j]);
                sum = sum_vec(vec2);
            }
            vec1.push_back(sum);
        }
        crela.push_back(vec1);
    }
    return crela;
}

// Invert Matrix
vector<vector<float>> TINVERT(vector<vector<float>> mat) {
    /* Performs the inverse of a matrix
    Assume 3x3 Matrix*/
    int N = 3;
    float det = det_mat(mat);
    vector<vector<float>> inv_mat;

    // cycle through all the coordinates in the 3x3 matrix
    for(int i = 0; i < N; i++) {
        vector<float> lst_mats;
        for(int j = 0; j < N; j++) {
            vector<vector<float>> min_mat = minor_mat(mat, i, j);
            float det_min = det_mat2(min_mat);
            // play with the signs
            if(i % 2 != 0 || j % 2 != 0) {
                det_min = -det_min;
            }
            lst_mats.push_back((1/det)*det_min); // multiply by 1/det
        }
        inv_mat.push_back(lst_mats);
    }

    // transpose the Matrix of Cofactors
    transpose_mat(inv_mat);
    
    return inv_mat;
}

///////////////////////////////////////////
/* Part 2: Forward and Inverse Kinematics*/
///////////////////////////////////////////

// Forward Kinematics
void KIN(vector<float> theta, vector<vector<float>> wrelb){
    /*theta: joint angles
      wrelb: the wrist frame WRT the base frame.
        - this is a 2x2 rotation matrix and a 2x1 position vector*/
}

vector<vector<float>> WHERE(vector<float> theta, vector<vector<float>> trelw, vector<vector<float>> brels) {
    /*  Inputs: 
            theta: joint angles
            trelw: Tool frame WRT Wrist frame
            brels: Base frame WRT Station frame
        Output:
            trels: Tool frame WRT Station frame
    */



   vector<vector<float>> trels;
   return trels;
}
    

// Inverse Kinematics

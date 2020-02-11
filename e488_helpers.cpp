#include "e488_helpers.h"

// User form TO Internal form
vector<vector<float>> UTOI(float x, float y) {
    /* We want to allow the user to specify a frame as a 3-tuple (x, y, z)
    Assume 3x3 matrix
    Assume all rotations are around Z-axis
    Input: (x, y) coordinate
    Output: R (rotation matrix)*/
    float theta = atan2f(y, x);
    float s_theta = sinf(theta);
    float c_theta = cosf(theta);

    vector<float> r1{c_theta, -s_theta, 0};
    vector<float> r2{s_theta, c_theta, 0};
    vector<float> r3{0, 0, 1};
    
    vector<vector<float>> R{r1, r2, r3};

    return R;
}

// Internal form TO User form
vector<float> ITOU(vector<vector<float>> R) {
    /*user will specify a 3x3 rotation matrix, and will output (x, y, z)
    Assume 3x3 matrix
    Assume all rotations are around Z-axis
    Input: R (rotation matrix)
    Output: (x, y, z) coordinate */

    // following equations 2.81 and 2.82 in the text

    vector<float> unit_vec{ R[2][1] - R[1][2],
                            R[0][2] - R[2][0],
                            R[1][0] - R[0][1]};
    float theta = acosf((R[0][0] + R[1][1] + R[2][2] - 1)/2);

    for(int i = 0; i < size(unit_vec); i++) {
        unit_vec[i] = (1/(2*sin(theta)))*unit_vec[i];
    }

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
void TINVERT(vector<vector<float>> mat) {
    /* Performs the inverse of a matrix
    Assume 3x3 Matrix*/
    
}
#include "e488_helpers.h"

///////////////
/* CONSTANTS */
///////////////
// lengths are in (mm)
// angles are in (rads)
float A1 = 195;
float A2 = 142;

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
    /*  user will specify a 3x3 rotation matrix, and will output (x, y, theta)
        Assume 3x3 matrix
        Assume all rotations are around Z-axis
    
        Input: R (rotation matrix)
        R = | cos(theta)    -sin(theta)     x |
            | sin(theta)    cos(theta)      y |
            | 0             0               1 |

        Output: (x, y, theta) coordinates 
    */

    // old method: getting x and y using theta. 
    // Assume length is 1
    // float c_theta = R[0][0];
    // float theta = acosf(c_theta);

    // float x = cosf(theta);
    // float y = sinf(theta);

    // vector<float> unit_vec{x, y};

    // new method: getting (x, y, theta) values from R (as defined above)
    float x = R[0][2];
    float y = R[1][2];
    float theta = acosf(R[0][0]);

    vector<float> vec{x, y, theta};

    return vec;
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
vector<vector<float>> KIN(vector<float> theta){
    /*
        Inputs:
            theta: joint angles (theta1, theta2)
        Output:
            wrelb: the wrist frame WRT the base frame.
                - this is a 2x2 rotation matrix and a 2x1 position vector
            wrelb = | cos(phi)  -sin(phi)   x |
                    | sin(phi)  cos(phi)    y |
                    | 0           0         1 |   
    */
    // rename joint angles
    float theta1 = theta[0];
    float theta2 = theta[1];

    // calculate necessary parameters to construct wrelb
    float x = A1*cosf(theta1) + A2*cosf(theta2);
    float y = A1*sinf(theta1) + A2*sinf(theta2);
    float phi = theta1+theta2;

    // construct rows for wrelb
    vector<float> r1{cosf(phi), -sinf(phi), x};
    vector<float> r2{sin(phi), cos(phi), y};
    vector<float> r3{0, 0, 1};

    // populate wrelb with rows
    vector<vector<float>> wrelb{r1, r2, r3};

    return wrelb;
}

vector<vector<float>> WHERE(vector<float> theta, vector<vector<float>> trelw, vector<vector<float>> brels) {
    /*  Inputs: 
            theta: joint angles
            trelw: Tool frame WRT Wrist frame
            brels: Base frame WRT Station frame
        Output:
            trels: Tool frame WRT Station frame
    */

    // get wrelb
    vector<vector<float>> wrelb = KIN(theta);
    
    // do matrix multiplications to calculate trels = brels x wrelb x trelw
    vector<vector<float>> wrels = TMULT(brels, wrelb); // wrels = brels x wrelb
    vector<vector<float>> trels = TMULT(wrels, trelw); // trels = wrels x trelw

    return trels;
}
    

// Inverse Kinematics

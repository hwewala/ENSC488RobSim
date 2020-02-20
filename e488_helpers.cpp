#include "e488_helpers.h"

///////////////
/* CONSTANTS */
///////////////
// lengths are in (mm)
// angles are in (rads)
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

///////////////////////////////////////////////
/* Part 1: Basic Matrix Computation Routines */
///////////////////////////////////////////////

// User form TO Internal form
vector<vector<float>> UTOI(vector<float> pos) {
    /*  Description: User form to internal form
        Assume all rotations are about Z axis
        Input:
            pos = (x, y, z)
        Output:
            T = | cos(theta)    -sin(theta)     0   x |
                | sin(theta)    cos(theta)      0   y |
                | 0             0               1   z |
                | 0             0               0   1 |
    */

    // get the different position values
    float x = pos[0];
    float y = pos[1];
    float z = pos[2];

    // calculate parameters for transformation matrix, T
    float theta = atan2f(y, x);
    float s_theta = sinf(theta);
    float c_theta = cosf(theta);

    // populate T
    vector<float> r1{c_theta, -s_theta, 0, x};
    vector<float> r2{s_theta, c_theta,  0, y};
    vector<float> r3{0,       0,        1, z};
    vector<float> r4{0,       0,        0, 1};
    
    vector<vector<float>> T{r1, r2, r3, r4};

    return T;
}

// Internal form TO User form
vector<float> ITOU(vector<vector<float>> T) {
    /*  Description: Internal form to user form
        Assume all rotations are around Z-axis 
        Input:
            T = | cos(theta)    -sin(theta)     0   x |
                | sin(theta)    cos(theta)      0   y |
                | 0             0               1   z |
                | 0             0               0   1 |
        Output:
            pos = (x, y, z)
    */

    // getting (x, y, z) values from T (as defined above)
    float x = T[0][3];
    float y = T[1][3];
    float z = T[2][3];
    float theta = acosf(T[0][0]);

    vector<float> pos{x, y, z, theta};

    return pos;
}

// T MULTiplication
vector<vector<float>> TMULT(vector<vector<float>> brela, vector<vector<float>> crelb) {
    /*  Desription: Multiplies two matrices together
        Input: brela, crelb
        Output: crela
        Matrices are in the form of T
            T = | cos(theta)    -sin(theta)     0   x |
                | sin(theta)    cos(theta)      0   y |
                | 0             0               1   z |
                | 0             0               0   1 |
    */
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
vector<vector<float>> TINVERT(vector<vector<float>> T) {
    /*  Description: Performs the inverse of a 4x4 matrix
        Input: 
            T = | cos(theta)    -sin(theta)     0   x |
                | sin(theta)    cos(theta)      0   y |
                | 0             0               1   z |
                | 0             0               0   1 |
        Output: inverse of T
    */
    int N = size(T);
    float det = det_mat(T); // NxN matrix
    vector<vector<float>> inv_mat;

    // cycle through all the coordinates in the (N-1)x(N-1) matrix
    for(int i = 0; i < N; i++) {
        vector<float> lst_mats;
        for(int j = 0; j < N; j++) {
            vector<vector<float>> min_mat = minor_mat(T, i, j);
            float det_min = det_mat(min_mat); // (N-1)x(N-1) matrix

            // play with the signs
            if((i+j) % 2 != 0) {
                det_min = -det_min;
                printf("i: %i, j: %i\n", i, j);
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
    /*  Description: Computers the wrist frame WRT the base frame
        Inputs:
            theta: input joint angles (theta1, theta2, theta4)
        Output:
            wrelb: the wrist frame WRT the base frame. 
    */
    // assign joint angles to variables
    float theta1 = theta[0];
    float theta2 = theta[1];
    float d3 = theta[2];
    float theta4 = theta[3];
    float phi = theta1 + theta2 - theta4;
    
    // compute sines and cosines
    float c_phi = cosf(phi);
    float s_phi = sinf(phi);

    // populate rows for wrelb
    vector<float> r1{c_phi,  s_phi,  0, L142*cosf(theta1+theta2) + L195*cosf(theta1)};
    vector<float> r2{s_phi, -c_phi,  0, L142*sinf(theta1+theta2) + L195*sinf(theta1)};
    vector<float> r3{0,      0,     -1, L70-(L410 - d3)};
    vector<float> r4{0,      0,      0, 1};

    vector<vector<float>> wrelb{r1, r2, r3, r4};
    
    return wrelb;
}

vector<vector<float>> WHERE(vector<float> theta, vector<vector<float>> brels, vector<vector<float>> trelw) {
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

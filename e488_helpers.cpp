#include "e488_helpers.h"
#include "ensc-488.h"

// converts deg to rad
float torad(float deg) {
    return deg*(PI/180);
}

float cosd(float deg){
    return cosf(torad(deg));
}

float sind(float deg){
    return sinf(torad(deg));
}

float acosd(float val){
    return acosf(torad(val));
}

float asind(float val){
    return asinf(torad(val));
}

///////////////////////////////////////////////
/* Part 1: Basic Matrix Computation Routines */
///////////////////////////////////////////////

// User form TO Internal form
vector<vector<float>> UTOI(vector<float> curr_pos) {
    /*  Description: User form to internal form
        Assume all rotations are about Z axis
        Input:
            pos = (x, y, z, phi)
        Output:
            T = | cos(phi)    -sin(phi)     0   x |
                | sin(phi)    cos(phi)      0   y |
                | 0             0           1   z |
                | 0             0           0   1 |
    */

    // get the different position values
    float x = curr_pos[0];
    float y = curr_pos[1];
    float z = curr_pos[2];
    float phi = curr_pos[3];

    // calculate parameters for transformation matrix, T
    float s_phi = sinf(phi);
    float c_phi = cosf(phi);

    // populate transfomation matrix
    vector<float> r1{c_phi, -s_phi, 0, x};
    vector<float> r2{s_phi, c_phi,  0, y};
    vector<float> r3{0,       0,    1, z};
    vector<float> r4{0,       0,    0, 1};
    
    vector<vector<float>> mat{r1, r2, r3, r4};

    return mat;
}

// Internal form TO User form
vector<float> ITOU(vector<vector<float>> mat) {
    /*  Description: Internal form to user form
        Assume all rotations are around Z-axis 
        Input:
            T = | cos(phi)    -sin(phi)     0   x |
                | sin(phi)    cos(phi)      0   y |
                | 0             0           1   z |
                | 0             0           0   1 |
        Output:
            pos = (x, y, z, phi)
    */

    // getting (x, y, z) values from T (as defined above)
    float x = mat[0][3];
    float y = mat[1][3];
    float z = mat[2][3];
    float phi = acosf(mat[0][0]);

    vector<float> pos{x, y, z, phi};

    return pos;
}

// T MULTiplication
vector<vector<float>> TMULT(vector<vector<float>> brela, vector<vector<float>> crelb) {
    /*  Desription: Multiplies two matrices together
        Input: brela, crelb
        Output: crela
        Matrices are in the form of T
            T = | cos(phi)    -sin(phi) 0   x |
                | sin(phi)    cos(phi)  0   y |
                | 0             0       1   z |
                | 0             0       0   1 |
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

vector<float> TMULT(vector<float> P, vector<vector<float>> R) {
    /*  Desription: Multiplies two matrices together
        Input: P (3x1), R (3x3)
        Output: P*R (3x1)
    */
    vector<float> P_new;
    int N = size(P);

    for(int i = 0; i < N; i++) {        
        // iterate through the rows
        vector<float> vec;
        float sum = 0;
        for(int j = 0; j < N; j++) {
            // iterate through the columns
            sum += P[j]*R[i][j];
        }
        P_new.push_back(sum);
    }
    return P_new;
}


// Invert Matrix
vector<vector<float>> TINVERT(vector<vector<float>> mat) {
    /*  Description: Performs the inverse of a 4x4 matrix
        Input: 
            T = | cos(phi)    -sin(phi) 0   x |
                | sin(phi)    cos(phi)  0   y |
                | 0             0       1   z |
                | 0             0       0   1 |
        Output:
        inv(T) = |R(A wrt B) -R(A wrt B) AP(BORG)|
                 | 0 0 0     1                   | 
    */
    int N = 3;

    // get Rotation matrix
    vector<vector<float>> R;
    for(int i = 0; i < N; i++) {
        vector<float> r;
        for(int j = 0; j < N; j++) {
            r.push_back(mat[i][j]);
        }
        R.push_back(r);
    }

    // invert R
    transpose_mat(R);

    // get Position vector
    vector<float> P;
    for(int i = 0; i < N; i++) {
        P.push_back(mat[i][N]);
    }

    vector<float> P_new = TMULT(P, R);

    // construct new matrix, T
    vector<float> r1{R[0][0], R[0][1], R[0][2], P_new[0]};
    vector<float> r2{R[1][0], R[1][1], R[1][2], P_new[1]};
    vector<float> r3{R[2][0], R[2][1], R[2][2], P_new[2]};
    vector<float> r4{0, 0, 0, 1};
    vector<vector<float>> inv_mat{r1, r2, r3, r4};
    
    return inv_mat;
}

///////////////////////////////////////////
/* Part 2: Forward and Inverse Kinematics*/
///////////////////////////////////////////

// Forward Kinematics
vector<vector<float>> KIN(vector<float> joint_vals){
    /*  Description: Computers the (WRELB) wrist frame WRT the base frame
        Inputs:
            joint_vals: input joint angles (theta1, theta2, d3, theta4)
        Output:
            wrelb: the wrist frame WRT the base frame. 
    */
    // assign joint angles to variables
    float theta1 = joint_vals[0];
    float theta2 = joint_vals[1];
    float d3 = joint_vals[2];
    float theta4 = joint_vals[3];
    float phi = theta1 + theta2 - theta4;
    
    // compute sines and cosines
    float c_phi = cosf(phi);
    float s_phi = sinf(phi);

    // populate rows for wrelb
    // other one, trelw should have 140
    vector<float> r1{c_phi,  s_phi,  0, L142*cosf(theta1+theta2) + L195*cosf(theta1)};
    vector<float> r2{s_phi, -c_phi,  0, L142*sinf(theta1+theta2) + L195*sinf(theta1)};
    vector<float> r3{0,      0,     -1, L70-(L410 - d3)};
    vector<float> r4{0,      0,      0, 1};

    // hakeem's other solution, trelw should have z = 60
    // vector<float> r1{c_phi,  s_phi,  0, L142*cosf(theta1+theta2) + L195*cosf(theta1)};
    // vector<float> r2{s_phi, -c_phi,  0, L142*sinf(theta1+theta2) + L195*sinf(theta1)};
    // vector<float> r3{0,      0,     -1, d3-420};
    // vector<float> r4{0,      0,      0, 1};

    vector<vector<float>> wrelb{r1, r2, r3, r4};
    
    return wrelb;
}

vector<float> WHERE(vector<float> joint_vals, vector<vector<float>> brels, vector<vector<float>> trelw) {
    /*  Inputs: 
            joint_vals: joint angles
            trelw: Tool frame WRT Wrist frame
            brels: Base frame WRT Station frame
        Output:
            position of wrist (x, y, z, phi) with respect to the station frame
    */

    // get wrelb
    vector<vector<float>> wrelb = KIN(joint_vals);
    
    // do matrix multiplications to calculate trels = brels x wrelb x trelw
    vector<vector<float>> wrels = TMULT(brels, wrelb); // wrels = brels x wrelb
    vector<vector<float>> trels = TMULT(wrels, trelw); // trels = wrels x trelw

    vector<float> pos = ITOU(trels);

    return pos;
}
    
// Inverse Kinematics
// Given a frame, finds the appropriate joint values
void INVKIN(vector<vector<float>> wrelb, JOINT curr_joint, vector<float> &near, vector<float> &far, bool &sol){
    /*  Description: Finds the inverse kinematics of the robot
        Inputs:
            wrelb: Wrist frame WRT Base frame
            curr_joint: current joint values of the robot arm (theta1, theta2, d3, theta4)
        Outputs: (pass these as inputs b/c we don't want all of them to have the same type)
            near: nearest solution
            far: second solution
            sol: determines if there is even a solution
        
        TODO:
            - find the nearest solution, by calculating the difference from the current position
            - assign weights for each joint
    */
    // get relevant values
    float x = wrelb[0][3];
    float y = wrelb[1][3];
    float z = wrelb[2][3];
    float c_phi = wrelb[0][0];
    float s_phi = wrelb[0][1];

    // pow(base, exponent) for x^2 = pow(x, 2)
    float c_theta2 = (pow(x, 2) + pow(y,2) - pow(L142, 2) - pow(L195, 2))/(2*L142*L195);
    float s_theta2 = sqrt(1 - pow(c_theta2, 2));
    float theta2_p = atan2f(s_theta2, c_theta2);
    // float theta2_n = atan2f(-s_theta2, c_theta2);

    // compute k's for theta1
    float k1_p = L195 + L142*cosf(theta2_p);
    float k2_p = L142*sinf(theta2_p);

    // float k1_n = L195 + L142*cosf(theta2_n);
    // float k2_n = L142*sinf(theta2_n);

    float theta1_p = atan2f(y, x) - atan2f(k2_p, k1_p);
    // float theta1_n = gamma - atan2f(k2_n, k1_n);

    // compute theta4, phi = theta1 + theta2 - theta4, theta4 = theta1 + theta2 - phi
    float phi = atan2f(s_phi, c_phi);
    float theta4_p = theta1_p + theta2_p - phi;
    // float theta4_n = theta1_n + theta2_n - phi;

    // compute d3
    float d3 = z - L70 + L410;

    vector<float> vp{theta1_p, theta2_p, d3, theta4_p};
    // vector<float> vn{theta1_n, theta2_n, d3, theta4_n};
    int N = size(vp);
    for(int i = 0; i < N; i++) {
        near.push_back(vp[i]);
        // far.push_back(vn[i]);
    }
}

// Given trels and brels, finds trelw
void SOLVE(vector<float> tar_pos, vector<float> curr_pos, vector<float> &near, vector<float> &far, bool &sol) {
    /*  Description: 
            Given a desired position (x, y, z, phi), determine the joint configuration closest to the 
            current joint configuration closest and move the robot to that configuration
        Inputs:
            trels: Tool frame WRT Station frame
            srelb: Station frame WRT Base frame
        Ouptuts:
            the position of the tool (x, y, z, phi), from the station frame
        TODO:
            figure out what is happening
    */

    // get wrelb
    vector<vector<float>> wrels = UTOI(curr_pos);
    // get brels
    vector<vector<float>> brels = UTOI({0,0,L405,0});
    vector<vector<float>> srelb = TINVERT(brels);

    // do trelw = srelb * wrels
    vector<vector<float>> wrelb = TMULT(srelb, wrels);

    // find nearest solution with INVKIN
    INVKIN(wrelb, tar_pos, near, far, sol);
}
#include "e488_helpers.h"

// converts deg to rad
double torad(double deg) {
    return deg*(PI/180);
}

double cosd(double deg){
    return cosf(torad(deg));
}

double sind(double deg){
    return sinf(torad(deg));
}

double acosd(double val){
    return acosf(torad(val));
}

double asind(double val){
    return asinf(torad(val));
}


///////////////////////////////////////////////
/* Part 1: Basic Matrix Computation Routines */
///////////////////////////////////////////////

// // User form TO Internal form
vector<JOINT> UTOI(JOINT curr_pos) {
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
    double x = curr_pos[0];
    double y = curr_pos[1];
    double z = curr_pos[2];
    double phi = curr_pos[3];

    // calculate parameters for transformation matrix, T
    double s_phi = sinf(phi);
    double c_phi = cosf(phi);

    // populate transfomation matrix
    JOINT r1{c_phi, -s_phi, 0, x};
    JOINT r2{s_phi, c_phi,  0, y};
    JOINT r3{0,       0,    1, z};
    JOINT r4{0,       0,    0, 1};
    
    vector<JOINT> mat;
    mat.push_back(r1);
    mat.push_back(r2);
    mat.push_back(r3);
    mat.push_back(r4);

    return mat;
}

// Internal form TO User form
// JOINT ITOU(vector<JOINT> mat) {
//     /*  Description: Internal form to user form
//         Assume all rotations are around Z-axis 
//         Input:
//             T = | cos(phi)    -sin(phi)     0   x |
//                 | sin(phi)    cos(phi)      0   y |
//                 | 0             0           1   z |
//                 | 0             0           0   1 |
//         Output:
//             pos = (x, y, z, phi)
//     */

//     // getting (x, y, z) values from T (as defined above)
//     double x = mat[0][3];
//     double y = mat[1][3];
//     double z = mat[2][3];
//     double phi = acosf(mat[0][0]);

//     JOINT pos = [x, y, z, phi];

//     return pos;
// }

// // T MULTiplication
// vector<JOINT> TMULT(vector<JOINT> brela, vector<JOINT> crelb) {
//     /*  Desription: Multiplies two matrices together
//         Input: brela, crelb
//         Output: crela
//         Matrices are in the form of T
//             T = | cos(phi)    -sin(phi) 0   x |
//                 | sin(phi)    cos(phi)  0   y |
//                 | 0             0       1   z |
//                 | 0             0       0   1 |
//     */
//     vector<JOINT> crela;
//     int N = size(brela);

//     for(int i = 0; i < N; i++) {        
//         // iterate through the rows
//         JOINT vec1;
//         for(int j = 0; j < N; j++) {
//             // iterate through the columns
//             JOINT vec2;
//             double sum = 0;
//             for(int k = 0; k < N; k++) {
//                 // do matrix calculations
//                 vec2[k] = brela[i][k]*crelb[k][j]);
//                 sum = sum_vec(vec2);
//             }
//             vec1[j] = sum;
//         }
//         crela.push_back(vec1);
//     }
//     return crela;
// }

// JOINT TMULT(JOINT P, vector<JOINT> R) {
//     /*  Desription: Multiplies two matrices together
//         Input: P (3x1), R (3x3)
//         Output: P*R (3x1)
//     */
//     JOINT P_new;
//     int N = sizeof(P)/sizeof*(P);
//     printf("N: %d\n", N);
//     for(int i = 0; i < N; i++) {        
//         // iterate through the rows
//         JOINT vec;
//         double sum = 0;
//         for(int j = 0; j < N; j++) {
//             // iterate through the columns
//             sum += P[j]*R[i][j];
//         }
//         P_new[i] = sum;
//     }
//     return P_new;
// }


// // Invert Matrix
// vector<JOINT> TINVERT(vector<JOINT> mat) {
//     /*  Description: Performs the inverse of a 4x4 matrix
//         Input: 
//             T = | cos(phi)    -sin(phi) 0   x |
//                 | sin(phi)    cos(phi)  0   y |
//                 | 0             0       1   z |
//                 | 0             0       0   1 |
//         Output:
//         inv(T) = |R(A wrt B) -R(A wrt B) AP(BORG)|
//                  | 0 0 0     1                   | 
//     */
//     int N = 3;

//     // get Rotation matrix
//     vector<JOINT> R;
//     for(int i = 0; i < N; i++) {
//         JOINT r;
//         for(int j = 0; j < N; j++) {
//             r.push_back(mat[i][j]);
//         }
//         R.push_back(r);
//     }

//     // invert R
//     transpose_mat(R);

//     // get Position vector
//     JOINT P;
//     for(int i = 0; i < N; i++) {
//         P.push_back(mat[i][N]);
//     }

//     JOINT P_new = TMULT(P, R);

//     // construct new matrix, T
//     JOINT r1{R[0][0], R[0][1], R[0][2], P_new[0]};
//     JOINT r2{R[1][0], R[1][1], R[1][2], P_new[1]};
//     JOINT r3{R[2][0], R[2][1], R[2][2], P_new[2]};
//     JOINT r4{0, 0, 0, 1};
//     vector<JOINT> inv_mat{r1, r2, r3, r4};
    
//     return inv_mat;
// }

// ///////////////////////////////////////////
// /* Part 2: Forward and Inverse Kinematics*/
// ///////////////////////////////////////////

// // Forward Kinematics
// vector<JOINT> KIN(JOINT joint_vals){
//     /*  Description: Computers the (WRELB) wrist frame WRT the base frame
//         Inputs:
//             joint_vals: input joint angles (theta1, theta2, d3, theta4)
//         Output:
//             wrelb: the wrist frame WRT the base frame. 
//     */
//     // assign joint angles to variables
//     double theta1 = joint_vals[0];
//     double theta2 = joint_vals[1];
//     double d3 = joint_vals[2];
//     double theta4 = joint_vals[3];
//     double phi = theta1 + theta2 - theta4;
    
//     // compute sines and cosines
//     double c_phi = cosf(phi);
//     double s_phi = sinf(phi);

//     // populate rows for wrelb
//     // other one, trelw should have 140
//     JOINT r1{c_phi,  s_phi,  0, L142*cosf(theta1+theta2) + L195*cosf(theta1)};
//     JOINT r2{s_phi, -c_phi,  0, L142*sinf(theta1+theta2) + L195*sinf(theta1)};
//     JOINT r3{0,      0,     -1, L70-(L410 - d3)};
//     JOINT r4{0,      0,      0, 1};

//     // hakeem's other solution, trelw should have z = 60
//     // JOINT r1{c_phi,  s_phi,  0, L142*cosf(theta1+theta2) + L195*cosf(theta1)};
//     // JOINT r2{s_phi, -c_phi,  0, L142*sinf(theta1+theta2) + L195*sinf(theta1)};
//     // JOINT r3{0,      0,     -1, d3-420};
//     // JOINT r4{0,      0,      0, 1};

//     vector<JOINT> wrelb{r1, r2, r3, r4};
    
//     return wrelb;
// }

// JOINT WHERE(JOINT joint_vals, vector<JOINT> brels, vector<JOINT> trelw) {
//     /*  Inputs: 
//             joint_vals: joint angles
//             trelw: Tool frame WRT Wrist frame
//             brels: Base frame WRT Station frame
//         Output:
//             position of wrist (x, y, z, phi) with respect to the station frame
//     */

//     // get wrelb
//     vector<JOINT> wrelb = KIN(joint_vals);
    
//     // do matrix multiplications to calculate trels = brels x wrelb x trelw
//     vector<JOINT> wrels = TMULT(brels, wrelb); // wrels = brels x wrelb
//     vector<JOINT> trels = TMULT(wrels, trelw); // trels = wrels x trelw

//     JOINT pos = ITOU(trels);

//     return pos;
// }
    
// // Inverse Kinematics
// // Given a frame, finds the appropriate joint values
// void INVKIN(vector<JOINT> wrelb, JOINT curr_joint, JOINT &near, JOINT &far, bool &sol){
//     /*  Description: Finds the inverse kinematics of the robot
//         Inputs:
//             wrelb: Wrist frame WRT Base frame
//             curr_joint: current joint values of the robot arm (theta1, theta2, d3, theta4)
//         Outputs: (pass these as inputs b/c we don't want all of them to have the same type)
//             near: nearest solution
//             far: second solution
//             sol: determines if there is even a solution
        
//         TODO:
//             - find the nearest solution, by calculating the difference from the current position
//             - assign weights for each joint
//     */
//     // get relevant values
//     double x = wrelb[0][3];
//     double y = wrelb[1][3];
//     double z = wrelb[2][3];
//     double c_phi = wrelb[0][0];
//     double s_phi = wrelb[0][1];

//     // pow(base, exponent) for x^2 = pow(x, 2)
//     double c_theta2 = (pow(x, 2) + pow(y,2) - pow(L142, 2) - pow(L195, 2))/(2*L142*L195);
//     double s_theta2 = sqrt(1 - pow(c_theta2, 2));
//     double theta2_p = atan2f(s_theta2, c_theta2);
//     // double theta2_n = atan2f(-s_theta2, c_theta2);

//     // compute k's for theta1
//     double k1_p = L195 + L142*cosf(theta2_p);
//     double k2_p = L142*sinf(theta2_p);

//     // double k1_n = L195 + L142*cosf(theta2_n);
//     // double k2_n = L142*sinf(theta2_n);

//     double theta1_p = atan2f(y, x) - atan2f(k2_p, k1_p);
//     // double theta1_n = gamma - atan2f(k2_n, k1_n);

//     // compute theta4, phi = theta1 + theta2 - theta4, theta4 = theta1 + theta2 - phi
//     double phi = atan2f(s_phi, c_phi);
//     double theta4_p = theta1_p + theta2_p - phi;
//     // double theta4_n = theta1_n + theta2_n - phi;

//     // compute d3
//     double d3 = z - L70 + L410;

//     JOINT vp{theta1_p, theta2_p, d3, theta4_p};
//     // JOINT vn{theta1_n, theta2_n, d3, theta4_n};
//     int N = size(vp);
//     for(int i = 0; i < N; i++) {
//         near.push_back(vp[i]);
//         // far.push_back(vn[i]);
//     }
// }

// // Given trels and brels, finds trelw
// void SOLVE(JOINT tar_pos, JOINT curr_pos, JOINT &near, JOINT &far, bool &sol) {
//     /*  Description: 
//             Given a desired position (x, y, z, phi), determine the joint configuration closest to the 
//             current joint configuration closest and move the robot to that configuration
//         Inputs:
//             trels: Tool frame WRT Station frame
//             srelb: Station frame WRT Base frame
//         Ouptuts:
//             the position of the tool (x, y, z, phi), from the station frame
//         TODO:
//             figure out what is happening
//     */

//     // get wrelb
//     vector<JOINT> wrels = UTOI(curr_pos);
//     // get brels
//     vector<JOINT> brels = UTOI({0,0,L405,0});
//     vector<JOINT> srelb = TINVERT(brels);

//     // do trelw = srelb * wrels
//     vector<JOINT> wrelb = TMULT(srelb, wrels);

//     // find nearest solution with INVKIN
//     INVKIN(wrelb, tar_pos, near, far, sol);
// }
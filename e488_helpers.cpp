#include "e488_helpers.h"

// User form TO Internal form
vector<vector<float>> UTOI(float x, float y) {
    // We want to allow the user to specify a frame as a 3-tuple (x, y, z)
    // Assume only dealing with 3x3 matrices
    // Assume all rotations are around Z-axis
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
    // user will specify a 3x3 rotation matrix, and will output (x, y, theta)
    vector<float> result;
    return result;
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
                vec2.push_back(brela[j][k]*crelb[k][j]);
                sum = sum_vec(vec2);
            }            
            vec1.push_back(sum);
        }
        crela.push_back(vec1);
    }
    return crela;
}


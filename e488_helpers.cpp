#include "e488_helpers.h"

// User form TO Internal form
vector<vector<float>> UTOI(float x, float y) {
	// user will specify (x, y)
    // we want to allow the user to specify a frame as a 3-tuple (x, y, theta)
    float theta = atan2(y, x) * 180/PI;
}

// Internal form TO User form
vector<float> ITOU(vector<vector<float>> R) {
    // user will specify a 3x3 rotation matrix, and will output (x, y, theta)

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
                sum = vSum(vec2);
            }            
            vec1.push_back(sum);
        }
        crela.push_back(vec1);
    }
    return crela;
}

// vector sum
float vSum(vector<float> vec) {
    // sum the elements in a vector
    int N = size(vec);
    float sum = 0;
    for(int i = 0; i < N; i++) {
        sum += vec[i];
    }
    return sum;
}

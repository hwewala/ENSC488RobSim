#include "vec_help.h"

// finds the determinant of a matrix
float det_mat(vector<vector<float>> mat) {
    /* Assume 3x3 matrix
    */
   int N = 3;
   vector<float> det_vals;
   for(int i = 0; i < N; i++) {
        // cycle through the values in the top row
        int r = 0;
        float val = mat[r][i];
        // get the smaller, 2x2 matrix
        vector<vector<float>> mat1;
        for(int j = 0; j < N; j++) {
            // cycle through the rows
            vector<float> vec;
            for(int k = 0; k < N; k++){
                // cycle through the columns
                if(j != r && k != i) {
                    vec.push_back(mat[j][k]);
                }
            }
            if(size(vec) > 0) {
                mat1.push_back(vec);
            }
        }
       
        // determinant of 2x2 matrix: ad - bc
        float a = mat1[0][0];
        float b = mat1[0][1];
        float c = mat1[1][0];
        float d = mat1[1][1];
        float ad_bc = (a*d)-(b*c);
        float res = val*ad_bc;
        if(i % 2 != 0) {
            res = -res;
        }
        det_vals.push_back(res);
    }
    float det = sum_vec(det_vals); 
    return det;
}

// prints the matrix
void print_mat(vector<vector<float>> mat) {
    // Assume NxN matrix
    int N = size(mat[0]);
    for(int i = 0; i < N; i++) {
        print_vec(mat[i]);
    }
    printf("\n");
}

// prints all elements in a vector
void print_vec(vector<float> vec) {
    int N = size(vec);
    vector<string> vec_str;
    printf("[");
    for(int i = 0; i < N; i++) {
        printf("%f", vec[i]);
        if(i < N-1) {
            printf("  ");
        }
    }
    printf("]\n");
}


// sums all elements in a vector
float sum_vec(vector<float> vec) {
    // sum the elements in a vector
    int N = size(vec);
    float sum = 0;
    for(int i = 0; i < N; i++) {
        sum += vec[i];
    }
    return sum;
}

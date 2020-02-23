#include "vec_help.h"

// finds the determinant of a matrix
float det_mat(vector<vector<float>> mat) {
    /*  Description: Finds the determinant of a matrix
        Input: NxN matrix, 2x2 matrix
    */
   int N = size(mat);
   float det;
   switch(N) {
        case 2 : 
            det = det_mat2(mat);
            break;
        default : 
            det = det_matN(mat);
            break;
   }
   return det;
}

// finds the determinant of a 3x3 matrix
float det_matN(vector<vector<float>> mat) {
    int N = size(mat);
    vector<float> det_vals;
    int r = 0;
    for(int i = 0; i < N; i++) {
            // cycle through the values in the top row
            float val = mat[r][i];
            vector<vector<float>> min_mat = minor_mat(mat, r, i);
        
            // determinant the minor matrix
            float ad_bc = det_mat(min_mat);
            float res = val*ad_bc;
            if(i % 2 != 0) {
                res = -res;
            }
            det_vals.push_back(res);
        }
        float det = sum_vec(det_vals); 
    return det;
}

// finds the determinant of a 2x2 matrix
float det_mat2(vector<vector<float>> mat) {
    /* Assume 2x2 matrix */
    float a = mat[0][0];
    float b = mat[0][1];
    float c = mat[1][0];
    float d = mat[1][1];
    float det = (a*d)-(b*c);

    return det;
}

// gets the minor matrix
vector<vector<float>> minor_mat(vector<vector<float>> mat, int r, int c) {
    int N = size(mat);
    vector<vector<float>> min_mat;
    for(int i = 0; i < N; i++) {
        // cycle through the rows
        vector<float> vec;
        for(int j = 0; j < N; j++){
            // cycle through the columns
            if(i != r && j != c) {
                vec.push_back(mat[i][j]);
            }
        }
        if(size(vec) > 0) {
            min_mat.push_back(vec);
        }
    }
    return min_mat;
}

// prints the matrix
void print(vector<vector<float>> mat) {
    // Assume NxN matrix
    int N = size(mat[0]);
    for(int i = 0; i < N; i++) {
        print(mat[i]);
    }
    printf("\n");
}

// prints all elements in a vector
void print(vector<float> vec) {
    int N = size(vec);
    vector<string> vec_str;
    printf("[");
    for(int i = 0; i < N; i++) {
        printf("%f", vec[i]);
        if(i < N-1) {
            printf("\t");
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

// transpose a NxN matrix
void transpose_mat(vector<vector<float>> &mat) {
    int N = size(mat);
    vector<float> r(N);
    vector<vector<float>> temp;
    for(int i = 0; i < N; i++) {
        temp.push_back(r);
    }
    // save transposed values to a temp matrix
    for(int i = 0; i < N; i++){
        for(int j = 0; j < N; j++) {
            temp[i][j] = mat[j][i];
        }
    }
    // rewrite input
    mat = temp;
}
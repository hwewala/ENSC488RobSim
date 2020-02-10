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
        vector<vector<float>> mat1 = minor_mat(mat, r, i);
       
        // determinant of 2x2 matrix: ad - bc
        float ad_bc = det_mat2(mat1);
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

// gets the minor matrix (2x2) of a matrix (3x3)
vector<vector<float>> minor_mat(vector<vector<float>> mat, int r, int c) {
    // get the smaller, 2x2 matrix
    int N = 3;
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

// finds the inverse of the matrix
vector<vector<float>> inv_mat(vector<vector<float>> mat) {
    /*Assume 3x3 matrix
    Followed steps here: https://www.mathsisfun.com/algebra/matrix-inverse-minors-cofactors-adjugate.html
    */
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

// transpose a 3x3 matrix
void transpose_mat(vector<vector<float>> &mat) {
    int N = 3;
    vector<float> r(N);
    vector<vector<float>> temp{r, r, r};
    // save transposed values to a temp matrix
    for(int i = 0; i < N; i++){
        for(int j = 0; j < N; j++) {
            temp[i][j] = mat[j][i];
        }
    }
    // rewrite input
    mat = temp;
}
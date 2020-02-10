#include "vec_help.h"

// prints the matrix
void print_mat(vector<vector<float>> mat) {
    // Assume 3x3 matrix
    int N = 3;
    printf("Matrix: \n");
    for(int i = 0; i < N; i++) {
        print_vec(mat[i]);
    }
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

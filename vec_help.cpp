// #include "vec_help.h"

// int size(JOINT arr) {
//     return sizeof(arr)/sizeof(*arr);
// }


// // finds the determinant of a matrix
// double det_mat(vector<JOINT> mat) {
//     /*  Description: Finds the determinant of a matrix
//         Input: NxN matrix, 2x2 matrix
//     */
//    int N = size(mat);
//    double det;
//    switch(N) {
//         case 2 : 
//             det = det_mat2(mat);
//             break;
//         default : 
//             det = det_matN(mat);
//             break;
//    }
//    return det;
// }

// // finds the determinant of a 3x3 matrix
// double det_matN(vector<JOINT> mat) {
//     int N = size(mat);
//     JOINT det_vals;
//     int r = 0;
//     for(int i = 0; i < N; i++) {
//             // cycle through the values in the top row
//             double val = mat[r][i];
//             vector<JOINT> min_mat = minor_mat(mat, r, i);
        
//             // determinant the minor matrix
//             double ad_bc = det_mat(min_mat);
//             double res = val*ad_bc;
//             if(i % 2 != 0) {
//                 res = -res;
//             }
//             det_vals.push_back(res);
//         }
//         double det = sum_vec(det_vals); 
//     return det;
// }

// // finds the determinant of a 2x2 matrix
// double det_mat2(vector<JOINT> mat) {
//     /* Assume 2x2 matrix */
//     double a = mat[0][0];
//     double b = mat[0][1];
//     double c = mat[1][0];
//     double d = mat[1][1];
//     double det = (a*d)-(b*c);

//     return det;
// }

// // gets the minor matrix
// vector<JOINT> minor_mat(vector<JOINT> mat, int r, int c) {
//     int N = size(mat);
//     vector<JOINT> min_mat;
//     for(int i = 0; i < N; i++) {
//         // cycle through the rows
//         JOINT vec;
//         for(int j = 0; j < N; j++){
//             // cycle through the columns
//             if(i != r && j != c) {
//                 vec.push_back(mat[i][j]);
//             }
//         }
//         if(size(vec) > 0) {
//             min_mat.push_back(vec);
//         }
//     }
//     return min_mat;
// }

// // prints the matrix
// void print(vector<JOINT> mat) {
//     // Assume NxN matrix
//     int N = size(mat[0]);
//     for(int i = 0; i < N; i++) {
//         print(mat[i]);
//     }
//     printf("\n");
// }

// // prints all elements in a vector
// void print(JOINT vec) {
//     int N = size(vec);
//     vector<string> vec_str;
//     printf("[");
//     for(int i = 0; i < N; i++) {
//         printf("%f", vec[i]);
//         if(i < N-1) {
//             printf("\t");
//         }
//     }
//     printf("]\n");
// }

// // sums all elements in a vector
// double sum_vec(JOINT vec) {
//     // sum the elements in a vector
//     int N = size(vec);
//     double sum = 0;
//     for(int i = 0; i < N; i++) {
//         sum += vec[i];
//     }
//     return sum;
// }

// // transpose a NxN matrix
// void transpose_mat(vector<JOINT> &mat) {
//     int N = size(mat);
//     JOINT r(N);
//     vector<JOINT> temp;
//     for(int i = 0; i < N; i++) {
//         temp.push_back(r);
//     }
//     // save transposed values to a temp matrix
//     for(int i = 0; i < N; i++){
//         for(int j = 0; j < N; j++) {
//             temp[i][j] = mat[j][i];
//         }
//     }
//     // rewrite input
//     mat = temp;
// }
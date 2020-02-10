#include <vector>
#include <iostream>
#include <string>
#include <stdio.h>

using namespace std;

// finds the determinant of a matrix
float det_mat(vector<vector<float>> mat);

// finds the determinant of a 2x2 matrix
float det_mat2(vector<vector<float>> mat);

// gets the minor matrix (2x2) of a matrix (3x3)
vector<vector<float>> minor_mat(vector<vector<float>> mat, int r, int c);

// finds the inverse of the matrix
vector<vector<float>> inv_mat(vector<vector<float>> mat);

// print a matrix
void print_mat(vector<vector<float>> mat);

// print the vector
void print_vec(vector<float> vec);

// sum all the elements in the vector
float sum_vec(vector<float> vec);

// transpose a 3x3 matrix
void transpose_mat(vector<vector<float>> &mat);


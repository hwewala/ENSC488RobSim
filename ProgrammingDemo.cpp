// ProgrammingDemo.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <conio.h>
#include "ensc-488.h"
#include <iostream>
#include <string>

#include "e488_helpers.h"
#include "vec_help.h"

using namespace std;

int main(int argc, char* argv[])
{
	vector<vector<float>> mat1 = UTOI(1, 2);
	vector<float> r1{10, 5, 3};
	vector<float> r2{3, 12, 4};
	vector<float> r3{6, 1, 2};
	vector<vector<float>> test_mat{r1, r2, r3};
	printf("Matrix 1:\n");
	print_mat(test_mat);
	float det = det_mat(test_mat);
	printf("determinant: %f\n", det);

	JOINT q1 = {0, 0, -100, 0};
	JOINT q2 = {90, 90, -200, 45};
	printf("Keep this window in focus, and...\n");
	

	char ch;
	int c;

	const int ESC = 27;
	
	printf("1Press any key to continue \n");
	printf("2Press ESC to exit  \n");

	c = _getch() ;

	while (1)
	{
		
		if (c != ESC)
		{
			printf("Press '1' or '2' \n");
			ch = _getch();

			if (ch == '1')
			{
				MoveToConfiguration(q1);
				//DisplayConfiguration(q1);
			}
			else if (ch == '2')
			{
				MoveToConfiguration(q2);
				//DisplayConfiguration(q2);
			}

			printf("Press any key to continue \n");
			printf("Press q to exit \n");
			c = _getch();
		}
		else
			break;
	}
	return 0;
}

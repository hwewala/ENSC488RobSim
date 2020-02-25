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
	// define brels
	vector<float> in1{ 0, 0, 405, 0 };
	vector<vector<float>> brels = UTOI(in1);

	// define trelw
	vector<float> in2{ 0, 0, 140, 0 };
	vector<vector<float>> trelw = UTOI(in2);

	// define joint values
	float theta1 = torad(float(45));
	float theta2 = torad(float(30));
	float d3 = 410;
	float theta4 = torad(float(61));
	vector<float> joint_vals{ theta1, theta2, d3, theta4 };

	vector<vector<float>> wrelb = KIN(joint_vals);
	vector<float> curr_pos = ITOU(KIN(joint_vals));

	vector<float> near;
	vector<float> far;
	bool sol;
	SOLVE({100, 200, 100}, curr_pos, near, far, sol);

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

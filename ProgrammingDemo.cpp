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
	vector<float> pos{ 0, 0, 405, 0 }; //
	vector<vector<float>> brels = UTOI(pos);
	printf("brels:\n");
	print(brels);

	// define trelw
	vector<float> pos2{ 0, 0, 140, 0 }; //
	vector<vector<float>> trelw = UTOI(pos2);
	printf("trelw:\n");
	print(trelw);

	// define w rel b
	vector<float> joint_vals{0, torad(45), 0, 0}; // theta1, theta2, d3, theta4
	vector<vector<float>> wrelb = KIN(joint_vals);
	printf("wrelb:\n");
	print(wrelb);

	// test using inv kinematics
	vector<float> far;
	vector<float> near;
	bool sol = false;
	
	// GetConfiguration to get current joint
	JOINT curr_joint;
	GetConfiguration(curr_joint);

	printf("Inverse KIN\n");
	INVKIN(wrelb, curr_joint, near, far, sol);
	printf("near: ");
	print(near);
	printf("\n");


	vector<vector<float>> mat = KIN(near);
	printf("test_mat:\n");
	print(mat);


	//// define t rel s
	//vector<vector<float>> wrels = TMULT(brels, wrelb);
	//vector<vector<float>> trels = TMULT(wrelb, trelw);
	//printf("trels:\n");
	//print(trels);

	JOINT q1 = {0, 0, -100, 0}; // x, y, z, phi
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

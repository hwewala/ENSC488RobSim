#include "e488_proj.h"

void main(void) {
	// runs UI

	/* Menu Options:
	1. Forward Kinematics
	2. Inverse Kinematics
	3. Reset Robot
	4. Exit
	*/

	int user_input;
	bool done;
	JOINT joint_vals, spt;

	UTOI(B, brels);
	UTOI(T, trelw);

	while(true) {
		printf("Please choose a number: \n 1. Forward Kinematics \n 2. Inverse Kinematics\n 3. Reset Robot\n 4. MoveToConfiguration\n 5. Exit\n");
		printf("Your input >> ");
		cin >> user_input;

		switch(user_input) {
			case 1 : // Forward Kinematics
				FwdKin(joint_vals, spt);
				break;
			case 2 : // Inverse Kinematics 
				InvKin();
				break;
			case 3 : // Reset Robot 
				ResetRobot();
				break;
			case 4 : // MoveToConfig
				SimpleMove();
				break;
			case 5 : // Exit
				return; 
				break;
			default : 
				printf("Invalid input. Please try again.\n\n\n");
				break;
		}		
	}
	

	// // define brels
	// JOINT pos1 {0, 0, L405, 0};
	// TFORM brels;
	// UTOI(pos1, brels);
	// printf("brels:\n");
	// print(brels);

	// // define trelw
	// JOINT pos2{0, 0, L130+L10, 0};
	// TFORM trelw;
	// UTOI(pos2, trelw);
	// printf("trelw:\n");
	// print(trelw);

	// // define wrelb
	// JOINT joint1{0, DEG2RAD(45), -100, 0};
	// TFORM wrelb;
	// KIN(joint1, wrelb);
	// printf("wrelb:\n");
	// print(wrelb);

	// // TFORM trels;
	// TFORM trels, wrels;
	// TMULT(brels, wrelb, wrels);
	// printf("wrels:\n");
	// print(wrels);
	// TMULT(wrels, trelw, trels);
	// printf("trels:\n");
	// print(trels);

	// // test inv kin eqns
	// JOINT params, near, far;
	// JOINT curr_pos;
	// bool sol;
	// INVKIN(wrelb, curr_pos, near, far, sol);
	// TFORM test_mat;
	// if (sol == true) {
	// 	printf("Near: \n");
	// 	print(near);
	// 	KIN(near, test_mat);
	// 	printf("wrelb using INVKIN values:\n");
	// 	print(test_mat);
	// }
	// else
	// 	printf("No solution found given joint constraints \n\n");

	// JOINT q1 = {0, 0, -100, 0};
	// JOINT q2 = {90, 90, -200, 45};
	// printf("Keep this window in focus, and...\n");
	
	// char ch;
	// int c;

	// const int ESC = 27;
	
	// printf("1Press any key to continue \n");
	// printf("2Press ESC to exit  \n");

	// c = _getch() ;

	// while (1)
	// {
		
	// 	if (c != ESC)
	// 	{
	// 		printf("Press '1' or '2' \n");
	// 		ch = _getch();

	// 		if (ch == '1')
	// 		{
	// 			MoveToConfiguration(q1);
	// 			//DisplayConfiguration(q1);
	// 		}
	// 		else if (ch == '2')
	// 		{
	// 			MoveToConfiguration(q2);
	// 			//DisplayConfiguration(q2);
	// 		}

	// 		printf("Press any key to continue \n");
	// 		printf("Press q to exit \n");
	// 		c = _getch();
	// 	}
	// 	else
	// 		break;
	// }
	// return;
}

// Part 0: Menu Operations
void FwdKin(JOINT &joint_vals, JOINT &spt) {
	printf("\nIn Forward Kin!\n");
	double theta1_d, theta2_d, d3, theta4_d;
	
	bool valid = false;
	while(!valid) {
		// asks the user for joint values
		printf("Input joint parameters:\n");
		printf("theta1 (deg) [-150, 150]: ");
		cin >> theta1_d;
		printf("theta2 (deg) [-100, 100]: ");
		cin >> theta2_d;
		printf("d3 (mm) [-200, -100]: ");
		cin >> d3;
		printf("theta4 (deg) [-150, 150]: ");
		cin >> theta4_d;

		double theta1_r = DEG2RAD(theta1_d);
		double theta2_r = DEG2RAD(theta2_d);
		double theta4_r = DEG2RAD(theta2_d);

		// check joint values
		JOINT temp{theta1_r, theta2_r, d3, theta4_r};
		pop_arr(temp, joint_vals);
		check_joints(joint_vals, valid);

		// determine if we need different joint values
		if(!valid) {
			printf("Invalid inputs! please try again.\n\n\n");
		}
	}
	printf("\nYour inputs: [%f (rads), %f (rads), %f (mm), %f (rads)]\n\n", joint_vals[0], joint_vals[1], joint_vals[2], joint_vals[3]);

	// report position and orientation of the tool (x, y, z, phi)
	WHERE(joint_vals, spt);
	printf("Position and Orientation of Tool Frame (x, y, z, phi):\n");
	print(spt);
	printf("\n\n");

	return;
}

void InvKin(void) {
	printf("\nIn Inverse Kin!\n");

	// asks user for pose position (x, y, z, phi) of the tool 
	double x, y, z, phi;
	printf("Input position and orientation of tool:\n");
	printf("x (mm): ");
	cin >> x;
	printf("y (mm): ");
	cin >> y;
	printf("z (mm): ");
	cin >> z;
	printf("phi (deg): ");
	cin >> phi;

	printf("Your input: [%f (mm), %f (mm), %f (mm), %f (rads)]\n", x, y, z, phi);
	printf("Solving inverse kinematics...\n\n");

	JOINT t_pos{x, y, z, DEG2RAD(phi)};
	JOINT c_pos;
	JOINT near, far;
	bool sol;
	GetConfiguration(c_pos);
	SOLVE(t_pos, c_pos, near, far, sol);

	if(!sol) {
		// no solution exists!
		printf("No solution exists!\n\n");
		return;
	} 

	// will probably need to print these as a loop
	printf("all solutions: ");
	print(far);

	// print the closest solution
	printf("\n\n"); 
	printf("closest solution: ");
	print(near);

	// move to closet solution
	MoveToConfiguration(near);

	printf("\n\n");
	return;
}

void SimpleMove(void) {
	double x, y, z, phi;
	// moves robot to configuration
	printf("Move robot to configuration (x, y, z, phi):\n");
	printf("x: ");
	cin >> x;
	printf("y: ");
	cin >> y;
	printf("z: ");
	cin >> z;
	printf("phi: ");
	cin >> phi;

	printf("\nMoving robot to (%f, %f, %f, %f)...\n\n", x, y, z, phi);
	JOINT pos{ x, y, z, phi };
	MoveToConfiguration(pos);
	return;
}

void check_joints(JOINT &joint_vals, bool &valid) {	
	// checks the joint values in [rads] and [mm]
	// interpret input
	double theta1 = joint_vals[0];
	double theta2 = joint_vals[1];
	double d3 = joint_vals[2];
	double theta4 = joint_vals[3];

	// define constraints
	double theta1_cons = DEG2RAD(THETA_CONS_150);
	double theta2_cons = DEG2RAD(THETA_CONS_100);
	double theta4_cons = DEG2RAD(THETA_CONS_150);

	// check theta1 with THETA_CONS_150
	bool theta1_valid = (theta1 <= theta1_cons && theta1 >= -theta1_cons);
	bool theta2_valid = (theta2 <= theta2_cons && theta2 >= -theta2_cons);
	bool d3_valid = (d3 <= D3UPPER_100 && d3 >= D3LOWER_200);
	bool theta4_valid = (theta4 <= theta4_cons && theta4 >= -theta4_cons);

	// check to see if the inputs are valid
	valid = theta1_valid && theta2_valid && d3_valid && theta4_valid;
}

// Part 1: Basic Matrix Transformation Procedures
void UTOI(JOINT &pos, TFORM &mat) {
    /*  Description: User form to internal form
        Assume all rotations are about Z axis
        Input:
            pos = (x, y, z, phi)
        Output:
            T = | cos(phi)    -sin(phi)     0   x |
                | sin(phi)    cos(phi)      0   y |
                | 0             0           1   z |
                | 0             0           0   1 |
    */

    // get the different position values
    double x = pos[0];
    double y = pos[1];
    double z = pos[2];
    double phi = pos[3];

    // calculate parameters for transformation matrix, T
    double s_phi = sin(phi);
    double c_phi = cos(phi);

    // populate transfomation matrix
    JOINT r1 {c_phi, -s_phi, 0, x};
    JOINT r2 {s_phi, c_phi, 0, y};
    JOINT r3 {0, 0, 1, z};
    JOINT r4 {0, 0, 0, 1};

	TFORM temp {{c_phi, -s_phi, 0, x},
				{s_phi, c_phi, 0, y},
				{0, 0, 1, z},
				{0, 0, 0, 1}};
	
	pop_mat(temp, mat);

	return;
}

void ITOU(TFORM &mat, JOINT &pos) {
	/*  Description: Internal form to user form
        Assume all rotations are around Z-axis 
        Input:
            T = | cos(phi)    -sin(phi)     0   x |
                | sin(phi)    cos(phi)      0   y |
                | 0             0           1   z |
                | 0             0           0   1 |
        Output:
            pos = (x, y, z, phi)
    */

    // getting (x, y, z) values from T (as defined above)
    float x = mat[0][3];
    float y = mat[1][3];
    float z = mat[2][3];
    float phi = acos(mat[0][0]);

    JOINT temp{x, y, z, phi};
	pop_arr(temp, pos);
}

void TMULT(TFORM &brela, TFORM &crelb, TFORM &crela) {
	/*  Desription: Multiplies two matrices together
        Input: brela, crelb
        Output: crela
        Matrices are in the form of T
            T = | cos(phi)    -sin(phi) 0   x |
                | sin(phi)    cos(phi)  0   y |
                | 0             0       1   z |
                | 0             0       0   1 |
    */
   	RFORM arb, brc, arc;
	POS apb, bpc, apc;
	// get the rotation matrices from brela and crelb
	get_r(brela, arb);
	get_r(crelb, brc);

	// multiply rotation matrices: crela = brela*crelb
	rmult(arb, brc, arc);
	
	// get the position vector from brela and crelb
	get_pos(brela, apb);
	get_pos(crelb, bpc);

	// multiply rotation matrix with position vector, get apc
	POS temp;
	rmult(arb, bpc, temp);
	// add position vectors to get apc
	padd(temp, apb, apc);

	// now have the rotation matrix and position vector for crela
	tconst(arc, apc, crela);
}

void TINVERT(TFORM &tmat, TFORM &inv) {
	// performs the inverse of a 4x4 matrix

	// gets the rotation matrix
	RFORM rmat;
	get_r(tmat, rmat); //arb

	// gets the position vector
	POS pos1;
	get_pos(tmat, pos1); //apb

	// transforms the rotation matrix (ie. the inverse of rotation matrix)
	RFORM trans_mat;
	transpose_mat(rmat, trans_mat); //bra

	POS pos2, pos3;
	pmult(pos1, -1, pos2); // -apb
	rmult(trans_mat, pos2, pos3); // bra * -apb = bpa

	// constructs the new matrix
	tconst(trans_mat, pos3, inv);
}

// Part 2: Forward Kinematics
void KIN(JOINT &joint_vals, TFORM &wrelb) {
	// computes the wrelb using joint values
	double theta1 = joint_vals[0];
    double theta2 = joint_vals[1];
    double d3 = joint_vals[2];
    double theta4 = joint_vals[3];
    double phi = theta1 + theta2 - theta4;

	double c_phi = cosf(phi);
    double s_phi = sinf(phi);

	TFORM temp = {	{c_phi, s_phi, 	0, L142*cos(theta1+theta2) + L195*cos(theta1)},
					{s_phi, -c_phi, 0, L142*sin(theta1+theta2) + L195*sin(theta1)},
					{0,     0,     -1, L70-(L410 - d3)},
					{0, 	0, 		0, 1}};
	pop_mat(temp, wrelb);

}

void WHERE(JOINT &joint_vals, JOINT &spt) {
	// computes the position (x, y, z, phi) of the tool frame with respect to the station
	// get wrelb
	TFORM wrelb, wrels, trels;
	KIN(joint_vals, wrelb);
	TMULT(brels, wrelb, wrels);
	TMULT(wrels, trelw, trels);

	ITOU(trels, spt);
}

// Part 3: Inverse Kinematics

void INVKIN(TFORM &wrelb, JOINT &curr_pos, JOINT &near, JOINT &far, bool &sol) {
	// finds the inverse kinematics for the robot, then compares the solutions with the curr_pos to 
	// find the nearest solution
	// exclude solutions with bad joint values

	double x = wrelb[0][3];
	double y = wrelb[1][3];
	double z = wrelb[2][3];
	double c_phi = wrelb[0][0];
	double s_phi = wrelb[0][1];
	bool p_invalid = false;
	bool n_invalid = false;

	// pow(base, exponent) for x^2 = pow(x, 2)
	double c_theta2 = (pow(x, 2) + pow(y,2) - pow(L142, 2) - pow(L195, 2))/(2*L142*L195);
    double s_theta2 = sqrt(1 - pow(c_theta2, 2));
    double theta2_p = atan2(s_theta2, c_theta2);
	if (theta2_p > THETA_CONS_100)
		p_invalid = true;

    double theta2_n = atan2(-s_theta2, c_theta2);
	if (theta2_n < -THETA_CONS_100)
		n_invalid = true;

	if (no_sol(p_invalid, n_invalid)) {
		sol = false;
		return;
	}
	// compute k's for theta1
    double k1_p = L195 + L142*cosf(theta2_p);
    double k2_p = L142*sinf(theta2_p);

    double k1_n = L195 + L142*cosf(theta2_n);
    double k2_n = L142*sinf(theta2_n);

    double theta1_p = atan2f(y,x) - atan2f(k2_p, k1_p);
	if (theta1_p > THETA_CONS_150 && p_invalid == false)
		p_invalid = true;
	
    double theta1_n = atan2f(y,x) - atan2f(k2_n, k1_n);
	if (theta1_n < -THETA_CONS_150 && n_invalid == false)
		n_invalid = true;
	if (no_sol(p_invalid, n_invalid)) {
		sol = false;
		return;
	}
	// compute theta4
	double phi = atan2f(s_phi, c_phi);
    double theta4_p = theta1_p + theta2_p - phi;
	if (theta4_p > THETA_CONS_150 && p_invalid == false)
		p_invalid = true;
    double theta4_n = theta1_n + theta2_n - phi;
	if (theta4_n < -THETA_CONS_150 && n_invalid == false)
		n_invalid = true;

	if (no_sol(p_invalid, n_invalid)) {
		sol = false;
		return;
	}
	// compute d3
    double d3 = z - L70 + L410;
	if (d3 < D3LOWER_200 || d3 > D3UPPER_100) {
		sol = false;
		return;
	}
	sol = true;
	JOINT jp{theta1_p, theta2_p, d3, theta4_p};
	JOINT jn{theta1_n, theta2_n, d3, theta4_n};

	pop_arr(jp, near);
	pop_arr(jn, far);
}

void SOLVE(JOINT &tar_pos, JOINT &curr_pos, JOINT &near, JOINT &far, bool &sol) {
	// given a target position, determines the joint values 
	// what is happening in this function?

	TFORM wrels, brels, srelb, wrelb;
	// get wrelb
    UTOI(curr_pos, wrelb);
    // get brels
	JOINT pos = {0, 0, L405, 0};
    UTOI(pos, brels);
    TINVERT(brels, srelb);

    // do trelw = srelb * wrels
    TMULT(srelb, wrels, wrelb);

    // find nearest solution with INVKIN
    INVKIN(wrelb, tar_pos, near, far, sol);

	// sum of differences between current and final joint position for a given solution is stored in each index
	// size 2 is hardcoded since we only get two solutions
	float sums[2] = { 0, 0 };
	JOINT temp;
	// weights for each joint
	float w[4] = { 1, 1, 1, 1 };
	int M = size(sums);
	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			sums[i] += w[j] * (abs(near[j] - curr_pos[j]));
		}
	}
	//swap near and far if it is the closer joint
	if (sums[1] < sums[0]){
		pop_arr(near, temp);
		pop_arr(far, near);
		pop_arr(temp, far);
	}
}

// Part A: Helper Functions
void print(TFORM &mat) {
	// Description: prints the Transform matrix
	for(int i = 0; i < N; i++) {
        print(mat[i]);
    }
    printf("\n");
	return;
}

void print(JOINT &arr) {
	// Description: prints the (size = 4)
    printf("[");
    for(int i = 0; i < N; i++) {
        printf("%f", arr[i]);
        if(i < N-1) {
            printf("\t");
        }
    }
    printf("]\n");
	return;
}

void print(POS &arr) {
	// Description: prints the (size = 4)
    printf("[");
    for(int i = 0; i < (N-1); i++) {
        printf("%f", arr[i]);
        if(i < N-2) {
            printf("\t");
        }
    }
    printf("]\n");
	return;
}

void get_r(TFORM &tmat, RFORM &rmat) {
	// gets the Rotation matrix from the Transformation matrix
	for(int i = 0; i < (N-1); i++) {
		for(int j = 0; j < (N-1); j++) {
			rmat[i][j] = tmat[i][j];
		}
	}
}

void get_pos(TFORM &tmat, POS &pos) {
	// gets the Position vector from the Transformation matrix
	for(int i = 0; i < (N-1); i++) {
		pos[i] = tmat[i][(N-1)];
	}
}

int arr_size(JOINT &arr) {
	// Gets the size of the array
	int size = sizeof(arr)/sizeof(arr[0]);
	return size;
}

void pop_mat(TFORM &vals, TFORM &mat) {
	// populates mat with vals
	for(int i = 0; i < 4; i++) {
		for(int j = 0; j < N; j++) {
			mat[i][j] = vals[i][j];
		}
	}
}

void pop_mat(RFORM &vals, RFORM &mat) {
	// populates mat with vals
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < N; j++) {
			mat[i][j] = vals[i][j];
		}
	}
}

void pop_arr(JOINT &vals, JOINT &arr) {
	// populates arr with vals
	for(int i = 0; i < N; i++) {
		arr[i] = vals[i];
	}
}

void pop_arr(POS &vals, POS &arr) {
	// populates arr with vals
	for(int i = 0; i < N; i++) {
		arr[i] = vals[i];
	}
}

void rmult(RFORM &mat1, RFORM &mat2, RFORM &res) {
	// multiplies two matrices mat1 * mat2 together
	RFORM temp;
	for(int i = 0; i < (N-1); i++) {
		for(int j = 0; j < (N-1); j++) {
			double sum = 0;
			for(int k = 0; k < (N-1); k++) {
				sum += mat1[i][k]*mat2[k][j];
			}
			res[i][j] = sum;
		}
	}
}

void rmult(RFORM &mat, POS &pos, POS &res) {
	// multiples mat and pos together
	POS temp;
	for(int i = 0; i < N-1; i++) {
		POS arr1;
		double sum = 0;
		for(int j = 0; j < N-1; j++) {
			sum += mat[i][j]*pos[j];
		}
		res[i] = sum;
	}
}

void pmult(POS &pos, double val, POS &res) {
	// multiply pos by a val
	for(int i = 0; i < N-1; i++) {
		res[i] = val*pos[i];
	}
}

void padd(POS &pos1, POS &pos2, POS &res) {
	// adds pos1 and pos2 togheter
	for(int i = 0; i < (N-1); i++) {
		res[i] = pos1[i] + pos2[i];
	}
}

void tconst(RFORM &rmat, POS &pos, TFORM &tmat) {
	// constructs the Transformation matrix with rotation matrix and position vector
	TFORM temp = {{rmat[0][0], rmat[0][1], rmat[0][2], pos[0]},
				{rmat[1][0], rmat[1][1], rmat[1][2], pos[1]},
				{rmat[2][0], rmat[2][1], rmat[2][2], pos[2]},
				{0, 0, 0, 1}};
	pop_mat(temp, tmat);
}

void transpose_mat(RFORM &rmat, RFORM &imat) {
	// transposes a rotation matrix

	for(int i = 0; i < N-1; i++) {
		for(int j = 0; j < N-1; j++) {
			imat[i][j] = rmat[j][i];
		}
	}
}

//returns true if no solution is available
bool no_sol(bool p_invalid, bool n_invalid) {
	if (n_invalid && p_invalid) {
		cout << "No solution found within revolute constraints \n";
		return true;
	}
	return false;
}
#include "e488_proj.h"
using namespace std;

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
		printf("Please choose a number: \n 1. Forward Kinematics \n 2. Inverse Kinematics\n 3. MoveToConfiguration\n 4. Grasp\n 5. Ungrasp\n 6. Exit\n");
		printf("Your input >> ");
		cin >> user_input;

		switch(user_input) {
			case 1 : // Forward Kinematics
				FwdKinDeg(joint_vals, spt);
				break;
			case 2 : // Inverse Kinematics 
				InvKin(spt);
				break;
			case 3 : // Move to Config
				SimpleMove();
				break;
			case 4 : // Grasp
				Grasp(true);
				break;
			case 5 : // UnGrasp
				Grasp(false);
				break;
			case 7 : // fwdkin rad
				FwdKinRad(joint_vals, spt);
				break;
			case 6 : // Exit
				return; 
				break;
			default : 
				printf("Invalid input. Please try again.\n\n\n");
				break;
		}		
	}
	
}

// Part 0: Menu Operations
void FwdKinDeg(JOINT &joint_vals, JOINT &spt) {
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
		double theta4_r = DEG2RAD(theta4_d);

		// check joint values
		JOINT temp{theta1_r, theta2_r, d3, theta4_r};
		pop_arr(temp, joint_vals);
		check_joints(joint_vals, valid);

		// determine if we need different joint values
		if(!valid) {
			printf("Invalid inputs! please try again.\n\n\n");
		}
	}
	printf("\nYour inputs: [%f (rads), %f (rads), %f (mm), %f (rads)]\n", joint_vals[0], joint_vals[1], joint_vals[2], joint_vals[3]);
	printf("Your inputs: [%f (deg), %f (deg), %f (mm), %f (deg)]\n\n", RAD2DEG(joint_vals[0]), RAD2DEG(joint_vals[1]), joint_vals[2], RAD2DEG(joint_vals[3]));

	// report position and orientation of the tool (x, y, z, phi)
	WHERE(joint_vals, spt);
	printf("Position and Orientation of Tool Frame (x, y, z, phi):\n");
	printf("(%f, %f, %f, %f)\n", spt[0], spt[1], spt[2], RAD2DEG(spt[3]));
	printf("\n\n");

	return;
}

void FwdKinRad(JOINT &joint_vals, JOINT &spt) {
	printf("\nIn Forward Kin (RAD)!\n");
	double theta1, theta2, d3, theta4;
	
	bool valid = false;
	while(!valid) {
		// asks the user for joint values
		printf("Input joint parameters:\n");
		printf("theta1 (rad): ");
		cin >> theta1;
		printf("theta2 (rad): ");
		cin >> theta2;
		printf("d3 (mm) [-200, -100]: ");
		cin >> d3;
		printf("theta4 (rad): ");
		cin >> theta4;

		// check joint values
		JOINT temp{theta1, theta2, d3, theta4};
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

void InvKin(JOINT &spt) {
	printf("\nIn Inverse Kin!\n");

	// asks user for pose position (x, y, z, phi) of the tool 
	double x, y, z, phi;
	printf("Input position and orientation of tool:\n");
	printf("x (mm): ");
	cin >> x;
	if(x == 666) {
		x = spt[0];
		y = spt[1];
		z = spt[2];
		phi = RAD2DEG(spt[3]);
	}
	else {
		printf("y (mm): ");
		cin >> y;
		printf("z (mm): ");
		cin >> z;
		printf("phi (deg): ");
		cin >> phi;
	}

	printf("Your input: [%f (mm), %f (mm), %f (mm), %f (deg)]\n", x, y, z, phi);
	printf("Solving inverse kinematics...\n\n");

	JOINT t_pos{x, y, z, DEG2RAD(phi)};
	JOINT c_pos;
	JOINT near, far;
	bool sol;
	GetConfiguration(c_pos);
	c_pos[0] = DEG2RAD(c_pos[0]);
	c_pos[1] = DEG2RAD(c_pos[1]);
	c_pos[3] = DEG2RAD(c_pos[3]);
	SOLVE(t_pos, c_pos, near, far, sol);

	if(!sol) {
		// no solution exists!
		printf("No solution exists!\n\n");
		return;
	} 

	printf("phi: %f\n", phi);
	// will probably need to print these as a loop
	printf("FAR solutions: ");
	printf("(%f, %f, %f, %f)\n", RAD2DEG(far[0]), RAD2DEG(far[1]), far[2], RAD2DEG(far[3]));

	// print the closest solution
	printf("\n"); 
	printf("NEARest solution: ");
	printf("(%f, %f, %f, %f)\n", RAD2DEG(near[0]), RAD2DEG(near[1]), near[2], RAD2DEG(near[3]));

	// move to closet solution
	MoveToConfiguration(near);

	printf("\n\n");
	return;
}

void SimpleMove(void) {
	double theta_1, theta_2, translation, phi;
	// moves robot to configuration
	printf("Move robot to configuration (x, y, z, phi):\n");
	printf("first joint angle: ");
	cin >> theta_1;
	printf("second joint angle: ");
	cin >> theta_2;
	printf("vertical translation: ");
	cin >> translation;
	printf("gripper_orientation: ");
	cin >> phi;

	printf("\nMoving robot to (%f, %f, %f, %f)...\n\n", theta_1, theta_2, translation, phi);
	JOINT pos{ theta_1, theta_2, translation, phi };
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

	TFORM temp {{c_phi, -s_phi, 0, x},
				{s_phi, c_phi, 0, y},
				{0, 0, 1, z},
				{0, 0, 0, 1}};
	
	pop_mat(temp, mat);

	return;
}
void UTOI_FLIP(JOINT &pos, TFORM &mat) {
    /*  Description: User form to internal form
        Assume all rotations are about Z axis
        Input:
            pos = (x, y, z, phi)
        Output:
            T = | cos(phi)    sin(phi)       0   x |
                | sin(phi)    -cos(phi)      0   y |
                | 0             0           -1   z |
                | 0             0            0   1 |
    */

    // get the different position values
    double x = pos[0];
    double y = pos[1];
    double z = pos[2];
    double phi = pos[3];

    // calculate parameters for transformation matrix, T
    double s_phi = sin(phi);
    double c_phi = cos(phi);

	TFORM temp {{c_phi, s_phi, 0, x},
				{s_phi, -c_phi, 0, y},
				{0, 0, -1, z},
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
					{0,     0,     -1, L70-(L410 + d3)},
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

	// if (no_sol(p_invalid, n_invalid)) {
	// 	sol = false;
	// 	return;
	// }
	// compute k's for theta2
    double k1_p = L195 + L142*cosf(theta2_p);
    double k2_p = L142*sinf(theta2_p);

    double k1_n = L195 + L142*cosf(theta2_n);
    double k2_n = L142*sinf(theta2_n);

    double alpha11_p = atan2(y,x) - atan2(k2_p, k1_p);
	double theta1_p = alpha11_p;
	// check if it's within joint limit

	// check if sum is in invalid range
	double alpha12_p = abs(abs(alpha11_p) - DEG2RAD(360));
	if(abs(alpha11_p) > DEG2RAD(THETA_CONS_150)) {
		if(alpha12_p < DEG2RAD(A210) && alpha12_p > DEG2RAD(A150)) {
			p_invalid = true;
		}
		else {
			theta1_p = (alpha11_p/abs(alpha11_p))*(abs(alpha11_p) - DEG2RAD(360));
		}
	}

    double alpha11_n = atan2(y,x) - atan2(k2_n, k1_n);
	double theta1_n = alpha11_n;
	double alpha12_n = abs(abs(alpha11_n) - DEG2RAD(360));
	if(abs(alpha11_n) > DEG2RAD(THETA_CONS_150)) {
		if(alpha12_n < DEG2RAD(A210) && alpha12_n > DEG2RAD(A150)) {
			n_invalid = true;
		} 
		else {
			theta1_n = (alpha11_n / abs(alpha11_n))*(abs(alpha11_n) - DEG2RAD(360));
		}
		
	}

	//valid alpha_n, alpha_p value


	// if (no_sol(p_invalid, n_invalid)) {
	// 	sol = false;
	// 	return;
	// }

	// compute theta4
	double phi = atan2(s_phi, c_phi);
    double alpha41_p = theta1_p + theta2_p - phi;
	double theta4_p = alpha41_p;
	double alpha42_p = abs(abs(alpha41_p) - DEG2RAD(360));
	if(abs(alpha41_p) > DEG2RAD(THETA_CONS_150)) {
		if(alpha42_p < DEG2RAD(A210-0.0001) && alpha42_p > DEG2RAD(A150+0.0001)) {
			p_invalid = true;
		}
		else {
			theta4_p = (alpha41_p/abs(alpha41_p))*(abs(alpha41_p) - DEG2RAD(360));
			//theta4_p = alpha41_p;
			if(abs(theta4_p) > DEG2RAD(180)) {
				theta4_p = (theta4_p/abs(theta4_p))*(theta4_p - DEG2RAD(360));
			}
		}
	}

	// if (theta4_p > THETA_CONS_150 && p_invalid == false)
	// 	p_invalid = true;

	

    double alpha41_n = theta1_n + theta2_n - phi;
	double theta4_n = alpha41_n;
	double alpha42_n = abs(abs(alpha41_n) - DEG2RAD(360));
	if(abs(alpha41_n) > DEG2RAD(THETA_CONS_150)) {
		if(alpha42_n < DEG2RAD(A210-0.0001) && alpha42_n > DEG2RAD(A150+0.0001)) {
			p_invalid = true;
		}
		else {
			theta4_n = (alpha41_n/abs(alpha41_n))*(abs(alpha41_n) - DEG2RAD(360));
			// theta4_n = alpha41_n;
			if(abs(theta4_n) > DEG2RAD(180)) {
				theta4_n = (theta4_n/abs(theta4_n))*(theta4_n - DEG2RAD(360));
			}
		}
	}

	// if (theta4_n < -THETA_CONS_150 && n_invalid == false)
	// 	n_invalid = true;

	// if (no_sol(p_invalid, n_invalid)) {
	// 	sol = false;
	// 	return;
	// }
	// compute d3
	double d3 = -z - L410 + L70;	
	// if (d3 < D3LOWER_200 || d3 > D3UPPER_100) {
	// 	sol = false;
	// 	return;
	// }
	sol = true;
	JOINT jp{theta1_p, theta2_p, d3, theta4_p};
	JOINT jn{theta1_n, theta2_n, d3, theta4_n};

	pop_arr(jp, near);
	pop_arr(jn, far);
}

void SOLVE(JOINT &tar_pos, JOINT &curr_pos, JOINT &near, JOINT &far, bool &sol) {
	// given a target position, determines the joint values 
	// what is happening in this function?

	// printf("curr_pos:");
	// print(curr_pos);

	TFORM wrels, wrelb, trels, trelb; // calculated with TMULT
	TFORM srelb, wrelt; // inverses of global transforms
	// get trels
    UTOI_FLIP(tar_pos, trels); //error

	// printf("trels\n");
	// print(trels);

    // get brels
    TINVERT(brels, srelb); //

	// printf("brels:\n");
	// print(brels);

	// printf("srelb:\n");
	// print(srelb);

	TINVERT(trelw, wrelt);

	// printf("trelw:\n");
	// print(trelw);

	// printf("wrelt:\n");
	// print(wrelt);

	TMULT(srelb, trels, trelb);
    // do trelw = srelb * wrels
    TMULT(trelb, wrelt, wrelb);

	// printf("wrelb\n");
	// print(wrelb);

    // find nearest solution with INVKIN
    INVKIN(wrelb, tar_pos, near, far, sol);

	// sum of differences between current and final joint position for a given solution is stored in each index
	// size 2 is hardcoded since we only get two solutions
	float sums[2] = { 0, 0 };
	// temp used for swapping near and far, itr used to iterate over near and far values when computing sums
	JOINT temp, itr;
	pop_arr(near, itr);
	// weights for each joint
	float w[4] = { 1, 1, 1, 1 };
	int M = size(sums);
	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			sums[i] += w[j] * (abs(itr[j] - curr_pos[j]));
		}
		pop_arr(far, itr);
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
#include "e488_proj.h"
using namespace std;

void main(void) {
	// runs UI

	/* Menu Options:
	1. Forward Kinematics
	2. Inverse Kinematics
	3. Stop+Reset Robot
	4. Close Gripper
	5. Move To Config
	6. Exit
	*/

	int user_input;
	bool done;
	bool gripper_status = false;
	JOINT joint_vals, spt;

	UTOI(B, brels);
	UTOI(T, trelw);

	while(true) {
		printf("Please choose a number: \n 1. Forward Kinematics \n 2. Inverse Kinematics\n 3. Stop+Reset Robot\n 4. Toggle Gripper\n 5. MoveToConfiguration\n 6. Exit\n");
		printf("Your input >> ");
		cin >> user_input;

		switch(user_input) {
			case 1 : // Forward Kinematics
				FwdKinDeg(joint_vals, spt);
				break;
			case 2 : // Inverse Kinematics 
				InvKin(spt);
				break;
			case 3 : // Stop Robot
				printf("Stopping and Resetting Robot\n");
				StopRobot();
				ResetRobot();
				break;
			case 4 : // Toggle Gripper
				ToggleGripper(gripper_status);
				break;
			case 5 : // MoveToConfig
				SimpleMove();
				break;
			case 6 : // Exit
				return; 
				break;
			case 7 : // fwdkin rad
				FwdKinRad(joint_vals, spt);
				break;
			default : 
				printf("Invalid input. Please try again.\n\n\n");
				break;
		}		
	}
	
}

////////////////////////////////////////////////////////////////////////////////
//									UI										  //
////////////////////////////////////////////////////////////////////////////////

// Part 0: Menu Operations
void FwdKinDeg(JOINT &joint_vals, JOINT &spt) {
	printf("\nIn Forward Kin!\n");
	double theta1_d, theta2_d, d3, theta4_d;
	
	bool valid = false;
	while(!valid) {
		// asks the user for joint values
		printf("Input joint parameters:\n");
		printf("theta1 (deg) [-%i, %i]: ", THETA1_CONS, THETA1_CONS);
		cin >> theta1_d;
		printf("theta2 (deg) [-%i, %i]: ", THETA2_CONS, THETA2_CONS);
		cin >> theta2_d;
		printf("d3 (mm) [%i, %i]: ", D3LOWER_200, D3UPPER_100);
		cin >> d3;
		printf("theta4 (deg) [-%i, %i]: ", THETA4_CONS, THETA4_CONS);
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
		printf("d3 (mm) [%i, %i]: ", D3LOWER_200, D3UPPER_100);
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
	JOINT curr_joint;
	JOINT near, far;
	bool sol;
	GetConfiguration(curr_joint);
	curr_joint[0] = DEG2RAD(curr_joint[0]);
	curr_joint[1] = DEG2RAD(curr_joint[1]);
	curr_joint[3] = DEG2RAD(curr_joint[3]);
	bool p_val, n_val;
	SOLVE(t_pos, curr_joint, near, far, p_val, n_val);

	if(!p_val && !n_val) {
		// no solution exists!
		printf("No solution exists!\n\n");

		printf("Invalid solutions:\n");
		printf("(%f, %f, %f, %f)\n", RAD2DEG(far[0]), RAD2DEG(far[1]), far[2], RAD2DEG(far[3]));
		printf("(%f, %f, %f, %f)\n\n", RAD2DEG(near[0]), RAD2DEG(near[1]), near[2], RAD2DEG(near[3]));

		return;
	}
	else if(p_val && !n_val){
		// p is valid and n is invalid
		printf("Invalid solution:\n");
		printf("(%f, %f, %f, %f)\n", RAD2DEG(far[0]), RAD2DEG(far[1]), far[2], RAD2DEG(far[3]));

		printf("NEARest solution:\n");
		printf("(%f, %f, %f, %f)\n", RAD2DEG(near[0]), RAD2DEG(near[1]), near[2], RAD2DEG(near[3]));
	}
	else if(!p_val && n_val) {
		// p is invalid and n is valid
		printf("Invalid solution:\n");
		printf("(%f, %f, %f, %f)\n", RAD2DEG(far[0]), RAD2DEG(far[1]), far[2], RAD2DEG(far[3]));

		printf("NEARest solution:\n");
		printf("(%f, %f, %f, %f)\n", RAD2DEG(near[0]), RAD2DEG(near[1]), near[2], RAD2DEG(near[3]));
	}
	else {
		printf("Current position:\n");
		print(curr_joint);

		printf("Far solution:\n");
		printf("(%f, %f, %f, %f)\n", RAD2DEG(far[0]), RAD2DEG(far[1]), far[2], RAD2DEG(far[3]));

		printf("NEARest solution:\n");
		printf("(%f, %f, %f, %f)\n", RAD2DEG(near[0]), RAD2DEG(near[1]), near[2], RAD2DEG(near[3]));
	}

	// move to closet solution
	near[0] = RAD2DEG(near[0]);
	near[1] = RAD2DEG(near[1]);
	near[3] = RAD2DEG(near[3]);
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
void ToggleGripper(bool &status) {
	//Toggle Status of gripper then open/close it
	//Gripper opens when status is false and closes when status is true
	if (status == true) {
		printf("Opening Gripper\n");
		status = false;
	}
	else {
		printf("Closing Gripper\n");
		status = true;
	}
	Grasp(status);
}

////////////////////////////////////////////////////////////////////////////////
//									DEMO 1  								  //
////////////////////////////////////////////////////////////////////////////////


void check_joints(JOINT &joint_vals, bool &valid) {	
	// checks the joint values in [rads] and [mm]
	// interpret input
	double theta1 = joint_vals[0];
	double theta2 = joint_vals[1];
	double d3 = joint_vals[2];
	double theta4 = joint_vals[3];

	// define constraints
	double theta1_cons = DEG2RAD(THETA1_CONS);
	double theta2_cons = DEG2RAD(THETA2_CONS);
	double theta4_cons = DEG2RAD(THETA4_CONS);

	// check theta1 with THETA1_CONS
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
    float phi = atan2(mat[1][0], mat[0][0]);

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

void INVKIN(TFORM &wrelb, JOINT &near, JOINT &far, bool &p_val, bool &n_val) {
	// finds the inverse kinematics for the robot, then compares the solutions with the  to 
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
    double theta2_n = atan2(-s_theta2, c_theta2);

	// compute k's for theta2
    double k1_p = L195 + L142*cosf(theta2_p);
    double k2_p = L142*sinf(theta2_p);

    double k1_n = L195 + L142*cosf(theta2_n);
    double k2_n = L142*sinf(theta2_n);

    double alpha11_p = atan2(y,x) - atan2(k2_p, k1_p);
	double theta1_p = alpha11_p;

	// check if sum is in invalid range
	double alpha12_p = abs(abs(alpha11_p) - DEG2RAD(360));
	if(abs(alpha11_p) > DEG2RAD(THETA1_CONS)) {
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
	if(abs(alpha11_n) > DEG2RAD(THETA1_CONS)) {
		if(alpha12_n < DEG2RAD(A210) && alpha12_n > DEG2RAD(A150)) {
			n_invalid = true;
		} 
		else {
			theta1_n = (alpha11_n / abs(alpha11_n))*(abs(alpha11_n) - DEG2RAD(360));
		}
	}

	// compute theta4
	double phi = atan2(s_phi, c_phi);
    double alpha41_p = theta1_p + theta2_p - phi;
	double theta4_p = alpha41_p;
	double alpha42_p = abs(abs(alpha41_p) - DEG2RAD(360));
	if(abs(alpha41_p) > DEG2RAD(THETA4_CONS)) {
		if(alpha42_p < DEG2RAD(A210-0.0001) && alpha42_p > DEG2RAD(A150+0.0001)) {
			p_invalid = true;
		}
		else {
			theta4_p = (alpha41_p/abs(alpha41_p))*(abs(alpha41_p) - DEG2RAD(360));
			if(abs(theta4_p) > DEG2RAD(180)) {
				theta4_p = (theta4_p/abs(theta4_p))*(theta4_p - DEG2RAD(360));
			}
		}
	}

    double alpha41_n = theta1_n + theta2_n - phi;
	double theta4_n = alpha41_n;
	double alpha42_n = abs(abs(alpha41_n) - DEG2RAD(360));
	if(abs(alpha41_n) > DEG2RAD(THETA4_CONS)) {
		if(alpha42_n < DEG2RAD(A210-0.0001) && alpha42_n > DEG2RAD(A150+0.0001)) {
			p_invalid = true;
		}
		else {
			theta4_n = (alpha41_n/abs(alpha41_n))*(abs(alpha41_n) - DEG2RAD(360));
			if(abs(theta4_n) > DEG2RAD(180)) {
				theta4_n = (theta4_n/abs(theta4_n))*(theta4_n - DEG2RAD(360));
			}
		}
	}

	// compute d3
	double d3 = -z - L410 + L70;	
	
	JOINT jp{theta1_p, theta2_p, d3, theta4_p};
	JOINT jn{theta1_n, theta2_n, d3, theta4_n};

	// check joint values
	check_joints(jp, p_val);
	check_joints(jn, n_val);

	pop_arr(jp, near);
	pop_arr(jn, far);
}

void SOLVE(JOINT &tar_pos, JOINT &curr_joint, JOINT &near, JOINT &far, bool &p_val, bool &n_val) {
	// given a target position, determines the joint values 

	TFORM wrels, wrelb, trels, trelb; // calculated with TMULT
	TFORM srelb, wrelt; // inverses of global transforms
	
	// do matrix multiplications and transformations
    UTOI_FLIP(tar_pos, trels); // get trels
    TINVERT(brels, srelb); // get brels
	TINVERT(trelw, wrelt);
	TMULT(srelb, trels, trelb);
    TMULT(trelb, wrelt, wrelb); // do trelw = srelb * wrels

    // find nearest solution with INVKIN
    INVKIN(wrelb, near, far, p_val, n_val);

	// sum of differences between current and final joint position for a given solution is stored in each index
	// size 2 is hardcoded since we only get two solutions
	// only check if both solutions are valid
	if(p_val && n_val) {
		float sums[2] = { 0, 0 };
		// temp used for swapping near and far, itr used to iterate over near and far values when computing sums
		JOINT temp, itr;
		pop_arr(near, itr);
		// weights for each joint
		float w[4] = { 1, 1, 1, 1 };
		int M = size(sums);
		for (int i = 0; i < M; i++) {
			for (int j = 0; j < N; j++) {
				sums[i] += w[j] * (abs(itr[j] - curr_joint[j]));
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
		return true;
	}
	return false;
}

////////////////////////////////////////////////////////////////////////////////
//									DEMO 2		  							  //
////////////////////////////////////////////////////////////////////////////////
void CUBCOEF(double theta0, double thetaf, double vel0, double velf, double tf, JOINT &coeff) {
	// Calculates the cubic coefficients
	// Assumes constant velocity between points
	// See eqn 7.11 of text (pg 207 of text OR 215/408 of pdfr)
	double a0, a1, a2, a3;
	a0 = theta0;	
	a1 = vel0;
	a2 = (3/pow(tf, 2))*(thetaf - theta0) - (2/tf)*vel0 - (1/tf)*velf;
	a3 = -(2/pow(tf, 3))*(thetaf - theta0) + (1/pow(tf, 2))*(velf + vel0);

	JOINT test{a0, a1, a2, a3};
	pop_arr(test, coeff);
}

void PATHGEN(double t, double vel, TFORM &A, TFORM &B, TFORM &C, TFORM &G) {
	/* 	Computes the path trajectory of a given set of points (max. 5)
		Inputs: 
			- t: total time for motion (user-specified)
			- G, A, B, C: DIFFERENT Tool frames WRT the Station frame

		DEBUG: Need error checking to see if any of the points are out of the workspace
		DEBUG: Not worrying about velocity right now, need to implement later. 
			See options below
			- Options:
				1. 	The user specifies the desired velocity at each via point in 
					terms of a Cartesian linear and angular velocity of the tool 
					frame at that instant
				2. 	The system automatically chooses the velocity at the via points 
					by applying a suitable heuristic in either Cartesian space, 
					or joint space 
				3. 	The system automatically chooses the velocities at the via point 
					in such a way as to cause the acceleration at the via points 
					to be continuous
	*/
	// convert all tool frames to position + orientation of frames (x, y, z, phi)
	JOINT a_pos, b_pos, c_pos, g_pos;	
	ITOU(A, a_pos);
	ITOU(B, b_pos);
	ITOU(C, c_pos);
	ITOU(G, g_pos);

	// get the current joint values of the robotic arm
	JOINT curr_joint;
	GetConfiguration(curr_joint);
	// convert angles to radians 
	curr_joint[0] = DEG2RAD(curr_joint[0]);
	curr_joint[1] = DEG2RAD(curr_joint[1]);
	curr_joint[3] = DEG2RAD(curr_joint[3]);

	// compute inverse kinematics for each position
	// current pos to A
	JOINT a_near, a_far;
	bool ap = false;
	bool an = false;
	SOLVE(a_pos, curr_joint, a_near, a_far, ap, an);

	// A to B
	JOINT b_near, b_far;
	bool bp = false;
	bool bn = false;
	SOLVE(b_pos, a_near, b_near, b_far, bp, bn);

	// B to C
	JOINT c_near, c_far;
	bool cp = false;
	bool cn = false;
	SOLVE(c_pos, b_near, c_near, c_far, cp, cn);

	// C to G
	JOINT g_near, g_far;
	bool gp = false;
	bool gn = false;
	SOLVE(g_pos, c_near, g_near, g_far, gp, gn);

	/*now that we have all of the joint values for positions: A, B, C, G compute 
	the cubic spline interpolation for each of the joints*/

	// but first, separate the all the joint values for one joint to a given array
	JOINT j1, j2, j3, j4;
	get_jv(1, a_near, b_near, c_near, g_near, j1);
	get_jv(2, a_near, b_near, c_near, g_near, j2);
	get_jv(3, a_near, b_near, c_near, g_near, j3);
	get_jv(4, a_near, b_near, c_near, g_near, j4);

	/*now that we have the joint values per joint, compute cubic spline for each
	of the different locations*/

	// cubic coefficients for theta1
	JOINT ab1, bc1, cg1;
	compute_coeff(j1, t, vel, ab1, bc1, cg1);

	// cubic coefficients for theta2
	JOINT ab2, bc2, cg2;
	compute_coeff(j2, t, vel, ab2, bc2, cg2);

	// cubic coefficients for d3
	JOINT ab3, bc3, cg3;
	compute_coeff(j3, t, vel, ab3, bc3, cg3);

	// cubic coefficients for theta4
	//JOINT ga4, ab4, bc4;
	JOINT ab4, bc4, cg4;
	compute_coeff(j4, t, vel, ab4, bc4, cg4);

	// DEBUG: plot trajectories (position, velocity, acceleration) for each of the joints
}

void get_jv(int idx, JOINT &a_joint, JOINT &b_joint, JOINT &c_joint, JOINT& g_joint, JOINT &joint) {
	// gets the joint values for IDX points A, B, C, G. ie. if IDX = 1, gets the 
	// joint1 values and saves it in joint
	int idx_arr = idx - 1;
	double a_val, b_val, c_val, g_val;
	a_val = a_joint[idx_arr];
	b_val = b_joint[idx_arr];
	c_val = c_joint[idx_arr];
	g_val = g_joint[idx_arr];

	JOINT vals{a_val, b_val, c_val, g_val};
	pop_arr(vals, joint);
}

void compute_coeff(JOINT &j, double t, double vel, JOINT &ab, JOINT &bc, JOINT &cg) {
	// takes the joint values, and computes the cubic coefficients between subsequent 
	// joint values
	// Assumes A -> B -> C -> G
	double t1 = 0;
	double t2 = 0;
	double t3 = 0;

	// A -> B
	CUBCOEF(j[0], j[1], 0, vel, t1, ab);

	// B -> C
	CUBCOEF(j[1], j[2], vel, vel, t2, bc);

	// C -> G
	CUBCOEF(j[2], j[3], vel, 0, t3, cg);
}
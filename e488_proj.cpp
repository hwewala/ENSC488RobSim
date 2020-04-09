#include "e488_proj.h"
using namespace std;

void main(void) {
	// runs UI

	/* Menu Options:
	1. Forward Kinematics
	2. Inverse Kinematics
	3. Trajectory Planning (x, y, z, phi)
	4. Trajectory Planning (theta1, theta2, d3, theta4)
	5. Stop+Reset Robot
	6. Close Gripper
	7. Move To Config
	8. Exit
	9. Custom 
	*/

	int user_input;
	bool done;
	bool gripper_status = false;
	JOINT joint_vals, spt;
	vector<vector<double>> traj_vals;

	UTOI(B, brels);
	UTOI(T, trelw);

	while(true) {
		printf("\nPlease choose a number:\n"
				" 1. Forward Kinematics\n"
				" 2. Inverse Kinematics\n"
				" 3. Trajectory Planning (x, y, z, phi)\n"
				" 4. Trajectory Planning (joint values)\n"
				" 5. Stop+Reset Robot\n"
				" 6. Toggle Gripper\n"
				" 7. MoveToConfiguration\n"
				" 8. Trajectory Planning (custom)\n"
				" 9. Execute Path"
				"\n 0. Exit\n");
		printf("Your input >> ");
		cin >> user_input;

		switch(user_input) {
			case 1 : // Forward Kinematics
				FwdKinDeg(joint_vals, spt);
				break;
			case 2 : // Inverse Kinematics 
				InvKin(spt);
				break;
			case 3 : // Trajectory Planner (x, y, z, phi)
				traj_vals.clear();
				TrajPlanPos(traj_vals);
				break;
			case 4 : // Trajectory Planner (joint vals)
				traj_vals.clear();
				TrajPlanJoint(traj_vals);
				break;
			case 5 : // Stop Robot
				printf("Stopping and Resetting Robot\n");
				StopRobot();
				ResetRobot();
				break;
			case 6 : // Toggle Gripper
				ToggleGripper(gripper_status);
				break;
			case 7 : // MoveToConfig
				SimpleMove();
				//Move();
				break;
			case 8 : // Custom
				traj_vals.clear();
				TrajCust(traj_vals);
				break;
			case 9 : // Move with Path
				ExecutePath(traj_vals);
				break;
			case 0 : // Exit
				return; 
				break;
			default : 
				printf("Invalid input. Please try again.\n\n\n");
				break;
		}		
	}
	
}

////////////////////////////////////////////////////////////////////////////////
//									User Interface							  //
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
	if(x == FK) {
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
	printf("Move robot to configuration (theta1, theta2, d3, theta4):\n");
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

void Move(void) {
	double theta1, theta2, d3, theta4, vel, acc;
	printf("Move robot to configuration with vel and acc\n");
	printf("j1: ");
	cin >> theta1;
	printf("j2: ");
	cin >> theta2;
	printf("j3: ");
	cin >> d3;
	printf("j4: ");
	cin >> theta4;
	printf("vel: ");
	cin >> vel;
	printf("acc: ");
	cin >> acc;

	JOINT pos{ theta1, theta2, d3, theta4 };
	JOINT vels{ vel, vel, vel, vel };
	JOINT accs{ acc, acc, acc, acc };

	MoveWithConfVelAcc(pos, vels, accs);

	int time = 1000 * 1;
	std::this_thread::sleep_for(std::chrono::milliseconds(time));

	StopRobot();
	ResetRobot();
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

void TrajPlanPos(vector<vector<double>>& traj_vals) {
	// Asks the user for a total time, velocity (???), set of points
	// DEBUG: 	for now, just get the user to input positions (x, y, z, phi) for 
	//			the different frames: A, B, C, G
	printf("Trajectory Planning w/ (x, y, z, phi)\n");

	// get total time of manipulator motion
	double t;
	printf("Time for motion (s): ");
	cin >> t;

	//// get velocity between via points
	//double vel;
	//printf("Velocity between via points: ");
	//cin >> vel;

	// get set of points
	double xa, ya, za, phia;
	printf("\nInput (x, y, z, phi) values for A\n");
	printf("x (mm): ");
	cin >> xa;
	printf("y (mm): ");
	cin >> ya;
	printf("z (mm): ");
	cin >> za;
	printf("phi (deg): ");
	cin >> phia;
	if(xa == FK || ya == FK || za == FK || phia == FK) return;
	JOINT a_pos{xa, ya, za, DEG2RAD(phia)};
		
	double xb, yb, zb, phib;
	printf("\nInput (x, y, z, phi) values for B\n");
	printf("x (mm): ");
	cin >> xb;
	printf("y (mm): ");
	cin >> yb;
	printf("z (mm): ");
	cin >> zb;
	printf("phi (deg): ");
	cin >> phib;
	if(xb == FK || yb == FK || zb == FK || phib == FK) return;
	JOINT b_pos{xb, yb, zb, DEG2RAD(phib)};

	double xc, yc, zc, phic;
	printf("\nInput (x, y, z, phi) values for C\n");
	printf("x (mm): ");
	cin >> xc;
	printf("y (mm): ");
	cin >> yc;
	printf("z (mm): ");
	cin >> zc;
	printf("phi (deg): ");
	cin >> phic;
	if(xc == FK || yc == FK || zc == FK || phic == FK) return;
	JOINT c_pos{xc, yc, zc, DEG2RAD(phic)};

	double xg, yg, zg, phig;
	printf("\nInput (x, y, z, phi) values for G\n");
	printf("x (mm): ");
	cin >> xg;
	printf("y (mm): ");
	cin >> yg;
	printf("z (mm): ");
	cin >> zg;
	printf("phi (deg): ");
	cin >> phig;
	if(xg == FK || yg == FK || zg == FK || phig == FK) return;
	JOINT g_pos{xg, yg, zg, DEG2RAD(phig)};

	// Convert the sets of points to TRELS: A, B, C, G
	TFORM a_mat, b_mat, c_mat, g_mat;
	UTOI_FLIP(a_pos, a_mat);
	UTOI_FLIP(b_pos, b_mat);
	UTOI_FLIP(c_pos, c_mat);
	UTOI_FLIP(g_pos, g_mat);

	// Plan the path
	printf("Planning the Trajectory!\n");
	traj_vals = PATHPLAN(t, a_mat, b_mat, c_mat, g_mat, true);
}

void TrajPlanJoint(vector<vector<double>>& traj_vals) {
	// Trajectory Planning with input of joint values
	printf("Trajectory Planning w/ (theta1, theta2, d3, theta4)\n");

	// get total time of manipulator motion
	double t;
	printf("Time for motion (s): ");
	cin >> t;

	//// get velocity between via points
	//double vel;
	//printf("Velocity between via points: ");
	//cin >> vel;

	double j1a, j2a, j3a, j4a;
	printf("\nA:\n");
	printf("theta1 (deg) [-%i, %i]: ", THETA1_CONS, THETA1_CONS);
	cin >> j1a;
	printf("theta2 (deg) [-%i, %i]: ", THETA2_CONS, THETA2_CONS);
	cin >> j2a;
	printf("d3 (mm) [%i, %i]: ", D3LOWER_200, D3UPPER_100);
	cin >> j3a;
	printf("theta4 (deg) [-%i, %i]: ", THETA4_CONS, THETA4_CONS);
	cin >> j4a;
	JOINT ja{DEG2RAD(j1a), DEG2RAD(j2a), j3a, DEG2RAD(j4a)};
	bool a_valid = false;
	check_joints(ja, a_valid);
	if(!a_valid) return;

	double j1b, j2b, j3b, j4b;
	printf("\nB:\n");
	printf("theta1 (deg) [-%i, %i]: ", THETA1_CONS, THETA1_CONS);
	cin >> j1b;
	printf("theta2 (deg) [-%i, %i]: ", THETA2_CONS, THETA2_CONS);
	cin >> j2b;
	printf("d3 (mm) [%i, %i]: ", D3LOWER_200, D3UPPER_100);
	cin >> j3b;
	printf("theta4 (deg) [-%i, %i]: ", THETA4_CONS, THETA4_CONS);
	cin >> j4b;
	JOINT jb{DEG2RAD(j1b), DEG2RAD(j2b), j3b, DEG2RAD(j4b)};
	bool b_valid = false;
	check_joints(jb, b_valid);
	if(!b_valid) return;

	double j1c, j2c, j3c, j4c;
	printf("\nC:\n");
	printf("theta1 (deg) [-%i, %i]: ", THETA1_CONS, THETA1_CONS);
	cin >> j1c;
	printf("theta2 (deg) [-%i, %i]: ", THETA2_CONS, THETA2_CONS);
	cin >> j2c;
	printf("d3 (mm) [%i, %i]: ", D3LOWER_200, D3UPPER_100);
	cin >> j3c;
	printf("theta4 (deg) [-%i, %i]: ", THETA4_CONS, THETA4_CONS);
	cin >> j4c;
	JOINT jc{DEG2RAD(j1c), DEG2RAD(j2c), j3c, DEG2RAD(j4c)};
	bool c_valid = false;
	check_joints(jc, c_valid);
	if(!c_valid) return;

	double j1g, j2g, j3g, j4g;
	printf("\nG:\n");
	printf("theta1 (deg) [-%i, %i]: ", THETA1_CONS, THETA1_CONS);
	cin >> j1g;
	printf("theta2 (deg) [-%i, %i]: ", THETA2_CONS, THETA2_CONS);
	cin >> j2g;
	printf("d3 (mm) [%i, %i]: ", D3LOWER_200, D3UPPER_100);
	cin >> j3g;
	printf("theta4 (deg) [-%i, %i]: ", THETA4_CONS, THETA4_CONS);
	cin >> j4g;
	JOINT jg{DEG2RAD(j1g), DEG2RAD(j2g), j3g, DEG2RAD(j4g)};
	bool g_valid = false;
	check_joints(jg, g_valid);
	if(!g_valid) return;

	JOINT a_pos, b_pos, c_pos, g_pos;
	WHERE(ja, a_pos);
	WHERE(jb, b_pos);
	WHERE(jc, c_pos);
	WHERE(jg, g_pos);

	TFORM a_mat, b_mat, c_mat, g_mat;
	UTOI_FLIP(a_pos, a_mat);
	UTOI_FLIP(b_pos, b_mat);
	UTOI_FLIP(c_pos, c_mat);
	UTOI_FLIP(g_pos, g_mat);

	printf("\nPlanning the Trajectory!\n");
	traj_vals = PATHPLAN(t, a_mat, b_mat, c_mat, g_mat, true);
}

void TrajCust(vector<vector<double>>& traj_vals) {
	// Trajectory Planning with input of joint values
	printf("Trajectory Planning w/ (theta1, theta2, d3, theta4)\n");

	// get total time of manipulator motion
	double t = 12;
	printf("Time for motion (s): %f\n", t);

	// get velocity between via points
	/*double vel = 5;
	printf("Velocity between via points: %f\n", vel);*/

	double j1a, j2a, j3a, j4a;
	j1a = 100;
	j2a = 10;
	j3a = -101;
	j4a = -100;
	JOINT ja{DEG2RAD(j1a), DEG2RAD(j2a), j3a, DEG2RAD(j4a)};
	bool a_valid = false;
	check_joints(ja, a_valid);
	if(!a_valid) return;

	double j1b, j2b, j3b, j4b;
	j1b = 90;
	j2b = -90;
	j3b = -199;
	j4b = 100;
	JOINT jb{DEG2RAD(j1b), DEG2RAD(j2b), j3b, DEG2RAD(j4b)};
	bool b_valid = false;
	check_joints(jb, b_valid);
	if(!b_valid) return;

	double j1c, j2c, j3c, j4c;
	j1c = 135;
	j2c = 90;
	j3c = -101;
	j4c = -100;
	JOINT jc{DEG2RAD(j1c), DEG2RAD(j2c), j3c, DEG2RAD(j4c)};
	bool c_valid = false;
	check_joints(jc, c_valid);
	if(!c_valid) return;

	double j1g, j2g, j3g, j4g;
	j1g = 45;
	j2g = -90;
	j3g = -199;
	j4g = 100;
	JOINT jg{DEG2RAD(j1g), DEG2RAD(j2g), j3g, DEG2RAD(j4g)};
	bool g_valid = false;
	check_joints(jg, g_valid);
	if(!g_valid) return;

	JOINT a_pos, b_pos, c_pos, g_pos;
	WHERE(ja, a_pos);
	WHERE(jb, b_pos);
	WHERE(jc, c_pos);
	WHERE(jg, g_pos);

	TFORM a_mat, b_mat, c_mat, g_mat;
	UTOI_FLIP(a_pos, a_mat);
	UTOI_FLIP(b_pos, b_mat);
	UTOI_FLIP(c_pos, c_mat);
	UTOI_FLIP(g_pos, g_mat);

	printf("\nPlanning the Trajectory!\n");
	traj_vals = PATHPLAN(t, a_mat, b_mat, c_mat, g_mat, true);
}

void ExecutePath(vector<vector<double>> traj_vals) {
	// get the trajectory values into their own vector
	vector<double> time;
	vector<double> j1_pos, j2_pos, j3_pos, j4_pos;
	vector<double> j1_vel, j2_vel, j3_vel, j4_vel;
	vector<double> j1_acc, j2_acc, j3_acc, j4_acc;

	time = traj_vals[0];
	j1_pos = traj_vals[1];
	j2_pos = traj_vals[2];
	j3_pos = traj_vals[3];
	j4_pos = traj_vals[4];
	j1_vel = traj_vals[5];
	j2_vel = traj_vals[6];
	j3_vel = traj_vals[7];
	j4_vel = traj_vals[8];
	j1_acc = traj_vals[9];
	j2_acc = traj_vals[10];
	j3_acc = traj_vals[11];
	j4_acc = traj_vals[12];

	// get the time increments to send commands
	int len = j1_pos.size();
	double max_time = time[len-1];
	int inc = 1000*(max_time)/len;

	printf("Moving joints to (%f, %f, %f, %f)\n", RAD2DEG(j1_pos[len - 1]), RAD2DEG(j2_pos[len - 1]), j3_pos[len - 1], RAD2DEG(j4_pos[len - 1]));

	for(int i = 0; i < len; i++) {
		// convert values to appropriate values		

		JOINT conf{ RAD2DEG(j1_pos[i]), RAD2DEG(j2_pos[i]), j3_pos[i], RAD2DEG(j4_pos[i]) };
		JOINT vel{ RAD2DEG(j1_vel[i]), RAD2DEG(j2_vel[i]), j3_vel[i], RAD2DEG(j4_vel[i]) };
		JOINT acc{ RAD2DEG(j1_acc[i]), RAD2DEG(j2_acc[i]), j3_acc[i], RAD2DEG(j4_acc[i]) };

		MoveWithConfVelAcc(conf, vel, acc);
		//printf("Pos #%d: (%f, %f, %f, %f)\n", i + 1, RAD2DEG(j1_pos[i]), RAD2DEG(j2_pos[i]), j3_pos[i], RAD2DEG(j4_pos[i]));
		//sleep for inc amount of time
		std::this_thread::sleep_for(std::chrono::milliseconds(inc));
		continue;
	}
	StopRobot();
	ResetRobot();
	return;
}
////////////////////////////////////////////////////////////////////////////////
//								Check Limits							  	  //
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

void check_vel(vector<vector<double>> vals, bool &valid) {
	// Checks to see if velocity limits are exceeded
	// separate different velocities for each joint
	vector<double> j1, j2, j3, j4;
	j1 = vals[0];
	j2 = vals[1];
	j3 = vals[2];
	j4 = vals[3];

	int sz = size(j1);
	for(int i = 0; i < sz; i++) {
		bool j1_val = abs(j1[i]) <= J1V_LIM;
		bool j2_val = abs(j2[i]) <= J2V_LIM;
		bool j3_val = abs(j3[i]) <= J3V_LIM;
		bool j4_val = abs(j4[i]) <= J4V_LIM;

		valid = j1_val && j2_val && j3_val && j4_val;
		if (!valid){
			return;
		}
	}
	valid = true;
}

void check_acc(vector<vector<double>> vals, bool &valid) {
	// Checks to see if acceleration limits are exceeded
	// separate different accelerations for each joint
	vector<double> j1, j2, j3, j4;
	j1 = vals[0];
	j2 = vals[1];
	j3 = vals[2];
	j4 = vals[3];

	int sz = size(j1);
	for(int i = 0; i < sz; i++) {
		bool j1_val = abs(j1[i]) <= J1A_LIM;
		bool j2_val = abs(j2[i]) <= J2A_LIM;
		bool j3_val = abs(j3[i]) <= J3A_LIM;
		bool j4_val = abs(j4[i]) <= J4A_LIM;

		valid = j1_val && j2_val && j3_val && j4_val;
		if (!valid){
			return;
		}
	}
	valid = true;
}


////////////////////////////////////////////////////////////////////////////////
//									DEMO 1  								  //
////////////////////////////////////////////////////////////////////////////////

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
    /*  Use this for TRELS Transform matrices!
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
	ARR3 apb, bpc, apc;
	// get the rotation matrices from brela and crelb
	get_r(brela, arb);
	get_r(crelb, brc);

	// multiply rotation matrices: crela = brela*crelb
	rmult(arb, brc, arc);
	
	// get the position vector from brela and crelb
	get_pos(brela, apb);
	get_pos(crelb, bpc);

	// multiply rotation matrix with position vector, get apc
	ARR3 temp;
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
	ARR3 pos1;
	get_pos(tmat, pos1); //apb

	// transforms the rotation matrix (ie. the inverse of rotation matrix)
	RFORM trans_mat;
	transpose_mat(rmat, trans_mat); //bra

	ARR3 pos2, pos3;
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
    for(int i = 0; i < 4; i++) {
        printf("%f", arr[i]);
        if(i < 3) {
            printf("\t");
        }
    }
    printf("]\n");
	return;
}

void print(ARR3 &arr) {
	// Description: prints the (size = 3)
    printf("[");
    for(int i = 0; i < 3; i++) {
        printf("%f", arr[i]);
        if(i < 2) {
            printf("\t");
        }
    }
    printf("]\n");
	return;
}

void print(ARR5 &arr) {
	// Description: prints the (size = 5)
    printf("[");
    for(int i = 0; i < 5; i++) {
        printf("%f", arr[i]);
        if(i < 4) {
            printf("\t");
        }
    }
    printf("]\n");
	return;
}

void print(ARR6& arr) {
	// Description: prints the (size = 5)
	int n = 6;
	printf("[");
	for (int i = 0; i < n; i++) {
		printf("%f", arr[i]);
		if (i < n-1) {
			printf("\t");
		}
	}
	printf("]\n");
	return;
}

void print(vector<double>& vec) {
	//prints a vector
	printf("[");
	for (int i = 0; i < vec.size(); i++) {
		printf("%lf,", vec.at(i));
		if (i < vec.size() - 1)
			printf("\t");
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

void get_pos(TFORM &tmat, ARR3 &pos) {
	// gets the Position vector from the Transformation matrix
	for(int i = 0; i < (N-1); i++) {
		pos[i] = tmat[i][(N-1)];
	}
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
	for(int i = 0; i < 4; i++) {
		arr[i] = vals[i];
	}
}

void pop_arr(ARR5 &vals, ARR5 &arr) {
	// populates arr with vals
	for(int i = 0; i < 5; i++) {
		arr[i] = vals[i];
	}
}

void pop_arr(ARR3 &vals, ARR3 &arr) {
	// populates arr with vals
	for(int i = 0; i < 3; i++) {
		arr[i] = vals[i];
	}
}

void pop_arr(ARR6& vals, ARR6& arr) {
	// populates arr with vals
	for (int i = 0; i < 6; i++) {
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

void rmult(RFORM &mat, ARR3 &pos, ARR3 &res) {
	// multiples mat and pos together
	ARR3 temp;
	for(int i = 0; i < N-1; i++) {
		ARR3 arr1;
		double sum = 0;
		for(int j = 0; j < N-1; j++) {
			sum += mat[i][j]*pos[j];
		}
		res[i] = sum;
	}
}

void pmult(ARR3 &pos, double val, ARR3 &res) {
	// multiply pos by a val
	for(int i = 0; i < N-1; i++) {
		res[i] = val*pos[i];
	}
}

void padd(ARR3 &pos1, ARR3 &pos2, ARR3 &res) {
	// adds pos1 and pos2 togheter
	for(int i = 0; i < (N-1); i++) {
		res[i] = pos1[i] + pos2[i];
	}
}

void tconst(RFORM &rmat, ARR3 &pos, TFORM &tmat) {
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

// Make a CSV file with one or more columns of values
// Each column of data is represented by the pair <column name, column data>
//   as pair<string, vector<double>>
// The dataset is represented as a vector of these columns
// Note that all columns should be the same size
// https://www.gormanalysis.com/blog/reading-and-writing-csv-files-with-cpp/
void write_csv(string filename, vector<pair<string, vector<double>>> dataset) {
	ofstream myFile(filename);
	// send column names to the stream
	for (int j = 0; j < dataset.size(); ++j) {
		myFile << dataset.at(j).first;
		if (j != dataset.size() - 1) myFile << ",";
	}
	myFile << "\n";
	// send data to the stream
	for (int i = 0; i < dataset.at(0).second.size(); ++i)
	{
		for (int j = 0; j < dataset.size(); ++j)
		{
			myFile << dataset.at(j).second.at(i);
			if (j != dataset.size() - 1) myFile << ","; // No comma at end of line
		}
		myFile << "\n";
	}
	// Close the file
	myFile.close();
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

void QUINCOEF(double pos0, double posf, double vel0, double velf, double acc0, double accf, double tf, ARR6& coeff) {
	// Calculates the quartic coefficients
	double a0, a1, a2, a3, a4, a5;
	a0 = pos0;
	a1 = vel0;
	a2 = acc0 / 2;
	a3 = (20 * posf - 20 * pos0 - (8 * velf + 12 * vel0) * tf - (3 * acc0 - accf) * pow(tf, 2)) / (2*pow(tf,3));
	a4 = (30 * pos0 - 30 * posf + (14 * velf + 16 * vel0) * tf + (3 * acc0 - 2 * accf) * pow(tf, 2)) / (2 * pow(tf, 4));
	a5 = (12 * posf - 12 * pos0 - (6 * velf + 6 * vel0) * tf - (acc0 - accf) * pow(tf, 2)) / (2 * pow(tf, 5));

	ARR6 test{ a0, a1, a2, a3, a4, a5 };
	pop_arr(test, coeff);
}

void compute_times(double& t, ARR5& j1, ARR5& j2, ARR5& j3, ARR5& j4, ARR5& times) {
	// computes the times at each point based on the displacement for all the joints
	// convert arrays to a matrix

	double t_inc = t / 4;
	ARR5 temp{ t_inc * 0, t_inc * 1, t_inc * 2, t_inc * 3, t_inc * 4 };
	times[0] = temp[0];
	times[1] = temp[1];
	times[2] = temp[2];
	times[3] = temp[3];
	times[4] = temp[4];

	//vector<vector<double>> vals;
	//vector<double> temp1, temp2, temp3, temp4;
	//to_vec(j1, temp1);
	//vals.push_back(temp1);
	//to_vec(j2, temp2);
	//vals.push_back(temp2);
	//to_vec(j3, temp3);
	//vals.push_back(temp3);
	//to_vec(j4, temp4);
	//vals.push_back(temp4);
	//ARR4 lims{ 2*(double)J1V_LIM, 2*(double)J2V_LIM, J3V_LIM, 2*(double)J4V_LIM };

	//// for each segment, find the joint that's moving the most WRT it's joint limit
	//int num_joints = 4;
	//int num_segs = 4;
	//// loop through joints
	//vector<vector<double>> ratios;
	//vector<vector<double>> distances;
	//for (int i = 0; i < num_joints; i++) {
	//	// loop through time segments
	//	vector<double> joint_rat;
	//	vector<double> joint_dist;
	//	for (int j = 0; j < num_segs; j++) {
	//		// compute the displacement ratio 
	//		double dist = abs(vals[i][j + 1] - vals[i][j]);
	//		joint_dist.push_back(dist);
	//		joint_rat.push_back(dist / lims[i]);
	//	}
	//	ratios.push_back(joint_rat);
	//	distances.push_back(joint_dist);
	//}
	//
	//// loop through the joint ratios and find the max ratio 
	//vector<int> max_idx;
	//for (int i = 0; i < num_segs; i++) {
	//	// create the vector that we want to compare values
	//	vector<double> curr_vec;
	//	for (int j = 0; j < num_joints; j++) {
	//		curr_vec.push_back(ratios[j][i]);
	//	}			
	//	int idx; 
	//	find_max(curr_vec, idx);
	//	max_idx.push_back(idx);
	//}

	//// Compute the minimum amount of time without going over the maximum velocity 
	//vector<double> min_time;
	//double sum_time = 0;
	//for(int i = 0; i < num_segs; i++) {
	//	double time = distances[max_idx[i]][i] / (lims[max_idx[i]] + 1);
	//	min_time.push_back(time);
	//	sum_time += time;
	//}

	//// Check if we need to change the time
	//double time_ratio = 0;
	//if(sum_time >= t) {
	//	// minimum time is greater than the specified time
	//	t = sum_time; // adding a +1 to keep things safe

	//	// set the time ratio
	//	time_ratio = 1;
	//} 
	//else {
	//	// minimum time is less than the specified time 
	//	// calculate the ratio to increase each time segment 
	//	time_ratio = t/sum_time;
	//}

	//// Compute the time segment for each via point
	//vector<double> time_segs;
	//for(int i = 0; i < num_segs; i++) {
	//	time_segs.push_back(time_ratio * min_time[i]);
	//}

	//// check if velocity limit is met


	//// Compute the time that's used for the robot
	//times[0] = 0;
	//for(int i = 1; i < num_segs+1; i++) {
	//	times[i] = times[i-1] + time_segs[i-1];
	//}	
	
	return;
}

void compute_coeff(ARR5& j, ARR5& times, JOINT& curra, JOINT& ab, JOINT& bc, JOINT& cg) {
	// takes the joint values, and computes the cubic coefficients between subsequent 
	// joint values
	// Assumes Curr -> A -> B -> C -> G
	// compute the slopes of each line segment
	ARR4 slopes;
	ARR4 t_seg{times[1] - times[0], times[2] - times[1], times[3] - times[2], times[4] - times[3]};
	compute_slopes(times, j, slopes);

	// the velocities at the via points, vel at start and end equal 0
	ARR5 vels;
	compute_vels(slopes, vels);

	// curr -> A
	CUBCOEF(j[0], j[1], vels[0], vels[1], t_seg[0], curra);

	// A -> B
	CUBCOEF(j[1], j[2], vels[1], vels[2], t_seg[1], ab);

	// B -> C
	CUBCOEF(j[2], j[3], vels[2], vels[3], t_seg[2], bc);

	// C -> G
	CUBCOEF(j[3], j[4], vels[3], vels[4], t_seg[3], cg);
}

void compute_slopes(ARR5 &t, ARR5 &j, ARR4 &slopes) {
	// computes the slope of the line segment
	int sz = sizeof(slopes)/sizeof(*slopes);
	for(int i = 0; i < sz; i++) {
		slopes[i] = (j[i+1] - j[i]) / (t[i+1] - t[i]);
	}
}

void compute_vels(ARR4 &slopes, ARR5 &vels) {
	/* computes the velocity at each via point
		- if slope's sign change, vel = 0
		- if slope's sign does not change, vel = average of two slopes */
	ARR3 temp1;
	int sz = sizeof(temp1) / sizeof(*temp1);
	for (int i = 0; i < sz; i++) {
		if (slopes[i] > 0 && slopes[i + 1] < 0 || slopes[i] < 0 && slopes[i + 1] > 0) {
			temp1[i] = 0;
		}
		else {
			temp1[i] = (slopes[i] + slopes[i + 1]) / 2;
		}
	}
	ARR5 temp2{ 0, temp1[0], temp1[1],  temp1[2], 0 };
	pop_arr(temp2, vels);
}

void compute_accs(ARR4& slopes, ARR5& accs) {
	compute_vels(slopes, accs);
}

void get_jv(int idx, JOINT &curr_joint, JOINT &a_joint, JOINT &b_joint, JOINT &c_joint, JOINT& g_joint, ARR5 &joint) {
	// gets the joint values for IDX points A, B, C, G. ie. if IDX = 1, gets the 
	// joint1 values and saves it in joint
	int idx_arr = idx - 1;
	double curr_val, a_val, b_val, c_val, g_val;
	curr_val = curr_joint[idx_arr];
	a_val = a_joint[idx_arr];
	b_val = b_joint[idx_arr];
	c_val = c_joint[idx_arr];
	g_val = g_joint[idx_arr];

	ARR5 vals{curr_val, a_val, b_val, c_val, g_val};
	pop_arr(vals, joint);
}

vector<vector<double>> PATHPLAN(double t, TFORM &A, TFORM &B, TFORM &C, TFORM &G, bool debug) {
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
	vector<vector<double>> return_vals;
	JOINT a_pos, b_pos, c_pos, g_pos;	
	ITOU(A, a_pos);
	ITOU(B, b_pos);
	ITOU(C, c_pos);
	ITOU(G, g_pos);

	if (debug) {
		printf("\n--Input positions (x, y, z, phi)--\n");
		printf("A: ");
		print(a_pos);
		printf("B: ");
		print(b_pos);
		printf("C: ");
		print(c_pos);
		printf("G: ");
		print(g_pos);
	}

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
	if(!ap && !an) {
		printf("Error: A is out of workspace!\n");
		return return_vals;
	}

	// A to B
	JOINT b_near, b_far;
	bool bp = false;
	bool bn = false;
	SOLVE(b_pos, a_near, b_near, b_far, bp, bn);
	if(!bp && !bn) {
		printf("Error: B is out of workspace!\n");
		return return_vals;
	}

	// B to C
	JOINT c_near, c_far;
	bool cp = false;
	bool cn = false;
	SOLVE(c_pos, b_near, c_near, c_far, cp, cn);
	if(!cp && !cn) {
		printf("Error: C is out of workspace!\n");
		return return_vals;
	}

	// C to G
	JOINT g_near, g_far;
	bool gp = false;
	bool gn = false;
	SOLVE(g_pos, c_near, g_near, g_far, gp, gn);
	if(!gp && !gn) {
		printf("Error: G is out of workspace!\n");
		return return_vals;
	}

	if(debug) {
		printf("\n--Joint Values (theta1, theta2, d3, theta4)--\n");
		printf("curr:   ");
		print(curr_joint);
		printf("Soln A: ");
		print(a_near);
		printf("Soln B: ");
		print(b_near);
		printf("Soln C: ");
		print(c_near);
		printf("Soln G: ");
		print(g_near);
	}

	/*now that we have all of the joint values for positions: A, B, C, G compute 
	the cubic spline interpolation for each of the joints*/

	// but first, separate the all the joint values for one joint to a given array
	ARR5 j1, j2, j3, j4;
	get_jv(1, curr_joint, a_near, b_near, c_near, g_near, j1);
	get_jv(2, curr_joint, a_near, b_near, c_near, g_near, j2);
	get_jv(3, curr_joint, a_near, b_near, c_near, g_near, j3);
	get_jv(4, curr_joint, a_near, b_near, c_near, g_near, j4);

	if(debug) {
		printf("\n--Joint Values for each joint--\n");
		printf("theta1 (rads):\t");
		print(j1);
		printf("theta2 (rads):\t");
		print(j2);
		printf("d3 (mm):\t");
		print(j3);
		printf("theta4 (rads):\t");
		print(j4);
	}

	/*now that we have the joint values per joint, compute cubic spline for each
	of the different locations*/
	// compute times
	ARR5 times;
	compute_times(t, j1, j2, j3, j4, times);

	if (debug) {
		printf("time segments:  ");
		print(times);
	}

	//// cubic coefficients for theta1
	JOINT curra1, ab1, bc1, cg1;
	compute_coeff(j1, times, curra1, ab1, bc1, cg1);

	// cubic coefficients for theta2
	JOINT curra2, ab2, bc2, cg2;
	compute_coeff(j2, times, curra2, ab2, bc2, cg2);

	// cubic coefficients for d3
	JOINT curra3, ab3, bc3, cg3;
	compute_coeff(j3, times, curra3, ab3, bc3, cg3);

	// cubic coefficients for theta4
	JOINT curra4, ab4, bc4, cg4;
	compute_coeff(j4, times, curra4, ab4, bc4, cg4);


	// cubic coefficients for theta1
	//ARR6 curra1, ab1, bc1, cg1;
	//compute_quincoeff(j1, times, curra1, ab1, bc1, cg1);

	//// cubic coefficients for theta2
	//ARR6 curra2, ab2, bc2, cg2;
	//compute_quincoeff(j2, times, curra2, ab2, bc2, cg2);

	//// cubic coefficients for d3
	//ARR6 curra3, ab3, bc3, cg3;
	//compute_quincoeff(j3, times, curra3, ab3, bc3, cg3);

	//// cubic coefficients for theta4
	//ARR6 curra4, ab4, bc4, cg4;
	//compute_quincoeff(j4, times, curra4, ab4, bc4, cg4);

	if(debug) {
		printf("\n--Cubic Coefficients (a0, a1, a2, a3)--\n");
		printf("theta1:\n");
		printf("0 -> A: ");
		print(curra1);
		printf("A -> B: ");
		print(ab1);
		printf("B -> C: ");
		print(bc1);
		printf("C -> G: ");
		print(cg1);
		printf("theta2:\n");
		printf("0 -> A: ");
		print(curra2);
		printf("A -> B: ");
		print(ab2);
		printf("B -> C: ");
		print(bc1);
		printf("C -> G: ");
		print(cg1);
		printf("d3:\n");
		printf("0 -> A: ");
		print(curra3);
		printf("A -> B: ");
		print(ab3);
		printf("B -> C: ");
		print(bc1);
		printf("C -> G: ");
		print(cg1);
		printf("theta4:\n");
		printf("0 -> A: ");
		print(curra4);
		printf("A -> B: ");
		print(ab4);
		printf("B -> C: ");
		print(bc1);
		printf("C -> G: ");
		print(cg1);
	}

	// Plot the desired points on the plot
	vector<double> t_interval;
	vector<double> t1_joints;
	to_vec(times, t_interval);
	to_vec(j1, t1_joints);
	vector<pair<string, vector<double>>> theta1_points {{"Time", t_interval}, {"Target", t1_joints}};
	write_csv("Target.csv", theta1_points);
	
	// POSITION VALUES
	vector<double> theta1_pos, theta2_pos, d3_pos, theta4_pos, curr_time;
	int sample_rate = 500;
	// Theta 1
	PATHGEN(times[0], times[1], sample_rate, curra1, theta1_pos, curr_time);
	PATHGEN(times[1], times[2], sample_rate, ab1, theta1_pos, curr_time);
	PATHGEN(times[2], times[3], sample_rate, bc1, theta1_pos, curr_time);
	PATHGEN(times[3], times[4], sample_rate, cg1, theta1_pos, curr_time);
	bool curr_time_filled = true; // vector of times is filled after first runthrough

	// Theta 2
	PATHGEN(times[0], times[1], sample_rate, curra2, theta2_pos, curr_time, curr_time_filled);
	PATHGEN(times[1], times[2], sample_rate, ab2, theta2_pos, curr_time, curr_time_filled);
	PATHGEN(times[2], times[3], sample_rate, bc2, theta2_pos, curr_time, curr_time_filled);
	PATHGEN(times[3], times[4], sample_rate, cg2, theta2_pos, curr_time, curr_time_filled);
	// d3
	PATHGEN(times[0], times[1], sample_rate, curra3, d3_pos, curr_time, curr_time_filled);
	PATHGEN(times[1], times[2], sample_rate, ab3, d3_pos, curr_time, curr_time_filled);
	PATHGEN(times[2], times[3], sample_rate, bc3, d3_pos, curr_time, curr_time_filled);
	PATHGEN(times[3], times[4], sample_rate, cg3, d3_pos, curr_time, curr_time_filled);
	// Theta 4
	PATHGEN(times[0], times[1], sample_rate, curra4, theta4_pos, curr_time, curr_time_filled);
	PATHGEN(times[1], times[2], sample_rate, ab4, theta4_pos, curr_time, curr_time_filled);
	PATHGEN(times[2], times[3], sample_rate, bc4, theta4_pos, curr_time, curr_time_filled);
	PATHGEN(times[3], times[4], sample_rate, cg4, theta4_pos, curr_time, curr_time_filled);
	vector<pair<string, vector<double>>> pos_vals = { {"Time", curr_time}, {"theta1_pos", theta1_pos}, {"theta2_pos", theta2_pos}, {"d3_pos", d3_pos}, {"theta4_pos", theta4_pos} };
	write_csv("Position.csv", pos_vals);

	// VELOCTIY VALUES
	vector<double> theta1_vel, theta2_vel, d3_vel, theta4_vel;
	// Theta 1
	VELGEN(times[0], times[1], sample_rate, curra1, theta1_vel, curr_time, curr_time_filled);
	VELGEN(times[1], times[2], sample_rate, ab1, theta1_vel, curr_time, curr_time_filled);
	VELGEN(times[2], times[3], sample_rate, bc1, theta1_vel, curr_time, curr_time_filled);
	VELGEN(times[3], times[4], sample_rate, cg1, theta1_vel, curr_time, curr_time_filled);
	// Theta 2
	VELGEN(times[0], times[1], sample_rate, curra2, theta2_vel, curr_time, curr_time_filled);
	VELGEN(times[1], times[2], sample_rate, ab2, theta2_vel, curr_time, curr_time_filled);
	VELGEN(times[2], times[3], sample_rate, bc2, theta2_vel, curr_time, curr_time_filled);
	VELGEN(times[3], times[4], sample_rate, cg2, theta2_vel, curr_time, curr_time_filled);
	// d3
	VELGEN(times[0], times[1], sample_rate, curra3, d3_vel, curr_time, curr_time_filled);
	VELGEN(times[1], times[2], sample_rate, ab3, d3_vel, curr_time, curr_time_filled);
	VELGEN(times[2], times[3], sample_rate, bc3, d3_vel, curr_time, curr_time_filled);
	VELGEN(times[3], times[4], sample_rate, cg3, d3_vel, curr_time, curr_time_filled);
	// Theta 4
	VELGEN(times[0], times[1], sample_rate, curra4, theta4_vel, curr_time, curr_time_filled);
	VELGEN(times[1], times[2], sample_rate, ab4, theta4_vel, curr_time, curr_time_filled);
	VELGEN(times[2], times[3], sample_rate, bc4, theta4_vel, curr_time, curr_time_filled);
	VELGEN(times[3], times[4], sample_rate, cg4, theta4_vel, curr_time, curr_time_filled);
	vector<pair<string, vector<double>>> vel_vals = { {"Time", curr_time}, {"theta1_vel", theta1_vel}, {"theta2_vel", theta2_vel}, {"d3_vel", d3_vel}, {"theta4_vel", theta4_vel} };
	write_csv("Velocity.csv", vel_vals);

	// ACCELERATION VALUES
	vector<double> theta1_acc, theta2_acc, d3_acc, theta4_acc;
	// Theta 1
	ACCGEN(times[0], times[1], sample_rate, curra1, theta1_acc, curr_time, curr_time_filled);
	ACCGEN(times[1], times[2], sample_rate, ab1, theta1_acc, curr_time, curr_time_filled);
	ACCGEN(times[2], times[3], sample_rate, bc1, theta1_acc, curr_time, curr_time_filled);
	ACCGEN(times[3], times[4], sample_rate, cg1, theta1_acc, curr_time, curr_time_filled);
	// Theta 2
	ACCGEN(times[0], times[1], sample_rate, curra2, theta2_acc, curr_time, curr_time_filled);
	ACCGEN(times[1], times[2], sample_rate, ab2, theta2_acc, curr_time, curr_time_filled);
	ACCGEN(times[2], times[3], sample_rate, bc2, theta2_acc, curr_time, curr_time_filled);
	ACCGEN(times[3], times[4], sample_rate, cg2, theta2_acc, curr_time, curr_time_filled);
	// Theta 3
	ACCGEN(times[0], times[1], sample_rate, curra3, d3_acc, curr_time, curr_time_filled);
	ACCGEN(times[1], times[2], sample_rate, ab3, d3_acc, curr_time, curr_time_filled);
	ACCGEN(times[2], times[3], sample_rate, bc3, d3_acc, curr_time, curr_time_filled);
	ACCGEN(times[3], times[4], sample_rate, cg3, d3_acc, curr_time, curr_time_filled);
	// Theta 4
	ACCGEN(times[0], times[1], sample_rate, curra4, theta4_acc, curr_time, curr_time_filled);
	ACCGEN(times[1], times[2], sample_rate, ab4, theta4_acc, curr_time, curr_time_filled);
	ACCGEN(times[2], times[3], sample_rate, bc4, theta4_acc, curr_time, curr_time_filled);
	ACCGEN(times[3], times[4], sample_rate, cg4, theta4_acc, curr_time, curr_time_filled);
	vector<pair<string, vector<double>>> acc_vals = { {"Time", curr_time}, {"theta1_acc", theta1_acc}, {"theta2_acc", theta2_acc}, {"d3_acc", d3_acc}, {"theta4_acc", theta4_acc} };
	write_csv("Acceleration.csv", acc_vals);

	// X vs Y Values
	vector<double> x, y, z, phi, x_targ, y_targ;
	POSGEN(theta1_pos, theta2_pos, d3_pos, theta4_pos, x, y, z, phi);
	vector<pair<string, vector<double>>> xy_vals = { {"X", x}, {"Y", y}, {"Z", z}, {"Phi", phi}};
	write_csv("XY.csv", xy_vals);
	// Get the target X and Y values for all (4) via points
	x_targ.push_back(a_pos[0]);
	x_targ.push_back(b_pos[0]);
	x_targ.push_back(c_pos[0]);
	x_targ.push_back(g_pos[0]);

	y_targ.push_back(a_pos[1]);
	y_targ.push_back(b_pos[1]);
	y_targ.push_back(c_pos[1]);
	y_targ.push_back(g_pos[1]);

	vector<pair<string, vector<double>>> xy_targ_vals = { {"X_targ", x_targ}, {"Y_targ", y_targ} };
	write_csv("XY_targ.csv", xy_targ_vals);

	// Check Velocity and Acceleration limits
	vector<vector<double>> vel_check{theta1_vel, theta2_vel, d3_vel, theta4_vel};
	bool vel_valid = false;
	check_vel(vel_check, vel_valid);
	if (!vel_valid) {
		printf("\n!-----WARNING-----!\nExceeded Velocity limits!\n");
	}

	vector<vector<double>> acc_check{theta1_acc, theta2_acc, d3_acc, theta4_acc};
	bool acc_valid = false;
	check_acc(acc_check, acc_valid);
	if (!acc_valid) {
		printf("\n!-----WARNING-----!\nExceeded Acceleration limits!\n");
	}

	// return one big vector
	return_vals.push_back(curr_time);
	return_vals.push_back(theta1_pos);
	return_vals.push_back(theta2_pos);
	return_vals.push_back(d3_pos);
	return_vals.push_back(theta4_pos);
	return_vals.push_back(theta1_vel);
	return_vals.push_back(theta2_vel);
	return_vals.push_back(d3_vel);
	return_vals.push_back(theta4_vel);
	return_vals.push_back(theta1_acc);
	return_vals.push_back(theta2_acc);
	return_vals.push_back(d3_acc);
	return_vals.push_back(theta4_acc);

	return return_vals;
}

//void PATHGEN(double ti, double tf, int sample_rate, ARR6 &coeff, vector<double> &pos, vector<double> &curr_time, bool isFull) {
void PATHGEN(double ti, double tf, int sample_rate, JOINT & coeff, vector<double> &pos, vector<double> & curr_time, bool isFull) {
	/* Computes the position of the path, theta(t)
		Input: 
			- ti: initial time for cubic spline
			- tf: final time for cubic spline
			- sample_rate: how frequent we want to compute values
			- coeff: the cubic spline coefficients 
			- theta: position values passed by reference (y axis)
			- curr_time: time values passed by reference (x axis)
			- isFull: To stop curr_time vector from filling, default value False
		Output:
			- pos (y, t): the output positions
	*/
	double t = tf - ti;
	int num_points = (t*sample_rate) + 1; // + 2 to get the final position as well
	for(double i = 0; i < num_points; i++) {
		if (isFull == false) curr_time.push_back(ti + i/sample_rate);
		pos.push_back(coeff[0] + coeff[1] * curr_time[i] + coeff[2] * pow(curr_time[i], 2) + coeff[3] * pow(curr_time[i], 3));
	}
}

//void VELGEN(double ti, double tf, int sample_rate, ARR6 &coeff, vector<double>& vel, vector<double>& curr_time, bool isFull) {
void VELGEN(double ti, double tf, int sample_rate, JOINT& coeff, vector<double>& vel, vector<double>& curr_time, bool isFull) {
	// Computes the velocity of the path vs. time
	// Units: rad/s
	double t = tf - ti;
	int num_points = (t * sample_rate) + 1; // + 2 to get the final position as well
	for(int i = 0; i < num_points; i++) {
		if (isFull == false) curr_time.push_back(ti + i / sample_rate);
		vel.push_back(coeff[1] + 2 * coeff[2] * curr_time[i] + 3 * coeff[3] * pow(curr_time[i], 2));
		//vel.push_back(coeff[1] + 2*coeff[2]*curr_time[i] + 3*coeff[3]*pow(curr_time[i], 2) + 4*coeff[4]*pow(curr_time[i], 3) + 5*coeff[5]*pow(curr_time[i], 4));
	}

}

//void ACCGEN(double ti, double tf, int sample_rate, ARR6 &coeff, vector<double>& acc, vector<double>& curr_time, bool isFull) {
void ACCGEN(double ti, double tf, int sample_rate, JOINT& coeff, vector<double>& acc, vector<double>& curr_time, bool isFull) {
	// Computes the acceleration of the path vs. time
	// Units: rad/s^2
	double t = tf - ti;
	int num_points = (t * sample_rate) + 1; // + 2 to get the final position as well
	for(int i = 0; i < num_points; i++) {
		if (isFull == false) curr_time.push_back(ti + i / sample_rate);
		acc.push_back(2 * coeff[2] + 6 * coeff[3] * curr_time[i]);
		//acc.push_back(2*coeff[2] +	6*coeff[3]*curr_time[i] + 12*coeff[4]*pow(curr_time[i], 2) + 20*coeff[5]*pow(curr_time[i], 3));
	}
}

void POSGEN(vector<double> theta1, vector<double> theta2, vector<double> &d3, vector<double> theta4, 
	vector<double> &x, vector<double> &y, vector<double> &z, vector<double> &phi) {
	
	// Computes the X vs Y and returns it
	int sz = size(theta1);
	for(int i = 0; i < sz; i++) {
		JOINT curr_joint{theta1[i], theta2[i], d3[i], theta4[i]};
		JOINT curr_pos;

		// Use WHERE to get the (x, y, z, phi) of TrelS
		WHERE(curr_joint, curr_pos);

		// Save position to the outputs
		x.push_back(curr_pos[0]);
		y.push_back(curr_pos[1]);
		z.push_back(curr_pos[2]);
		phi.push_back(curr_pos[3]);
	}
}

void to_vec(ARR5 &arr, vector<double> &vec) {
	int sz = sizeof(arr)/sizeof(*arr);
	for(int i = 0; i < sz; i++) {
		vec.push_back(arr[i]);
	}
}

void to_arr(vector<double>& vec, ARR4& arr) {
	int sz = size(vec);
	for (int i = 0; i < sz; i++) {
		arr[i] = vec[i];
	}
}

void find_max(vector<double>& vec, int& idx) {
	int len = vec.size();
	int biggest = 0;
	for (int i = 0; i < len; i++) {
		if (vec[i] > biggest) {
			biggest = vec[i];
			idx = i;
		}
	}
}

void find_min(vector<double>& vec, int& idx) {
	int len = vec.size();
	int smallest = 0;
	for (int i = 0; i < len; i++) {
		if (vec[i] < smallest) {
			smallest = vec[i];
			idx = i;
		}
	}
}
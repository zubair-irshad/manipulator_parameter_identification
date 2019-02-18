// 00-singlearm


// #include "../../common/initModules.h"
//initModules stuff moved here
#include <unistd.h> //for usleep thing
#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <string>
#include <time.h>
#include <sys/time.h>

#include <fstream>
#define SAMPLES 900

using namespace std;

#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION
#define CURRENT SOMATIC__MOTOR_PARAM__MOTOR_CURRENT



char outputDir[1024];
bool train;

// Intialize variables
somatic_d_t daemon_cx;
somatic_motor_t singlearm;



//bool start = true;
const int dof = 7;
double qInit[7] = {-0.187, 0.085, -0.09, -0.014, -0.053, 0.187, 0.066};


double wf = 0.606608708122;//0.414118417633;

//double wf = 0.558048373585;
double amp = 0.8;

double currTime = 0.0;
double prevTime = 0.0;
double duration = 0.0;
double currentTime = 0.0;
clock_t startTime;
clock_t st;
double prev_time;
double iteration_time;




Eigen::Matrix<double, 7, 4> a, b;
double qref [7];
double dqref [7];
double curr [7]  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double integral[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double curLow[7] = {-9.5, -9.5, -7.5, -7.5, -5.5, -5.5, -5.5};
double curHigh[7] = {9.5, 9.5, 7.5, 7.5, 5.5, 5.5, 5.5};
// Eigen::Matrix<double, dof, dof> mKp; 
// Eigen::Matrix<double, dof, dof> mKd;
// Eigen::Matrix<double, dof, dof> mKi;

double mKp[7] = {12.0, 10.0, 6.0, 5.0, 4.5, 4.0, 4.0};
double mKd[7] = {2.0, 2.0, 1.5, 1.5, 1.2, 1.2, 1.2};
double mKi[7] = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001};


double prior_time = 0.0;
double CURR[2500];
int ROW = 0;


struct timeval tv;
//double t;
long start_time;
unsigned long get_time();



//double qDOT[7] = {0, 0, 0, 0, 0, 0, 0};
// double dqref [8] = 	{0.0000,    0.0000,    0.0000,    0.0000,    0.0000,   	0.0000,    0.0000,    0.0000};

ofstream dataQ;
ofstream dataDotQ;
ofstream dataCur;
ofstream dataTimeStamp;
ofstream dataQref;
ofstream dataDQref;
ofstream dataDDotQ;
ofstream dataError;
ifstream readCur;
ofstream sentCur;
//ifstream dataPos;






/* ********************************************************************************************* */
void controlArm(){

	// Get currTime by diff with startTime
	//Time used for storing data

	currentTime = (get_time() - start_time)/1000.0;
	cout<<"-------------------------------"<<endl;
	cout<<"current time "<<currentTime<<endl;
	cout<<"-------------------------------"<<endl;

	//position and veloctity trajectory
	for (int i = 0; i < dof; i++) {
		qref[i] = 0.0;
		dqref[i] = 0.0;
	}

	// Update qref value
	for (int joint = 0; joint < dof; joint++) {
	    for (int l = 1; l <= 4; l++) {

	    	qref[joint] = qref[joint] + (a(joint, l-1)/(wf*l))*sin(wf*l*currentTime)
                - (b(joint, l-1)/(wf*l))*cos(wf*l*currentTime);

            dqref[joint] = dqref[joint] + a(joint, l-1)*cos(wf*l*currentTime)
                + b(joint, l-1)*sin(wf*l*currentTime);

        }
	}

	// //dump data
	somatic_motor_update(&daemon_cx, &singlearm);
	for(int i=0; i<6; i++) { dataQ << singlearm.pos[i] <<  " "; }  dataQ << singlearm.pos[6] << endl;
	for(int i=0; i<6; i++) { dataDotQ << singlearm.vel[i] <<  " "; } dataDotQ << singlearm.vel[6] << endl;
	for(int i=0; i<6; i++) { dataCur << singlearm.cur[i] <<  " "; } dataCur << singlearm.cur[6] << endl;
	for(int i=0; i<6; i++) { dataQref << qref[i] <<  " "; } dataQref << qref[6] << endl;
	for(int i=0; i<6; i++) { dataDQref << dqref[i] <<  " "; } dataDQref << dqref[6] << endl;
	dataTimeStamp << currentTime << endl;

	//PID controller
	iteration_time = currentTime - prior_time;
	for(int i=0; i<7; i++) {

		integral[i] = integral[i] + ((qref[i] - singlearm.pos[i])*iteration_time);
		double u = 2*mKp[i]*(qref[i] - singlearm.pos[i]) + mKd[i]*(dqref[i] - singlearm.vel[i]) + 0*mKi[i]*integral[i];
		curr[i] = std::max(curLow[i], std::min(curHigh[i], u));
		cout << "current " << i+1 << " " << curr[i] << endl;
	}


	somatic_motor_cmd(&daemon_cx, &singlearm, CURRENT, curr, 7, NULL);
	
	prior_time = currentTime;
	
	return;
}

unsigned long get_time(){
	gettimeofday(&tv, NULL);
	unsigned long ret = tv.tv_usec;
	ret /= 1000;
	ret += (tv.tv_sec *1000);
	return ret;

}

/* ********************************************************************************************* */
/// Initializes the arm
static void initArm (somatic_d_t& daemon_cx, somatic_motor_t& arm, const char* armName) {	

	// Get the channel names
	char cmd_name [16], state_name [16];
	sprintf(cmd_name, "%s-cmd", armName);
	sprintf(state_name, "%s-state", armName);

	// Initialize the arm with the daemon context, channel names and # of motors
	somatic_motor_init(&daemon_cx, &arm, 7, cmd_name, state_name);
	usleep(1e5);

	// Set the min/max values for valid and limits values
	double** limits [] = {
		&arm.pos_valid_min, &arm.vel_valid_min, 
		&arm.pos_limit_min, &arm.pos_limit_min, 
		&arm.pos_valid_max, &arm.vel_valid_max, 
		&arm.pos_limit_max, &arm.pos_limit_max};
	for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, 7);
	for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, 7);


	// Update and reset them
	somatic_motor_update(&daemon_cx, &arm);
	somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
	usleep(1e5);
}

/* ********************************************************************************************* */
void init () {
	
	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt));
	dopt.ident = "00-singlearm";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the arm
	initArm(daemon_cx, singlearm, "singlearm");

	if(train) {
		a << -0.009, -0.36, 0.311, -0.362,
		0.095, -0.132, -0.363, 0.474,
		-0.418, -0.25, -0.12, 0.119,
		0.023, 0.113, 0.497, 0.213,
		-0.23, -0.237, 0.153, -0.147,
		0.366, 0.366, 0.302, -0.373,
		-0.247, -0.166, 0.315, 0.031;

		b <<  -0.051, 0.027, 0.003, -0.332,
		-0.292, 0.358, -0.056, -0.436,
		-0.355, 0.039, -0.397, -0.445,
		0.328, 0.256, -0.36, 0.143,
		0.428, 0.093, 0.035, -0.28,
		-0.39, -0.085, 0.388, 0.46,
		-0.046, 0.135, -0.428, 0.387;

	}
    else{

		a <<  -0.06, -0.108, 0.205, -0.258,
		0.092, -0.086, -0.286, 0.086,
		0.386, 0.386, 0.386, -0.043,
		0.296, 0.262, 0.062, -0.043,
		0.386, 0.262, 0.062, 0.062,
		0.368, 0.186, 0.262, 0.186,
		-0.262, -0.386, 0.28, 0.28;


		b <<  -0.031, 0.05, 0.03, 0.108,
		-0.298, 0.286, -0.086, 0.386,
		0.386, 0.154, 0.08, 0.08,
		0.231, 0.372, 0.367, 0.162,
		0.386, -0.043, -0.043, 0.262,
		0.319, -0.031, 0.262, -0.043,
		0.386, -0.043, 0.22, 0.18;
    }

	// Set initial position
	somatic_motor_cmd(&daemon_cx, &singlearm, POSITION, qInit, 7, NULL);

	usleep(4e6);

}

/* ********************************************************************************************* */
// Continuously process the data and set commands to the modules
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	start_time = get_time();
	while(!somatic_sig_received) {

		controlArm();
	
		// Free buffers allocated during this cycle
		aa_mem_region_release(&daemon_cx.memreg);	
		usleep(1e4);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
void destroy() {

	// Halt the Schunk modules
	somatic_motor_cmd(&daemon_cx, &singlearm, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);

	cout << "Destroyed daemons and halted modules" << endl;
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {

	// assert((argc != 2) && "need the target path for saving data");

	if(argc != 2) {
		cout << "Usage: ./currentControl <test|train>" << endl;
		return 0;
	}
	else if(!strcmp(argv[1], "train")) train = true;
	else if(!strcmp(argv[1], "test")) train = false;
	else {
		cout << "Usage: ./currentControl <test|train>" << endl;
		return 0;
	}

	if(train) {
		dataQ.open("../trainData/dataQ.txt");
		dataDotQ.open("../trainData/dataDotQ_NF.txt");
		dataCur.open("../trainData/dataCur.txt");
		dataTimeStamp.open("../trainData/dataTimeStamp.txt");
		dataQref.open("../trainData/dataQref.txt");
		dataDQref.open("../trainData/dataDQref.txt");
	}
	else{
		dataQ.open("../testData/dataQ.txt");
		dataDotQ.open("../testData/dataDotQ_NF.txt");
		dataCur.open("../testData/dataCur.txt");
		dataTimeStamp.open("../testData/dataTimeStamp.txt");
		dataQref.open("../testData/dataQref.txt");
		dataDQref.open("../testData/dataDQref.txt");
	}

	init();

	run();

	dataQ.close();
	dataDotQ.close();
	dataCur.close();
	dataTimeStamp.close();
	dataQref.close();
	dataDQref.close();

	destroy();

	return 0;
}
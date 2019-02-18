// Author: Akash Patel (apatel435@gatech.edu)
// Modified by: Zubair

// genPhiMatrixAsFile
// Purpose: Determine phi vectors for each input data point
//   This phi will be used for finding beta (weights of actual robot)
//
// Input: Ideal beta={mi, MXi, MYi, ...}, krang urdf model, perturbation value,
//   data points (q/poses) as a file,
// Output: Phi matrix as a file

// Overall Input: Data Points q, qdot, qddot (1 x 7) format
// Overall Output: Phi Matrix
// Intermediary Input/Output Flow:
// Input Data Points -> Phi Matrix

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>
#include <math.h>

// Namespaces
using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::utils;

bool train;

// Defines
#define MAXBUFSIZE ((int) 1e6)

// Function Prototypes
// // Generate Phi Matrix
Eigen::MatrixXd genPhiMatrix(Eigen::MatrixXd inputQ, Eigen::MatrixXd inputQdot, Eigen::MatrixXd inputQdotdot, Eigen::MatrixXd inputTorque, string fullRobotPath, double perturbedValue);

// // Read file as matrix
//Eigen::MatrixXd readInputFileAsMatrix(string inputFilename);
Eigen::MatrixXd readInputFileAsMatrix(string inputFilename);

//krang's first commit
// Main Method
int main(int argc, char* argv[]) {

	// assert((argc != 2) && "need the target path for saving data");

	if(argc != 2) {
		cout << "Usage: ./Get_phi_ArmDynamics <test|train>" << endl;
		return 0;
	}
	else if(!strcmp(argv[1], "train")) train = true;
	else if(!strcmp(argv[1], "test")) train = false;
	else {
		cout << "Usage: ./Get_phi_ArmDynamics <test|train>" << endl;
		return 0;
	}

	string inputQFilename, inputQdotFilename, inputQdotdotFilename, inputTorqueFilename;

	if(train) {
	    inputQFilename = "/home/krang/Documents/DART/29-ArmDataCollection/trainData/dataQ.txt";
	    inputQdotFilename = "/home/krang/Documents/DART/29-ArmDataCollection/trainData/dataDotQ.txt";
	    inputQdotdotFilename = "/home/krang/Documents/DART/29-ArmDataCollection/trainData/dataDDotQ.txt";
	    inputTorqueFilename = "/home/krang/Documents/DART/29-ArmDataCollection/trainData/dataCur.txt";
	}
	else{
	    inputQFilename = "/home/krang/Documents/DART/29-ArmDataCollection/testData/dataQ.txt";
    	inputQdotFilename = "/home/krang/Documents/DART/29-ArmDataCollection/testData/dataDotQ.txt";
    	inputQdotdotFilename = "/home/krang/Documents/DART/29-ArmDataCollection/testData/dataDDotQ.txt";
    	inputTorqueFilename = "/home/krang/Documents/DART/29-ArmDataCollection/testData/dataCur.txt";
	}

    // INPUT on below line (perturbation value for finding phi)
    double perturbedValue = std::pow(10, -8);

    // INPUT on below line (absolute robot path)
    string fullRobotPath = "/home/krang/Documents/DART/09-URDF/7DOFArm/singlearm.urdf";

    // INPUT on below line (data point limit)
    try {
        cout << "Reading input q ...\n";
        Eigen::MatrixXd inputQtemp = readInputFileAsMatrix(inputQFilename);
        Eigen::MatrixXd inputQ = inputQtemp.block(1, 0, inputQtemp.rows()-1, inputQtemp.cols());
        cout << "|-> Done\n";

        cout << "Reading input qdot ...\n";
        Eigen::MatrixXd inputQdottemp = readInputFileAsMatrix(inputQdotFilename);
        Eigen::MatrixXd inputQdot = inputQdottemp.block(1, 0, inputQdottemp.rows()-1, inputQdottemp.cols());
        cout << "|-> Done\n";

        cout << "Reading input qdotdot ...\n";
        Eigen::MatrixXd inputQdotdot = readInputFileAsMatrix(inputQdotdotFilename);
        cout << "|-> Done\n";

        cout << "Reading input torque ...\n";
        Eigen::MatrixXd inputTorquetemp = readInputFileAsMatrix(inputTorqueFilename);
        Eigen::MatrixXd inputTorque = inputTorquetemp.block(1, 0, inputTorquetemp.rows()-1, inputTorquetemp.cols());
        cout << "|-> Done\n";

        Eigen::MatrixXd phiMatrix = genPhiMatrix(inputQ, inputQdot, inputQdotdot, inputTorque, fullRobotPath, perturbedValue);

    } catch (exception& e) {
        cout << e.what() << endl;
        return EXIT_FAILURE;
    }




}

// // Generate Phi Matrix
Eigen::MatrixXd genPhiMatrix(Eigen::MatrixXd allInitq, Eigen::MatrixXd allInitqdot, Eigen::MatrixXd allInitqdotdot, Eigen::MatrixXd allInitTorque, string fullRobotPath, double perturbedValue) 
{
    
    int numDataPoints = allInitq.rows();
    cout << "NUM DATA POINTS: " << numDataPoints << endl;
    int numJoints = 7;

    // ********************* Get Torques from Currents
    Eigen::VectorXd km(7);
    km(0)=31.4e-3;
    km(1)=31.4e-3;
    km(2)=38e-3;
    km(3)=38e-3;
    km(4)=16e-3;
    km(5)=16e-3;
    km(6)=16e-3;

    Eigen::VectorXd G_R(7);
    G_R(0)=596;
    G_R(1)=596;
    G_R(2)=625;
    G_R(3)=625;
    G_R(4)=552;
    G_R(5)=552;
    G_R(6)=552;

    ofstream torqueFile;
    torqueFile.open (train?"/home/krang/Documents/DART/20-ParametricIdentification-7DOF/trainOutput/dataTorque.txt":"/home/krang/Documents/DART/20-ParametricIdentification-7DOF/testOutput/dataTorque.txt");
    for(int b=0; b<numJoints; b++) {
		allInitTorque.col(b)= allInitTorque.col(b)*km(b)*G_R(b);
	}
	torqueFile << allInitTorque;
	torqueFile.close();

	// ******************************* Read the beta vector
    cout << "Creating ideal beta vector ...\n";
    dart::utils::DartLoader loader;
    dart::dynamics::SkeletonPtr idealRobot = loader.parseSkeleton("/home/krang/Documents/DART/09-URDF/7DOFArm/singlearm.urdf");
    idealRobot->setGravity(Eigen::Vector3d (0.0, -9.81, 0.0));
    int bodyParams = 10;
    int numBodies = idealRobot->getNumBodyNodes();
    dart::dynamics::BodyNodePtr bodyi;
    string namei;
    double mi;
    double xi, xMi;
    double yi, yMi;
    double zi, zMi;
    double ixx;
    double ixy;
    double ixz;
    double iyy;
    double iyz;
    double izz;

    double cm_x;
    double cm_y;
    double cm_z;


    int numPertRobots = (numBodies-1)*bodyParams;
    Eigen::MatrixXd betaParams(1, numPertRobots);

    for (int i = 1; i < numBodies; i++) {
        bodyi = idealRobot->getBodyNode(i);

        namei = bodyi->getName();
        mi = bodyi->getMass();
        cm_x = bodyi->getLocalCOM()(0);
        cm_y = bodyi->getLocalCOM()(1);
        cm_z = bodyi->getLocalCOM()(2);
        bodyi->getMomentOfInertia (ixx, iyy,izz,ixy,ixz,iyz);


        betaParams(0, (i-1) * bodyParams + 0) = mi;
        betaParams(0, (i-1) * bodyParams + 1) = mi*cm_x;
        betaParams(0, (i-1) * bodyParams + 2) = mi*cm_y;
        betaParams(0, (i-1) * bodyParams + 3) = mi*cm_z;
        betaParams(0, (i-1)*  bodyParams + 4) = ixx + mi * (pow(cm_y,2) + pow(cm_z,2));
        betaParams(0, (i-1)*  bodyParams +5)  = iyy + mi * (pow(cm_x,2) + pow(cm_z,2));
        betaParams(0, (i-1)* bodyParams +6)   = izz + mi * (pow(cm_z,2) + pow(cm_y,2));
        betaParams(0, (i-1)* bodyParams +7)   = ixy - (mi*cm_x*cm_y);
        betaParams(0, (i-1)*bodyParams +8)    = ixz - (mi*cm_x*cm_z);
        betaParams(0, (i-1)*bodyParams +9)    = iyz - (mi*cm_y*cm_z); 
	}
    cout << "|-> Done\n";
    cout << "Creating robot array ...\n";

    ofstream betafile;
    betafile.open (train?"/home/krang/Documents/DART/20-ParametricIdentification-7DOF/trainOutput/betaparameters.txt":"/home/krang/Documents/DART/20-ParametricIdentification-7DOF/testOutput/betaparameters.txt");
    betafile<< betaParams.transpose()<<endl;
    betafile.close();


    // ************************************* Create Perturbed Robot Arrays
    cout << "Creating robot array ...\n";
    // Load robots into fwdPertRobotArray and revPertRobotArray
    dart::dynamics::SkeletonPtr fwdPertRobotArray[numPertRobots];
    dart::dynamics::SkeletonPtr revPertRobotArray[numPertRobots];
    for(int i=0; i<numPertRobots; i++) {
        fwdPertRobotArray[i] = idealRobot->cloneSkeleton();
        revPertRobotArray[i] = idealRobot->cloneSkeleton();
    }

    // Perturb all Beta values in the forward direction
    for(int i=1; i<numBodies; i++) { 
        fwdPertRobotArray[(i-1)*bodyParams + 0]->getBodyNode(i)->setMass(mi + perturbedValue);
        fwdPertRobotArray[(i-1)*bodyParams + 1]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi + perturbedValue, yi, zi));
        fwdPertRobotArray[(i-1)*bodyParams + 2]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi, yi + perturbedValue, zi));
        fwdPertRobotArray[(i-1)*bodyParams + 3]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi, yi, zi + perturbedValue));
        fwdPertRobotArray[(i-1)*bodyParams + 4]->getBodyNode(i)->setMomentOfInertia(ixx + perturbedValue, iyy, izz, ixy, ixz, iyz);
        fwdPertRobotArray[(i-1)*bodyParams + 5]->getBodyNode(i)->setMomentOfInertia(ixx, iyy + perturbedValue, izz, ixy, ixz, iyz);
        fwdPertRobotArray[(i-1)*bodyParams + 6]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz + perturbedValue, ixy, ixz, iyz);
        fwdPertRobotArray[(i-1)*bodyParams + 7]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz, ixy + perturbedValue, ixz, iyz);
        fwdPertRobotArray[(i-1)*bodyParams + 8]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz, ixy, ixz + perturbedValue, iyz);
        fwdPertRobotArray[(i-1)*bodyParams + 9]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz + perturbedValue);  
    }

	// Perturb all Beta values in the reverse direction
    for(int i=1; i<numBodies; i++) {
        revPertRobotArray[(i-1)*bodyParams + 0]->getBodyNode(i)->setMass(mi - perturbedValue);
        revPertRobotArray[(i-1)*bodyParams + 1]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi -  perturbedValue, yi, zi));
        revPertRobotArray[(i-1)*bodyParams + 2]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi, yi - perturbedValue, zi));
        revPertRobotArray[(i-1)*bodyParams + 3]->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(xi, yi, zi - perturbedValue));
        revPertRobotArray[(i-1)*bodyParams + 4]->getBodyNode(i)->setMomentOfInertia(ixx - perturbedValue, iyy, izz, ixy, ixz, iyz);
        revPertRobotArray[(i-1)*bodyParams + 5]->getBodyNode(i)->setMomentOfInertia(ixx, iyy - perturbedValue, izz, ixy, ixz, iyz);
        revPertRobotArray[(i-1)*bodyParams + 6]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz - perturbedValue, ixy, ixz, iyz);
        revPertRobotArray[(i-1)*bodyParams + 7]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz, ixy - perturbedValue, ixz, iyz);
        revPertRobotArray[(i-1)*bodyParams + 8]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz, ixy, ixz - perturbedValue, iyz);
        revPertRobotArray[(i-1)*bodyParams + 9]->getBodyNode(i)->setMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz - perturbedValue);  
    }

    cout << "|-> Done\n";
    cout << "Calculating Phi Matrix ...\n";

    ofstream RHSIdeal;
    
    ofstream phibetaRHS;
    
    ofstream phidartfile;
    
    ofstream phifile;
    
    if(train) {
	    RHSIdeal.open ("/home/krang/Documents/DART/20-ParametricIdentification-7DOF/trainOutput/dataTorque_RHS.txt");
	    phibetaRHS.open ("/home/krang/Documents/DART/20-ParametricIdentification-7DOF/trainOutput/phibeta_RHS.txt");
	    phidartfile.open ("/home/krang/Documents/DART/20-ParametricIdentification-7DOF/trainOutput/phidart.txt");
	    phifile.open("/home/krang/Documents/DART/20-ParametricIdentification-7DOF/trainOutput/phi.txt");
	}
	else {
	    RHSIdeal.open ("/home/krang/Documents/DART/20-ParametricIdentification-7DOF/testOutput/dataTorque_RHS.txt");
	    phibetaRHS.open ("/home/krang/Documents/DART/20-ParametricIdentification-7DOF/testOutput/phibeta_RHS.txt");
	    phidartfile.open ("/home/krang/Documents/DART/20-ParametricIdentification-7DOF/testOutput/phidart.txt");
	    phifile.open("/home/krang/Documents/DART/20-ParametricIdentification-7DOF/testOutput/phi.txt");
	}

    //*******
	Eigen::MatrixXd phiDartMatrix(numBodies-1, numPertRobots);
    Eigen::MatrixXd phi(numBodies-1,1);
    Eigen::MatrixXd phiMatrix(numBodies-1, numPertRobots+21);

    for (int i = 0; i < numDataPoints; i++) {
        idealRobot->setPositions(allInitq.row(i));
        idealRobot->setVelocities(allInitqdot.row(i));
        Eigen::VectorXd ddq = allInitqdotdot.row(i);
        Eigen::MatrixXd M = idealRobot->getMassMatrix(); // n x n
        Eigen::VectorXd C = idealRobot->getCoriolisForces(); // n x 1
        Eigen::VectorXd G = idealRobot->getGravityForces(); // n x 1
        Eigen::VectorXd RHS_ideal= M*ddq + C + G;
        RHSIdeal << RHS_ideal.transpose() << endl;

        for (int k = 0; k < numPertRobots; k++) {
            // Set foward perturbed Robot
            fwdPertRobotArray[k]->setPositions(allInitq.row(i));
            fwdPertRobotArray[k]->setVelocities(allInitqdot.row(i));
            Eigen::MatrixXd M_pertfwd = fwdPertRobotArray[k]->getMassMatrix(); // n x n
            Eigen::VectorXd C_pertfwd = fwdPertRobotArray[k]->getCoriolisForces(); // n x 1
            Eigen::VectorXd G_pertfwd = fwdPertRobotArray[k]->getGravityForces(); // n x 1
            Eigen::VectorXd RHS_pertfwd = M_pertfwd*ddq + C_pertfwd + G_pertfwd; //}
            // Set reverse perturbed Robot
            revPertRobotArray[k]->setPositions(allInitq.row(i));
            revPertRobotArray[k]->setVelocities(allInitqdot.row(i));
            Eigen::MatrixXd M_pertrev = revPertRobotArray[k]->getMassMatrix(); // n x n
            Eigen::VectorXd C_pertrev = revPertRobotArray[k]->getCoriolisForces(); // n x 1
            Eigen::VectorXd G_pertrev = revPertRobotArray[k]->getGravityForces(); // n x 1
            Eigen::VectorXd RHS_pertrev = M_pertrev*ddq + C_pertrev + G_pertrev;

            // Calculate phi for beta i and pose
            phi = (RHS_pertfwd - RHS_pertrev)/(2*perturbedValue);
            // Add phi to phiMatrix and then print it looks cleaner
            phiDartMatrix.col(k) = phi;
		}

		Eigen::MatrixXd phi_ixx(numBodies-1,1);
		Eigen::MatrixXd phi_iyy(numBodies-1,1);
		Eigen::MatrixXd phi_izz(numBodies-1,1);
		Eigen::MatrixXd phi_ixy(numBodies-1,1);
		Eigen::MatrixXd phi_ixz(numBodies-1,1);
		Eigen::MatrixXd phi_iyz(numBodies-1,1);

		// Fix phi
        for(int b=1; b<numBodies; b++) {
            int c = bodyParams*(b-1);
            double m = idealRobot->getBodyNode(b)->getMass();

            // Eigen::Vector3d COM = idealRobot->getBodyNode(b)->getLocalCOM();

            cm_x = idealRobot->getBodyNode(b)->getLocalCOM()(0);
        	cm_y = idealRobot->getBodyNode(b)->getLocalCOM()(1);
        	cm_z = idealRobot->getBodyNode(b)->getLocalCOM()(2);

            phi_ixx = phiDartMatrix.col(4);
            phi_iyy = phiDartMatrix.col(5);
            phi_izz = phiDartMatrix.col(6);
            phi_ixy = phiDartMatrix.col(7);
            phi_ixz = phiDartMatrix.col(8);
            phi_iyz = phiDartMatrix.col(9);

            phiDartMatrix.col(c+1) = phiDartMatrix.col(c+1)/m - 2*cm_x*(phi_iyy + phi_izz) + cm_y*phi_ixy + cm_z*phi_ixz;
            phiDartMatrix.col(c+2) = phiDartMatrix.col(c+2)/m - 2*cm_y*(phi_ixx + phi_izz) + cm_x*phi_ixy + cm_z*phi_iyz;
            phiDartMatrix.col(c+3) = phiDartMatrix.col(c+3)/m - 2*cm_z*(phi_ixx + phi_iyy) + cm_x*phi_ixz + cm_y*phi_iyz;

		    // phiDartMatrix.block<7,3>(0,c+1) = phiDartMatrix.block<7,3>(0,c+1)/m; 
		    phiDartMatrix.col(c) = phiDartMatrix.col(c) - phiDartMatrix.col(c+1)*cm_x - phiDartMatrix.col(c+2)*cm_y - phiDartMatrix.col(c+3)*cm_z - phi_ixx*(pow(cm_y,2) + pow(cm_z,2)) - phi_iyy*(pow(cm_x,2) + pow(cm_z,2)) - phi_izz*(pow(cm_x,2) + pow(cm_y,2)) + phi_ixy*cm_x*cm_y + phi_ixz*cm_x*cm_z + phi_iyz*cm_y*cm_z;
 		}



 	// 	cout << "RHS:"<< RHS_ideal<<".\n";

		// cout << "RHS DART:"<<phiDartMatrix*betaParams.transpose()<<".\n";


        Eigen::MatrixXd gear_mat = Eigen::MatrixXd::Identity(7, 7);
		Eigen::MatrixXd viscous_mat = Eigen::MatrixXd::Identity(7, 7);
		Eigen::MatrixXd coulomb_mat = Eigen::MatrixXd::Identity(7, 7); 

		for (int j=0;j<7;j++)
		{
			gear_mat(j,j)    =  G_R(j)*G_R(j)*ddq(j);
			viscous_mat(j,j) =  allInitqdot.row(i)(j);
			// coulomb_mat(j,j)  = ((allInitqdot.row(i)(j)) > 0)? 1: -1;
            coulomb_mat(j,j)  = 2/(1 + exp(-2*(allInitqdot.row(i)(j)))) - 1;
		}       

		for(int j=0; j<7; j++){
			phiMatrix.block<7,10>(0,j*13) = phiDartMatrix.block<7,10>(0,j*10);
			phiMatrix.col(13*j+10) = gear_mat.col(j);
			phiMatrix.col(13*j+11) = viscous_mat.col(j);
			phiMatrix.col(13*j+12) = coulomb_mat.col(j);
		}

        Eigen::MatrixXd rhs_phibeta_diff(numBodies-1,3);
        rhs_phibeta_diff <<  RHS_ideal, (phiDartMatrix*betaParams.transpose()), ((phiDartMatrix*betaParams.transpose()) - RHS_ideal);
        phibetaRHS<<"RHS, phi*beta, difference at "<< i << endl << endl << rhs_phibeta_diff << endl << endl;
        phibetaRHS<< "=========================================================================" << endl << endl << endl << endl;
        phidartfile<< phiDartMatrix<<endl;
        phifile<< phiMatrix<<endl;
	}

	RHSIdeal.close();
	phibetaRHS.close();
	phidartfile.close();
	phifile.close();
	return phiMatrix;
}

Eigen::MatrixXd readInputFileAsMatrix(string inputFilename) {
    ifstream infile;
    infile.open(inputFilename);

    if (!infile.is_open()) {
        throw runtime_error(inputFilename + " can not be read, potentially does not exit!");
    }

    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    while(! infile.eof()) {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];
        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
    }

    infile.close();
    rows--;


    // Populate matrix with numbers.
    Eigen::MatrixXd outputMatrix(rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            outputMatrix(i,j) = buff[cols*i+j];

    return outputMatrix;
}
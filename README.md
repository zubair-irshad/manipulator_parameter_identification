# Parameter_ID

This repo contains code for determining regressor matrix for systematic parameter identification of robots.

## Dependencies

### DART - 

Dart is an open source physics engine for simulating articulated rigid bodies with high accuracy and efficiency and PyDart2 is a python binding of Dart. To install Dart, please follow the below instructions:
   
#### Prerequisites for Ubuntu

    sudo apt-get install build-essential cmake pkg-config git
    sudo apt-get install libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev
    sudo apt-get install libopenscenegraph-dev
    sudo apt-get install libbullet-dev
    sudo apt-get install liburdfdom-dev
    sudo apt-get install libnlopt-dev
    sudo apt-get install libxi-dev libxmu-dev freeglut3-dev
    sudo apt-get install libode-dev # ignore this if it tells you ode has already been installed
    
#### Download and install Dart

    git clone git://github.com/dartsim/dart.git
    cd dart
    git checkout tags/v6.3.0
    mkdir build
    cd build
    cmake ..
    make -j4
    sudo make install
    
#### 09-URDF     
This repo comes when you clone the entire repo 'Parameter_ID'
    
 Before compialation, change the file names of some important files listed below:
    
- InputQfilename
- InoutQdotfilename
- InputQdotdotfilename
- Inputtorquefilename
    
    
 ### Compilation

 Follow the steps to install

 mkdir build
 cd build
 cmake ..
 make

 ### Usage

In order to run with a simulation, follow instructions in 41-krang-sim-ach to launch the ach channels and processes required before this program is run. Then in the build folder of this repo, type:
  
    sudo ./Get_phi_ArmDynamics <train/test.

 Please use either train or test to run algorithm on training or testing data

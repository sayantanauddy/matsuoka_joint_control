# robot_walking
Repository for Independent Study - Sayantan Auddy

Temporary notes - To be sorted and finalized later

## Setup instructions - Nao in VREP + naoqi and choreographe

1.  Download VREP from http://www.coppeliarobotics.com/downloads.html
2.  Join the Aldebaran developer program. Register at https://community.ald.softbankrobotics.com/en/developerprogram. After successful registration, you will receive a developers license to use choreographe. This license will be required when choreographe is started later on.
3.  After registration, download the following from https://community.ald.softbankrobotics.com/en/resources/software/language/en-gb:
    1. Choregraphe 2.1.4
    2. Python 2.7 SDK 2.1.4 
    3. C++ SDK 2.1.4
    4. Cross Toolchain 2.1.4
4.  The naoqi documentation is available online at http://doc.aldebaran.com/2-1/ or can be downloaded from https://community.ald.softbankrobotics.com/en/resources/documents/language/en-gb
5.  Documentation about the Nao robot can also be downloaded from https://community.ald.softbankrobotics.com/en/resources/documents/language/en-gb
6.  Clone 'Project NAO Control': git clone https://github.com/PierreJac/Project-NAO-Control.git
7.  Launch v-rep and load the scene contained in the Vrep-scene folder
8.  Go to your choregraphe suite folder, then in the bin folder and launch naoqi-bin to create a virtual NAO on your computer
   `./naoqi-bin -p [Port Number] &`
9.  Add the path of the Python naoqi SDK to PYTHONPATH
10. You can launch choregraphe to visualize your virtual NAO or check its IP and Port using the connect button. For the simulation use IP: 127.0.0.1 and any port number.
11. Launch the VREP simulation by clicking on the play button.
12. Launch the multi_nao_control.py script if you have several NAO or the single_nao_control.py if you have just one NAO to control
Give all the informations needed (IPs and Ports) and wait until NAO is listening. This is needed for connecting naoqi with the VREP simulation.
13. In choreograph, set some robot movements. These movements will be reproduced in VREP as well.

[Setps 7-13 are from https://github.com/PierreJac/Project-NAO-Control]

## Nao Joint Control API
1. http://doc.aldebaran.com/2-1/naoqi/motion/control-joint-api.html#ALMotionProxy::setAngles__AL::ALValueCR.AL::ALValueCR.floatCR
2. http://doc.aldebaran.com/1-14/naoqi/motion/control-joint-api.html

## How to call Python functions from Matlab
1. Refer to instructions at http://de.mathworks.com/help/matlab/matlab_external/call-user-defined-custom-module.html and https://de.mathworks.com/help/matlab/matlab_external/call-modified-python-module.html#zmw57dd0e3028.
2. Example script: matlab/callPython.m and matlab/nao.py


## Matlab and V-REP interfacing

All resources for this section can be found in robot_walking/matlab/matlab-vrep.
Refer to http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm#simxGetObjectGroupData for the API

### Setup
1. The following files should be present in the current directory in Matlab
    1. remApi.m
    2. remoteApiProto.m
    3. the appropriate remote API library: "remoteApi.dll" (Windows), "remoteApi.dylib" (Mac) or "remoteApi.so" (Linux) 
    4. simpleTest.m (or any other example program)
    
    The remote API library can be found in the V-REP folder. For me it is in V-REP_PRO_EDU_V3_3_2_64_Linux/programming/remoteApiBindings/lib/lib/64Bit. Choose either the 64 or 32 bit library based on your architecture. The other files can be found in V-REP_PRO_EDU_V3_3_2_64_Linux/programming/remoteApiBindings/matlab/matlab. I have modified the file simpleTest.m to suit my purpose and copied the other necessary files to the folder robot_walking/matlab/matlab-vrep. If you use the 32 bit library then copy the 32 bit remoteApi.so file here.
    
    
2. Make sure to have the server side running in V-REP. For this start vrep.sh and import the scene Nao_empty.ttt.
   The following command is already present in the threaded child script Cuboid5 (can be viewed by opening the 'Scene hierarchy')
   simExtRemoteApiStart(19999)
   This command is needed to for V-REP and Matlab to communicate.
   Start the simulation.
   
3. call the function simpleTest() in Matlab. It will display the name and current absolute position [x,y,z] of the Nao in the environment.

4. Any other matlab script can be used to communicate with V-REP. The details of the functions can be found in the API documentation given in the beginning of this section.

   


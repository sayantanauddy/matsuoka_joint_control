# robot_walking
Repository for Independent Study - Sayantan Auddy

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


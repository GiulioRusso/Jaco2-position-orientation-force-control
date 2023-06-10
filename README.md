This document is part of the material of the robotics classes for the several master courses at the University of Cassino, downloaded from:
http://webuser.unicas.it/lai/robotica/index.php/software/
The compressed file containing this document is composed by:
	instructions.odt (this file)
	main_template.m
	/functions_coppelia
	/functions_matlab
	/vrepScene
uncompress it in your working folder (example: /myfolder/).
Step 1: link Matlab with coppeliaSim
	1.	Download the coppeliaSim educational version for the own O.S. from the website:
	2.	http://www.coppeliarobotics.com
	3.	Locate the coppelia subfolder
	4.	~/programming/remoteAPIBindings/matlab/matlab 
	5.	and copy the files into your working folder
	6.	Do the same with the library in the coppelia subfolder
	7.	~/programming/remoteApiBindings/lib/lib
	8.	(you can find a .dll, .dylib or .so file according to your O.S. and 32/64 bit)
	9.	Launch coppeliaSim (in Linux type ./coppeliaSim.sh from shell).  The program automatically runs as server on the door 19997
	10.	Drag any robot from the left menu on the empty scene
	11.	Run Matlab and launch the script SimpleTest.m. The program should read the number of objects present in the V-REP scene and it should end. If it does not work check line 17 and change port 19999 to 19997

If you see Failed connecting to remote API server
Se comunque non funziona e Matlab restituisce:
>> simpleTest
Program started
Note: always make sure you use the corresponding remoteApi library
(i.e. 32bit Matlab will not work with 64bit remoteApi, and vice-versa)
Failed connecting to remote API server
Program ended

From Coppelia launch
On terminal of Coppelia: simRemoteApi.start(19997) oppure simRemoteApi.start(19999)
The scene will start as Server on the port specified (make sure the port here is the same in the code). You should see on the MATLAB terminal:
>> simpleTest
Program started
Note: always make sure you use the corresponding remoteApi library
(i.e. 32bit Matlab will not work with 64bit remoteApi, and vice-versa)
Connected to remote API server
Number of objects in the scene: 41
Program ended

Step 2: run the template from the robotics class
	1.	In Coppelia, File->open scene and select any file *.ttt from the folder /vrepScene (it is simply the default scene with the chosen robot model where the pre-existent LUA script has been deleted)
	2.	Launch main_template.m on Matlab. The robot should perform sinusoidal trajectories in the joint space
The code sections to modify for the projects can be easily located in main_template.m.



YEI 3-Space Sensor C API
========================
##############
#Change Notes#
##############
1.0.5 -
Added examples directory that contains working source code examples of the API
Added Version Properties to the DLL to make it more clear the current version
Added Wireless Sensors will stop asynchronous once the DLL is cleaning up
Added setStopAllTSSAysncSensorsOnQuit to allow the user to explicitly override
	stopping asynchronous on close
Fixed getAccelerometerCalibrationParam command
Fixed the DLL from accidentally cleaning up when a thread detaches
Fixed crash on updateAsynchData with multiple sensors.
Added functions to interact with multiple wireless sensors similar to the 
	python API broadcasters, still in early stages, usage is not recommended 
	at this time

1.0.4 -
Reverted back to having arrays instead of singly linked lists, getting
	rid of the ComPortList structure. 
Added a defined print flag, "DEBUG_PRINT",that will print debug prints to 
	terminal window. Changed getYEIComPorts to take two vectors of vector of 
	strings, one for 3-Space Sensor devices and one for unknown devices.

1.0.3 -
Fixed memory leaks and changed from having an array of ComPort pointers
	to a linked list of ComPortList pointers. 
Created ComPortList structure for singly linked list. Added bthsdpdef.h to 
	include files for compatibility with older Windows.
Changed getFirstAvailableComPort and getNextAvailableComPort to 
	getFirstAvailableTSSComPort and getNextAvailableTSSComPort and added
	getFirstAvailableUnknownComPort and getNextAvailableUnknownComPort functions

1.0.2 -
Fixed some naming issues with asynchronous calls.
Fixed the TS classes to have upper case hex strings.
Added getFirstAvailableComPort and getNextAvailableComPort functions. 

1.0.1 -
Fixed some major issues from the 1.0.0 and BETA versions of the API.
Changed the run-time lib to no longer have a vs2010 dependency
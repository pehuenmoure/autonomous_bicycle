//This example demonstrates setting options on a sensor
//This is compatible with all sensors plugged in via USB or Bluetooth
//Will not work with the dongle or wireless sensor wirelessly see
// getting_information_wireless for a wireless example
//For Sensors plugged in with RS232 see creating_class_instances
#include "yei_threespace_api.h"
#include <stdio.h>
#include <Windows.h>

int main(){
	TSS_ID  device;
	TSS_Error tss_error;
	ComPort comport;

	if(!getFirstAvailableTSSComPort(&comport, TSS_FIND_ALL^TSS_FIND_DNG)){
		device =createTSDevice(comport);
		if( device == NO_DEVICE_ID){
			printf("Failed to create a sensor on %s\n",comport.com_port);
			return 1;
		}
	}
	else{
		printf("No sensors found\n");
		return 1;
	}
	printf("==================================================\n");
	printf("Setting the tared data of the device to an arbitrary quaternion.\n");
	TSS_Quaternion in_quat;
	in_quat.x = 0.7071f;
	in_quat.y = 0.0f;
	in_quat.z = 0.7071f;
	in_quat.w = 0.0f;
	tss_error = setTareQuaternion(device,in_quat);
	//Checking the error returned from the function, typicly succeed as long as the
	//parameters are valid on a plugged in sensor, functions sent to a sensor
	//wirelessly should allways be checked
	if(!tss_error){
		TSS_Quaternion out_quat;
		tss_error= getTareOrientQuat(device, &out_quat);
		if(!tss_error){
			printf("TareQuat:%f, %f, %f, %f\n", out_quat.x,
												out_quat.y,
												out_quat.z,
												out_quat.w);
		}
		else{
			printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
		}
	}
	else{
		printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
	}
	printf("==================================================\n");
	printf("Setting the LED color of the device to RED.\n");
	float red[3]={ 1.0f, 0.0f, 0.0f};
	tss_error= setLEDColorf3(device, red);
	if(!tss_error){
		printf("LED should be be RED\n");
	}
	else{
		printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
	}
	Sleep(2000);
	printf("==================================================\n");
	printf("Setting the LED color of the device to BLUE.\n");
	float blue[3]={ 0.0f, 0.0f, 1.0f};
	tss_error= setLEDColorf3(device, blue);
	if(!tss_error){
		printf("LED should be be BLUE\n");
	}
	else{
		printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
	}

	//To save settings on the sensor call commitSettings
	//commitSettings(device);

	closeTSDevice(device);

	printf("Finished press Enter to continue");
	getchar();
	return 0;
}
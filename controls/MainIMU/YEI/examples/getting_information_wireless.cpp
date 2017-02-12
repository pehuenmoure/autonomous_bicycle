//This is example requires a 3-Space Wireless and Dongle
//Dongle must be plugged into the computer and wireless sensor turned on
//This example gets and sets data on a wireless sensor
//If using bluetooth see getting_information and setting_information
//A wireless sensor must be paired on in Logical id 0 for this example
#include "yei_threespace_api.h"
#include <stdio.h>
#include <Windows.h>

int main(){
	TSS_ID w_device = NO_DEVICE_ID;
	TSS_ID dongle = NO_DEVICE_ID;
	TSS_Error tss_error;
	ComPort comport;

	if(!getFirstAvailableTSSComPort(&comport, TSS_FIND_DNG)){
		dongle =createTSDevice(comport);
		if( dongle == NO_DEVICE_ID){
			printf("Failed to create a sensor on %s\n",comport.com_port);
			return 1;
		}
	}
	else{
		printf("No sensors found\n");
		return 1;
	}
	//A wireless sensor must be paired on in Logical id 0 for this example
	getSensorFromDongle(dongle, 0, &w_device);
	if( w_device == NO_DEVICE_ID){
		printf("No sensor found \n");
		return 1;
	}
	//Getting data example, if the same data is needed frequently look into
	// using wireless async as it will provide better throughput
	printf("==================================================\n");
	printf("Getting the filtered tared quaternion orientation.\n");
	//Many functions provide a structure or an array to get the data
	TSS_Quaternion quat;
	tss_error= getFiltTaredOrientQuat(w_device, &quat);
	//wireless communication can fail at times, tss_error should allways be
	// be checked on wireless commands
	if(!tss_error){
		printf("Quaternion: %f, %f, %f, %f\n", quat.x, quat.y, quat.z ,quat.w);
	}
	else{
		printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
	}
	printf("==================================================\n");
	printf("Getting the raw sensor data.\n");
	float raw_data[9];
	tss_error= getAllSensorsRawf9(w_device, raw_data);
	if(!tss_error){
		printf("Gyro:  %f, %f, %f\n", raw_data[0],raw_data[1],raw_data[2]);
		printf("Accel: %f, %f, %f\n", raw_data[3],raw_data[4],raw_data[5]);
		printf("Comp:  %f, %f, %f\n", raw_data[6],raw_data[7],raw_data[8]);
	}
	else{
		printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
	}
	//Setting data example
	printf("==================================================\n");
	printf("Setting the LED color of the device to RED.\n");
	float red[3]={ 1.0f, 0.0f, 0.0f};
	tss_error= setLEDColorf3(w_device, red);
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
	tss_error= setLEDColorf3(w_device, blue);
	if(!tss_error){
		printf("LED should be be BLUE\n");
	}
	else{
		printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
	}

	//To save settings on the sensor call commitSettings
	//commitSettings(w_device);

	closeTSDevice(dongle);

	printf("Finished press Enter to continue");
	getchar();
	return 0;
}
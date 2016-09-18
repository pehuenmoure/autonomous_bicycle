//This example demonstrates getting basic data from the sensor
//This is compatible with all sensors plugged in via USB or Bluetooth
//Will not work with the dongle or wireless sensor wirelessly see
// getting_information_wireless for a wireless example
//For Sensors plugged in with RS232 see creating_class_instances
#include "yei_threespace_api.h"
#include <stdio.h>

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
	printf("Getting the filtered tared quaternion orientation.(xyzw)\n");
	//Many functions provide a structure or an array to get the data
	TSS_Quaternion quat;
	tss_error= getFiltTaredOrientQuat(device, &quat);
	if(!tss_error){
		printf("Quaternion: %f, %f, %f, %f\n", quat.x, quat.y, quat.z ,quat.w);
	}
	else{
		printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
	}
	printf("==================================================\n");
	printf("Getting the raw sensor data.\n");
	float raw_data[9];
	tss_error= getAllSensorsRawf9(device, raw_data);
	if(!tss_error){
		printf("Gyro:  %f, %f, %f\n", raw_data[0],raw_data[1],raw_data[2]);
		printf("Accel: %f, %f, %f\n", raw_data[3],raw_data[4],raw_data[5]);
		printf("Comp:  %f, %f, %f\n", raw_data[6],raw_data[7],raw_data[8]);
	}
	else{
		printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
	}
	printf("==================================================\n");
	printf("Getting the LED color of the device.\n");
	TSS_Color color;
	tss_error= getLEDColor(device, &color);
	if(!tss_error){
		printf("Color: %f, %f, %f\n", color.r, color.g, color.b);
	}
	else{
		printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
	}
	printf("==================================================\n");
	closeTSDevice(device);

	printf("Finished press Enter to continue");
	getchar();
	return 0;
}
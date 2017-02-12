//This is example requires a 3-Space Wireless and Dongle
//Dongle and wireless sensor must be plugged into the  same computer
//This example pairs the first wireless sensor it finds to the first dongle
// and assigns it to slot 0 in the logical id table and commits settings
//If using bluetooth read the 3-space bluetooth quickstart for pairing
#include "yei_threespace_api.h"
#include <stdio.h>

int main(){
	TSS_ID device = NO_DEVICE_ID;
	TSS_ID dongle = NO_DEVICE_ID;
	ComPort comport;

	if(!getFirstAvailableTSSComPort(&comport, TSS_FIND_WL)){
		device =createTSDevice(comport);
		if( device == NO_DEVICE_ID){
			printf("Failed to create a sensor on %s\n",comport.com_port);
			return 1;
		}
	}
	else{
		printf("No Wireless Sensor found\n");
		return 1;
	}
	if(!getFirstAvailableTSSComPort(&comport, TSS_FIND_DNG)){
		dongle =createTSDevice(comport);
		if( dongle == NO_DEVICE_ID){
			printf("Failed to create a sensor on %s\n",comport.com_port);
			return 1;
		}
	}
	else{
		printf("No Dongle found\n");
		return 1;
	}
	//Pairing requires 3 things, the dongle and wireless sensor pan_id, 
	// channel much match and the wireless sensor serial need
	unsigned short pan_id = 1234;
	unsigned char channel = 25; //Must be an integer value between 11 and 26
	AxisDirections axis_dir;
	TSS_ID wireless_id;
	unsigned int hw_id;

	printf("Setting the Pan id to: %d, and channel to: %d\n", pan_id, channel);
	//To simplify the example functions are not validated look at setting_data
	getSerialNumber(device, &hw_id);
	setPanID(device, pan_id);
	setChannel(device, channel);
	setPanID(dongle, pan_id);
	setChannel(dongle, channel); 
	
	printf("Setting the wireless table to pair\n");
	addSensorToDongle(dongle, 0, hw_id, &axis_dir, &wireless_id);

	printf("Commiting settings\n");
	commitWirelessSettings(device);
	commitWirelessSettings(dongle);

	closeTSDevice(device);
	closeTSDevice(dongle);

	printf("Finished press Enter to continue");
	getchar();
	return 0;
}
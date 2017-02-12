//This example finds and creates the device and returns a TSS_ID which is
// needed to call many functions in the API
#include "yei_threespace_api.h"
#include <stdio.h>

int main(){
	//If the COM port is already known and the device type is known for the 3-Space
	//Sensor device, we can just create the appropriate instance without doing 
	//a search.
	TSS_ID  device;
	char hex_str[9];
	printf("====Creating TS Devices on COM71 ====\n");
	device = createTSDeviceStr("COM71", TSS_WL);
	if( device == NO_DEVICE_ID){
		printf("Could not create device\n");
	}
	else{
		getSerialNumberHex(device,hex_str);
		printf("Port created successfully for sensor %s\n", hex_str);
	}
	closeTSDevice(device);
	
	//If the COM port is not known or the device type is not known for the 3-Space
	//Sensor device, we must do a search for the devices. We can do this by calling
	//the getAvailableComPorts function which returns two lists of COM port
	//information.
	//(known 3-Space Sensor devices and unknown devices)
	//getAvailableComPorts also as a parameter called filter_list that takes a list
	//of 3-Space Sensor device types which can be used to find specific 3-Space
	//Sensor devices. If filter_list is not used or set to None all connected
	//3-Space Sensor devices are found.
	//Each COM port information is a list containing
	//(COM port name, friendly name, 3-Space Sensor device type)
	ComPort *tss_ports;
	ComPort *ukn_ports;
	unsigned int tss_size;
	unsigned int ukn_size;

	getAvailableComPorts(	&tss_ports, 
							&ukn_ports, 
							&tss_size, 
							&ukn_size,
							TSS_FIND_ALL);
	printf("====Creating Known TS Devices ====\n");
	for( unsigned int i=0; i< tss_size; ++i){
		device =createTSDevice(tss_ports[i]);
		//Check if the id was successfully  returned, unless the device is in use by
		//another application this usually  succeeds
		if( device != NO_DEVICE_ID){
			getSerialNumberHex(device,hex_str);
			printf("Port created successfully for sensor %s\n", hex_str);
			closeTSDevice(device);
		}
		else{
			printf("Failed to create a sensor on %s\n",tss_ports[i].com_port);
		}
	}
	//The unknown list could contain 3-Space Sensor devices that were connected via
	//a RS232 connection. We can find out if there is any 3-Space Sensor devices
	//by using the getComPortInfo function. However, we must send commands over the
	//port which could make other devices other than 3-Space Sensor devices act
	//in an undesirable way. So be sure to know what port your 3-Space Sensor
	//device(s) are on when calling this function.
	//getComPortInfo returns a list of information that it found from the COM port
	//(Friendly name, 3-Space Type, 3-Space ID, 3-Space Firmware Version String,
	// 3-Space Hardware Version String, is in bootloader mode)
	printf("====Poking Unknown Ports For TS Devices====\n");
	for( unsigned int i=0; i< ukn_size; ++i){
		ComInfo com_info;
		getComPortInfo(ukn_ports[i].com_port, &com_info, true);
		if( com_info.sensor_type != TSS_UNKNOWN){
			device =createTSDeviceStr(ukn_ports[i].com_port, com_info.sensor_type);
			getSerialNumberHex(device,hex_str);
			printf("Port created successfully for sensor %s\n", hex_str);
			closeTSDevice(device);
		}
		else{
			printf("Sensor Not found\n");
		}
	}
	//This an alternate method of getting sensors, this one you call 
	//getFirstAvailableComPort and all the rest are called 
	//getNextAvailableComPort. If the call fails TSS_ERROR_NO_SENSOR_FOUND is
	//returned
	printf("===========Using getFirstAvailableComPort Method==============\n");
	ComPort comport;
	if(!getFirstAvailableTSSComPort(&comport, TSS_FIND_ALL)){
		device =createTSDevice(comport);
		//Check if the id was successfully  returned, unless the device 
		//is in use by another application this usually  succeeds
		if( device != NO_DEVICE_ID){
			getSerialNumberHex(device,hex_str);
			printf("Port created successfully for sensor %s\n", hex_str);
			closeTSDevice(device);
		}
		else{
			printf("Failed to create a sensor on %s\n",comport.com_port);
		}
		while( !getNextAvailableTSSComPort(&comport)){
			device =createTSDevice(comport);
			if( device != NO_DEVICE_ID){
				getSerialNumberHex(device,hex_str);
				printf("Port created successfully for sensor %s\n", hex_str);
				closeTSDevice(device);
			}
			else{
				printf("Failed to create a sensor on %s\n",comport.com_port);
			}
		}
	}

	printf("Finished press Enter to continue");
	getchar();
	return 0;
}
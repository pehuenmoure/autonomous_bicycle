//This example is using the wireless asynchronous method
//This is example requires a 3-Space Wireless and Dongle
//Dongle must be plugged into the computer and wireless sensor turned on
//Wireless Sensor Must be paired with Dongle and be in logical id 0
//This example uses the wireless async mode to stream data rapidly
// to the dongle for increased performance.
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
		printf("No Dongle found\n");
		return 1;
	}
    //This id is issued from the dongle
    w_device = NO_DEVICE_ID;
    getSensorFromDongle(dongle, 0, &w_device);
    if( w_device == NO_DEVICE_ID){
        printf("No sensor found \n");
        return 1;
    }
    //In order to asynchronously receive data, an asynchronous session must first be
    //started.
    //asynchGetFiltTaredOrientQuat starts the async from the wireless. If there is multiple
    //sensors active at the same time this may fail and require a retry.
    tss_error = asynchGetFiltTaredOrientQuat(w_device, 15, INF_DURATION);
    if(!tss_error){
        LARGE_INTEGER frequency;        // ticks per second
        LARGE_INTEGER t1, t2;           // ticks
        float quat[4];
        int timestamp;
        int size;
        QueryPerformanceFrequency(&frequency);
        QueryPerformanceCounter(&t1);
        QueryPerformanceCounter(&t2);
        while( 2.0 > ((t2.QuadPart-t1.QuadPart)*1.0f/frequency.QuadPart)){
            QueryPerformanceCounter(&t2);
            //asynchronous data received by the dongle must be read in and stored
            //in a local buffer. updateAsynchdata performs this function.
            //updateAsynchData must be called every time to grab the latest data from 
            //the dongle
            updateAsynchData();
            //Once the buffer is updated, the data for a particular sensor
            //may be read using one of the 'getAsynchData' functions. The function
            //chosen depends on the format of the data requested with the
            //function call that initiated the asynchronous session.
            getAsynchDataFloat(w_device, quat, &timestamp, &size);
            printf("Quat: %f, %f, %f, %f\n",    quat[0], 
                                                quat[1],
                                                quat[2],
                                                quat[3]);
        }
    }
    else{
        printf("Async failed to start\n");
    }
    //This stops the asynchronous session for a given device.
    stopAsynchronous(w_device);
 
    closeTSDevice(dongle);
 
    printf("Finished press Enter to continue");
    getchar();
    return 0;
}
////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#define ZMQ_PORT "tcp://*:7574"
#include "RTIMULib.h"
#include "zmq.h"

char* axis[12] = {
	"roll", 
	"pitch",
	"yaw",
	"compass_x",
	"compass_y",
	"compass_z",
	"gyro_x",
	"gyro_y",
	"gyro_z",
	"accel_x",
	"accel_y",
	"accel_z"
};

int zmq_publish_IMU(void* publisher, RTIMU_DATA &imuData);

int main()
{
	// ZMQ variables
	int err;
	void* context = zmq_init(1);
	void* publisher = zmq_socket(context, ZMQ_PUB);
	char value[20];

	// Prepare ZMQ context and publisher
	err = zmq_bind(publisher, ZMQ_PORT);
	if(err != 0){
		printf("zmq_bind failed with %s\n", zmq_strerror(zmq_errno()));
		zmq_close(publisher);
		zmq_term(context);
		exit(1);
	}

    // set up IMU
    RTIMUSettings *settings = new RTIMUSettings("URPO_IMU");
    RTIMU *imu = RTIMU::createIMU(settings);
    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }
    imu->IMUInit();

    RTIMU_DATA imuData;
    //  process and publish data
	int i=0;
    while (1) {
        usleep(imu->IMUGetPollInterval() * 1000);
		imu->IMURead();
		if(i++<10){
			continue;
		}else{
			i=0;	
        	//  poll at the rate recommended by the IMU
        	RTIMU_DATA imuData = imu->getIMUData();
			zmq_publish_IMU(publisher, imuData);
		}
    }
}

int str_to_msg(char* send, zmq_msg_t* msg)
{
    int err;
    int len = strlen(send);
    err = zmq_msg_init_size(msg, len);
    if (err != 0){
        printf("ERROR: zmq_msg_init_size failed with %s\n", zmq_strerror(zmq_errno()));
        return err;
    }
    memcpy(zmq_msg_data(msg), send, len);
    return 0;
}

int zmq_publish_IMU(void* publisher, RTIMU_DATA &imuData)
{
    int err;
    zmq_msg_t msgpart;
    // use the axis as topic
    err = str_to_msg("IMU", &msgpart);
    if (err != 0){
        zmq_msg_close(&msgpart);
        return err;
    }
    err = zmq_send(publisher, &msgpart, ZMQ_SNDMORE);
    zmq_msg_close(&msgpart);
    if (err != 0){
        printf("ERROR: zmq_send failed with %s\n", zmq_strerror(zmq_errno()));
        return err;
    }

	char value[64];
	sprintf(value, "%f", imuData.compass.x());
    err = str_to_msg(value, &msgpart);
    err = zmq_send(publisher, &msgpart, ZMQ_SNDMORE);
    zmq_msg_close(&msgpart);
	
	sprintf(value, "%f", imuData.compass.y());
    err = str_to_msg(value, &msgpart);
    err = zmq_send(publisher, &msgpart, ZMQ_SNDMORE);
    zmq_msg_close(&msgpart);
	
	sprintf(value, "%f", imuData.compass.z());
    err = str_to_msg(value, &msgpart);
    err = zmq_send(publisher, &msgpart, ZMQ_SNDMORE);
    zmq_msg_close(&msgpart);
	
	sprintf(value, "%f", imuData.gyro.x());
    err = str_to_msg(value, &msgpart);
    err = zmq_send(publisher, &msgpart, ZMQ_SNDMORE);
    zmq_msg_close(&msgpart);
	
	sprintf(value, "%f", imuData.gyro.y());
    err = str_to_msg(value, &msgpart);
    err = zmq_send(publisher, &msgpart, ZMQ_SNDMORE);
    zmq_msg_close(&msgpart);
	
	sprintf(value, "%f", imuData.gyro.z());
    err = str_to_msg(value, &msgpart);
    err = zmq_send(publisher, &msgpart, ZMQ_SNDMORE);
    zmq_msg_close(&msgpart);
	
	sprintf(value, "%f", imuData.accel.x());
    err = str_to_msg(value, &msgpart);
    err = zmq_send(publisher, &msgpart, ZMQ_SNDMORE);
    zmq_msg_close(&msgpart);
	
	sprintf(value, "%f", imuData.accel.y());
    err = str_to_msg(value, &msgpart);
    err = zmq_send(publisher, &msgpart, ZMQ_SNDMORE);
    zmq_msg_close(&msgpart);
	
	sprintf(value, "%f", imuData.accel.z());
    err = str_to_msg(value, &msgpart);
    err = zmq_send(publisher, &msgpart, 0);
    zmq_msg_close(&msgpart);
		
    return 0;
}

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

char* axis[3] = {
	"roll", 
	"pitch",
	"yaw"
};

int zmq_publish_value(void* publisher, char* axis, char* value);

int main()
{
	// ZMQ variables
	int err;
	void* context = zmq_init(1);
	void* publisher = zmq_socket(context, ZMQ_PUB);
	char value[20];

	//IMU variables
    int sampleCount = 0;
    uint64_t publishTimer;
    uint64_t now;


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

	float mean_x = 0, mean_y = 0, mean_z = 0;
	int i;
	publishTimer = 0;
    //  process and publish data
    while (1) {
        //  poll at the rate recommended by the IMU
        usleep(imu->IMUGetPollInterval() * 1000);

        while (imu->IMURead()) {
            RTIMU_DATA imuData = imu->getIMUData();
			mean_x += imuData.fusionPose.x();
			mean_y += imuData.fusionPose.y();
			mean_z += imuData.fusionPose.z();
            sampleCount++;

            now = RTMath::currentUSecsSinceEpoch();
            //  publish 10 times per second
            if ((now - publishTimer) > 100000) {
				mean_x /= sampleCount;
				mean_y /= sampleCount;
				mean_z /= sampleCount;

				sprintf(value, "%f", mean_x);
				zmq_publish_value(publisher, axis[0], value);
				sprintf(value, "%f", mean_y);
				zmq_publish_value(publisher, axis[1], value);
				sprintf(value, "%f", mean_z);
				zmq_publish_value(publisher, axis[2], value);

				sampleCount = 0;
				mean_x = 0;
				mean_y = 0;
				mean_z = 0;
/*
				sprintf(value, "%f", imuData.fusionPose.x());
				zmq_publish_value(publisher, axis[0], value);
				sprintf(value, "%f", imuData.fusionPose.y());
				zmq_publish_value(publisher, axis[1], value);
				sprintf(value, "%f", imuData.fusionPose.z());
				zmq_publish_value(publisher, axis[2], value);
*/
                publishTimer = now;
            }
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

int zmq_publish_value(void* publisher, char* axis, char* value)
{
    int err;
    zmq_msg_t msgpart;
    // use the axis as topic
    err = str_to_msg(axis, &msgpart);
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

    // send value as the message
    err = str_to_msg(value, &msgpart);
    if (err != 0){
        zmq_msg_close(&msgpart);
        return err;
    }
    err = zmq_send(publisher, &msgpart, 0);
    zmq_msg_close(&msgpart);
    if (err != 0){
        printf("ERROR: zmq_send failed with %s\n", zmq_strerror(zmq_errno()));
        return err;
    }
    return 0;
}

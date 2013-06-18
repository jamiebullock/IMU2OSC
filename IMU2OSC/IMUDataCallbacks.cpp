//
//  IMUDataCallbacks.c
//  IMU2OSC
//
//  Created by Jamie Bullock on 18/05/2012.
//  Copyright (c) 2012 Birmingham City University. All rights reserved.
//

#include <stdlib.h>

#include "IMUOSCHandlers.h"
#include "IMUDataCallbacks.h"
/* horrible hack: FIX! */



struct data_ data = {NULL, NULL, NULL, NULL};


struct data_ get_data(void) {
    return data;
}


/* horrible hack: FIX! */

void callback_incoming_euler(double *euler) {
    data.euler = euler;
    osc_send_euler(euler);
}

void callback_incoming_cal(double *gyro, double *accel, double *mag) {
    data.gyro = gyro;
    data.accel = accel;
    data.mag = mag;
    osc_send_cal(gyro, accel, mag);
}



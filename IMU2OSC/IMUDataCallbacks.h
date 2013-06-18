//
//  IMUDataCallbacks.h
//  IMU2OSC
//
//  Created by Jamie Bullock on 18/05/2012.
//  Copyright (c) 2012 Birmingham City University. All rights reserved.
//

#ifndef IMU2OSC_IMUDataCallbacks_h
#define IMU2OSC_IMUDataCallbacks_h


struct data_ {
    
    double *euler;
    double *gyro;
    double *mag;
    double *accel;
    
};

struct data_ get_data(void);

void callback_incoming_euler(double *euler);
void callback_incoming_cal(double *gyro, double *accel, double *mag);


 #endif // IMU2OSC_IMUDataCallbacks_h    

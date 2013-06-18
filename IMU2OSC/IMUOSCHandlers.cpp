//
//  IMUOSCHandlers.cpp
//  IMU2OSC
//
//  Created by Jamie Bullock on 18/05/2012.
//  Copyright (c) 2012 Birmingham City University. All rights reserved.
//

#include <iostream>

#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"
//#include "IpEndpointName.h"


#define OUTPUT_BUFFER_SIZE  1024
#define ADDRESS "127.0.0.1"
#define PORT 7000

UdpTransmitSocket *transmitSocket = NULL;

void osc_init(const char *address, unsigned int port)
{
//    UdpTransmitSocket newTransmitSocket(IpEndpointName(address, port));
    if (transmitSocket != NULL)
    {
        delete transmitSocket;
    }
    
    transmitSocket = new UdpTransmitSocket(IpEndpointName(address, port));
}

void osc_send_euler(double *euler) {
    
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
    
    p << osc::BeginMessage( "/x-imu/euler" ) << (float)euler[0] << (float)euler[1] << (float)euler[2] << osc::EndMessage;

    if (transmitSocket->IsConnected())
    {
        transmitSocket->Send( p.Data(), p.Size() );
    }
    else
    {
        printf("No OSC connection\n");
    }
    p.Clear();

}

void osc_send_cal(double *gyro, double *accel, double *mag) {
    
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
    
    p << osc::BeginMessage( "/x-imu/calibrated" ) << (float)gyro[0] << (float)gyro[1] << (float)gyro[2] << (float)accel[0] << (float)accel[1] << (float)accel[2] << (float)mag[0] << (float)mag[1] << (float)mag[2] << osc::EndMessage;
    
    if (transmitSocket->IsConnected())
    {
        transmitSocket->Send( p.Data(), p.Size() );
    }
    p.Clear();
    
}

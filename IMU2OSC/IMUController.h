//
//  IMUController.h
//  IMU2OSC
//
//  Created by Jamie Bullock on 08/05/2012.
//  Copyright (c) 2012 Jamie Bullock. All rights reserved.
//

#import <Cocoa/Cocoa.h>

#include "XIMU.h"

@interface IMUController : NSObject {
    
    IBOutlet NSPopUpButton *serialListPullDown;
    IBOutlet NSTextView *serialOutputArea;
    
    NSTimer *timer; 
    
    bool captureIsOn;
    bool imuInitialized;
    
}

- (IBAction) serialPortSelected: (id) cntrl;
- (IBAction) captureOnOff: (id) toggle;

@end

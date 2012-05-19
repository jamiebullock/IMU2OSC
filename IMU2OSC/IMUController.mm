//
//  IMUMController.m
//  IMU2OSC
//
//  Created by Jamie Bullock on 08/05/2012.
//  Copyright (c) 2012 Jamie Bullock. All rights reserved.
//


#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/IOBSD.h>
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>

#import "IMUController.h"

#include "XIMU.h"
#include "IMUDataCallbacks.h"


@interface IMUController() {
    
    XIMU *ximu;
    
}
@end



@implementation IMUController

- (id)init
{
    self = [super init];
    if (self) {
        //self->ximu = new XIMU("/foo/bar", XIMU::XIMU_LOGLEVEL_LOG);
        //[self detectAndGetDeviceId];
        timer = [NSTimer scheduledTimerWithTimeInterval:1.0/24. target:self selector:@selector(tick:) userInfo:nil repeats:YES];
        captureIsOn = false;
    }
    
    return self;
}


- (void)awakeFromNib {
    [self refreshSerialList:@"Select a Serial Port"];
}

- (void) refreshSerialList: (NSString *) selectedText {
	io_object_t serialPort;
	io_iterator_t serialPortIterator;
	
	// remove everything from the pull down list
	[serialListPullDown removeAllItems];
	
	// ask for all the serial ports
	IOServiceGetMatchingServices(kIOMasterPortDefault, IOServiceMatching(kIOSerialBSDServiceValue), &serialPortIterator);
	
	// loop through all the serial ports and add them to the array
	while ((serialPort = IOIteratorNext(serialPortIterator))) {
		[serialListPullDown addItemWithTitle:
         (__bridge NSString*)IORegistryEntryCreateCFProperty(serialPort, CFSTR(kIOCalloutDeviceKey),  kCFAllocatorDefault, 0)];
		IOObjectRelease(serialPort);
	}
	
	// add the selected text to the top
	[serialListPullDown insertItemWithTitle:selectedText atIndex:0];
	[serialListPullDown selectItemAtIndex:0];
	
	IOObjectRelease(serialPortIterator);
}


- (void)appendToIncomingText: (id) text {
	// add the text to the textarea
	NSAttributedString* attrString = [[NSMutableAttributedString alloc] initWithString: text];
	NSTextStorage *textStorage = [serialOutputArea textStorage];
	[textStorage beginEditing];
	[textStorage appendAttributedString:attrString];
	[textStorage endEditing];
	
	// scroll to the bottom
	NSRange myRange;
	myRange.length = 1;
	myRange.location = [textStorage length];
	[serialOutputArea scrollRangeToVisible:myRange]; 
}

- (void)detectAndGetDeviceId {
    
    unsigned int deviceid;
    
    if(!self->ximu->get_device_detected()) {
        [self appendToIncomingText: @"> no device found"];
    }
    if (!ximu->get_register(XIMU::REGISTER_ADDRESS_DeviceID, &deviceid)){
        [self appendToIncomingText: @"> ERROR: cant fetch deviceid\n"];
    } else {
        NSString *msg = [[NSString alloc] initWithFormat:@"> found device with id 0x%04X\n", deviceid];
        [self appendToIncomingText:msg];
    }
    
}

// action sent when serial port selected
- (IBAction) serialPortSelected: (id) cntrl {
    
    NSString *portName = [serialListPullDown titleOfSelectedItem];
    const char *portName_s = [portName UTF8String];
	// open the serial port
	//NSString *error = [self openSerialPort: [serialListPullDown titleOfSelectedItem] baud:[baudInputField intValue]];
	
    self->ximu = new XIMU((char *)portName_s, XIMU::XIMU_LOGLEVEL_LOG);
    [self detectAndGetDeviceId];
    [self refreshSerialList:portName];
}

- (void) enableCapture {
    captureIsOn = true;
    self->ximu->connect_slot_incoming_data_euler(&callback_incoming_euler);
    self->ximu->connect_slot_incoming_data_cal(&callback_incoming_cal);
}

- (void) disableCapture {
    captureIsOn = false;
    self->ximu->disconnect_slot_incoming_data_euler(); 
    self->ximu->disconnect_slot_incoming_data_cal(); 
}

- (IBAction) captureOnOff: (id) toggle {
    
    NSButton *button = toggle;
    if ([button state] == NSOnState){
        [self enableCapture];
        return;
    } else {
        [self disableCapture];
    }
}

-(void) tick:(NSTimer *)timer
{
    if(!captureIsOn) {
        return;
    }
    
    struct data_ data = get_data();
    
    double *euler = data.euler;
    double *gyro = data.gyro;
    double *accel = data.accel;
    double *mag = data.mag;
    
    NSString *msg;
    
    if (euler != NULL) {
        msg = [[NSString alloc] initWithFormat:@"> euler:\t%.3f\t%.3f\t%.3f\n",
                         euler[0], euler[1], euler[2]];
        [self appendToIncomingText: msg];
    }
    if (gyro != NULL) {
       msg = [[NSString alloc] initWithFormat:@"> gyro:\t%.3f\t%.3f\t%.3f\n",
                         gyro[0], gyro[1], gyro[2]];
        [self appendToIncomingText: msg];
    }
    if (accel != NULL) {
        msg = [[NSString alloc] initWithFormat:@"> accel:\t%.3f\t%.3f\t%.3f\n",
               accel[0], accel[1], accel[2]];
        [self appendToIncomingText: msg];
    }
    if (mag != NULL) {
        msg = [[NSString alloc] initWithFormat:@"> mag:\t%.3f\t%.3f\t%.3f\n",
               mag[0], mag[1], mag[2]];
        [self appendToIncomingText: msg];
    }

}


-(void)dealloc {
    delete self->ximu;
    //[super dealloc];
}

@end

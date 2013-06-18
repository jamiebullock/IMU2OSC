//
//  IMUMController.m
//  IMU2OSC
//
//  Created by Jamie Bullock on 08/05/2012.
//  Copyright (c) 2012 Birmingham City University. All rights reserved.
//


#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/IOBSD.h>
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>

#import "IMUController.h"

#include "XIMU.h"
#include "IMUDataCallbacks.h"
#include "IMUOSCHandlers.h"

#define ADDRESS "127.0.0.1"
#define PORT 7000
#define DISPLAY_RATE 1.0 / 10.0

@interface IMUController() {
    
    XIMU *ximu;
    
}

@property (assign, nonatomic) NSUInteger port;
@property (strong, nonatomic) NSString *address;

@property (weak) IBOutlet NSTextField *euler1;
@property (weak) IBOutlet NSTextField *euler2;
@property (weak) IBOutlet NSTextField *euler3;
@property (weak) IBOutlet NSTextField *gyro1;
@property (weak) IBOutlet NSTextField *gyro2;
@property (weak) IBOutlet NSTextField *gyro3;
@property (weak) IBOutlet NSTextField *mag1;
@property (weak) IBOutlet NSTextField *mag2;
@property (weak) IBOutlet NSTextField *mag3;
@property (weak) IBOutlet NSTextField *accel1;
@property (weak) IBOutlet NSTextField *accel2;
@property (weak) IBOutlet NSTextField *accel3;
@property (weak) IBOutlet NSTextField *portTextField;
@property (weak) IBOutlet NSTextField *ipAddressTextField;

@end

@implementation IMUController

- (id)init
{
    self = [super init];
    if (self) {
        
        NSUserDefaults *standardUserDefaults = [NSUserDefaults standardUserDefaults];
//        NSString *val = nil;
        
        self.port = PORT;
        self.address = [NSString stringWithUTF8String:ADDRESS];
        
        if (standardUserDefaults)
        {
            NSString *storedAddress = [standardUserDefaults stringForKey:@"Address"];
            NSInteger storedPort = [standardUserDefaults integerForKey:@"Port"];
            
            if (storedAddress == NULL)
            {
                [standardUserDefaults setObject:self.address forKey:@"Address"];
            }
            else
            {
                self.port = storedPort;
            }
            if (storedPort == 0)
            {
                [standardUserDefaults setValue:[NSNumber numberWithLong:self.port] forKey:@"Port"];
            }
            else
            {
                self.address = storedAddress;
            }
            [standardUserDefaults synchronize];
        }
                
        osc_init([self.address UTF8String], (unsigned int)self.port);
        
        timer = [NSTimer scheduledTimerWithTimeInterval:DISPLAY_RATE target:self selector:@selector(tick:) userInfo:nil repeats:YES];
        captureIsOn = false;
        self->ximu = NULL;
    }
        
    return self;
}


- (void)awakeFromNib {
    [self refreshSerialList:@"Select a Serial Port"];
    [self.portTextField setStringValue:[NSNumber numberWithLong:self.port].stringValue];
    [self.ipAddressTextField setStringValue:self.address];
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
    
    if (!ximu->get_register(XIMU::REGISTER_ADDRESS_DeviceID, &deviceid)){
        [self appendToIncomingText: @"> ERROR: cant fetch device ID\n"];
    } else {
        NSString *msg = [[NSString alloc] initWithFormat:@"> Connected to device with ID 0x%04X\n", deviceid];
        [self appendToIncomingText:msg];
    }
    if(!self->ximu->get_device_detected()) {
        [self appendToIncomingText: @"> No device found\n"];
    }
}

- (IBAction) serialPortSelected: (id) cntrl {
        
    NSString *portName = [serialListPullDown titleOfSelectedItem];
    const char *portName_s = [portName UTF8String];
	
    [self appendToIncomingText: @"> Attempting to connect to x-IMU\n"];
    
    delete self->ximu;
    self->ximu = new XIMU((char *)portName_s, XIMU::XIMU_LOGLEVEL_NONE);
    
    if (self->ximu == NULL) {
        NSLog(@"error: failed to instantiate XIMU object");
        return;
    }
    
    imuInitialized = self->ximu->detect_device();
    
    if (!imuInitialized) {
        [self appendToIncomingText: @"> Unable to connect to x-IMU\n"];
        delete self->ximu;
        self->ximu = NULL;
    }
    else
    {
        [self detectAndGetDeviceId];
    }
    
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
        
    if (euler != NULL) {
        [self.euler1 setStringValue:[NSNumber numberWithFloat:euler[0]].stringValue];
        [self.euler2 setStringValue:[NSNumber numberWithFloat:euler[1]].stringValue];
        [self.euler3 setStringValue:[NSNumber numberWithFloat:euler[2]].stringValue];
    }
    if (gyro != NULL) {
        [self.gyro1 setStringValue:[NSNumber numberWithFloat:gyro[0]].stringValue];
        [self.gyro2 setStringValue:[NSNumber numberWithFloat:gyro[1]].stringValue];
        [self.gyro3 setStringValue:[NSNumber numberWithFloat:gyro[2]].stringValue];
    }
    if (accel != NULL) {
        [self.accel1 setStringValue:[NSNumber numberWithFloat:accel[0]].stringValue];
        [self.accel2 setStringValue:[NSNumber numberWithFloat:accel[1]].stringValue];
        [self.accel3 setStringValue:[NSNumber numberWithFloat:accel[2]].stringValue];
    }
    if (mag != NULL) {
        [self.mag1 setStringValue:[NSNumber numberWithFloat:mag[0]].stringValue];
        [self.mag2 setStringValue:[NSNumber numberWithFloat:mag[1]].stringValue];
        [self.mag3 setStringValue:[NSNumber numberWithFloat:mag[2]].stringValue];
    }
}


- (IBAction)ipSet:(id)sender {
    
    NSTextField *textField = (NSTextField *)sender;
    NSUserDefaults *standardUserDefaults = [NSUserDefaults standardUserDefaults];
    self.address = [textField stringValue];

    [standardUserDefaults setObject:self.address forKey:@"Address"];
    [standardUserDefaults synchronize];
    //create new transmitSocket
    osc_init([self.address UTF8String], (unsigned int)self.port);


    
}

- (IBAction)portSet:(id)sender {
    
    NSTextField *textField = (NSTextField *)sender;
    NSUserDefaults *standardUserDefaults = [NSUserDefaults standardUserDefaults];
    self.port = [textField integerValue];
    
    [standardUserDefaults setValue:[NSNumber numberWithLong:self.port] forKey:@"Port"];
    [standardUserDefaults synchronize];

    osc_init([self.address UTF8String], (unsigned int)self.port);

}


-(void)dealloc {
    delete self->ximu;
    //[super dealloc];
}

@end

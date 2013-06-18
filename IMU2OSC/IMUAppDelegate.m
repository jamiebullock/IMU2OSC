//
//  IMUAppDelegate.m
//  IMU2OSC
//
//  Created by Jamie Bullock on 08/05/2012.
//  Copyright (c) 2012 Jamie Bullock. All rights reserved.
//

#import "IMUAppDelegate.h"

@implementation IMUAppDelegate

@synthesize window = _window;

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification
{
    // Insert code here to initialize your application
}

- (IBAction)helpMenuItemSelector:(id)sender {
    
//    NSString *helpBookName = [[NSBundle mainBundle] objectForInfoDictionaryKey: @"CFBundleHelpBookName"];
//    [[NSHelpManager sharedHelpManager] openHelpAnchor: inBook:helpBookName];

    [[NSApplication sharedApplication] showHelp:nil];
}


@end

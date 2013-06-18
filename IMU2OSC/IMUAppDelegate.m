//
//  IMUAppDelegate.m
//  IMU2OSC
//
//  Created by Jamie Bullock on 08/05/2012.
//  Copyright (c) 2012 Birmingham City University. All rights reserved.
//

#import "IMUAppDelegate.h"

@implementation IMUAppDelegate

@synthesize window = _window;

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification
{
    // Insert code here to initialize your application
}

- (IBAction)helpMenuItemSelector:(id)sender
{
    [[NSApplication sharedApplication] showHelp:nil];
}

- (IBAction)supportMenuItemSelector:(id)sender
{
    [[NSWorkspace sharedWorkspace] openURL:[NSURL URLWithString:@"https://github.com/jamiebullock/IMU2OSC/issues"]];
}


@end

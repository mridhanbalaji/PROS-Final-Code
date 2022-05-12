#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor Intake = motor(PORT21, ratio6_1, true);
motor LifterFrontMotorA = motor(PORT1, ratio36_1, true);
motor LifterFrontMotorB = motor(PORT2, ratio36_1, false);
motor_group LifterFront = motor_group(LifterFrontMotorA, LifterFrontMotorB);
digital_out Pneumatics1 = digital_out(Brain.ThreeWirePort.A);
motor LifterBack = motor(PORT9, ratio36_1, false);
motor DriveTrainLeftMotorA = motor(PORT13, ratio18_1, true);
motor DriveTrainLeftMotorB = motor(PORT14, ratio18_1, true);
motor_group DriveTrainLeft = motor_group(DriveTrainLeftMotorA, DriveTrainLeftMotorB);
motor DriveTrainRightMotorA = motor(PORT17, ratio18_1, false);
motor DriveTrainRightMotorB = motor(PORT18, ratio18_1, false);
motor_group DriveTrainRight = motor_group(DriveTrainRightMotorA, DriveTrainRightMotorB);
digital_out Pneumatics2 = digital_out(Brain.ThreeWirePort.C);
// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}
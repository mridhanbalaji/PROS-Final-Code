void doDrive() {
  int DriveTrainRightVelocity = 0;
  int DriveTrainLeftVelocity = 0;
    DriveTrainRightVelocity = Controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - Controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    DriveTrainLeftVelocity = Controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - Controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    if (DriveTrainRightVelocity > 100) {
      DriveTrainRightVelocity = 100;
    } else if (DriveTrainRightVelocity < -100) {
      DriveTrainRightVelocity = -100;
    }
    if (DriveTrainLeftVelocity > 100) {
      DriveTrainLeftVelocity = 100;
    } else if (DriveTrainLeftVelocity < -100) {
      DriveTrainLeftVelocity = -100;
    }
    DriveTrainRB = DriveTrainRightVelocity;
    DriveTrainRF = DriveTrainRightVelocity; 
    DriveTrainLB = DriveTrainLeftVelocity;
    DriveTrainLB = DriveTrainLeftVelocity;
}
void doIntake() {
   int intakeVelocity = 0;
     if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_X) and Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
       intakeVelocity = 0;
     } else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
       intakeVelocity = 100;
     } else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
       intakeVelocity = -100;
     } else {
       intakeVelocity = 0;
     }
     Intake = intakeVelocity;
 }
 void doLifterFront() {
   int lifterFrontVelocity = 0;
     if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1) and Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
       lifterFrontVelocity = 0;
     } else if (pros::E_CONTROLLER_DIGITAL_R1) {
       lifterFrontVelocity = 100;
     } else if (pros::E_CONTROLLER_DIGITAL_R2) {
       lifterFrontVelocity = -100;
     } else {
       lifterFrontVelocity = 0;
     }
     LifterFrontRight = lifterFrontVelocity;
     LifterFrontLeft = lifterFrontVelocity;
 }
 void doLifterBack() {
   int lifterBackVelocity = 0;
     if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1) and Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
       lifterBackVelocity = 0;
     } else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
       lifterBackVelocity = 100;
     } else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
       lifterBackVelocity = -100;
     } else {
       lifterBackVelocity = 0;
     }
     LifterBack = lifterBackVelocity;
 }
 void doPneumatics() {
     if(Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
       Pneumatics1.set_value(true);
       Pneumatics2.set_value(true);
     }
     if(Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
       Pneumatics1.set_value(false);
       Pneumatics2.set_value(false);
     }
 }


//Main Driver Control 
void opcontrol() {
  DriveTrainLF.move_velocity(200);
  DriveTrainLB.move_velocity(200);
  DriveTrainRB.move_velocity(200); 
  DriveTrainRF.move_velocity(200); 
  Intake.move_velocity(600); 
  LifterFrontLeft.move_velocity(100);
  LifterFrontRight.move_velocity(100); 
  LifterBack.move_velocity(100);
  while (true) {
    doDrive();
    doIntake();
    doLifterBack();
    doLifterFront();
    doPneumatics();
    pros::delay(10);
  }
}
#include <DeltaKinematics.h>
#include <AccelStepper.h>


// Define constants
#define Multiplier 18.5185  // step per degree 1/16th and 50/24 pulley
#define Speed 1000          // SPEED = Steps / second 1000 is max reliability
#define Acceleration 500    // ACCELERATION = Steps / (second)^2
#define HomeAngle 0      // Set the home angle
// Define pin assignments
#define AStepPin 2
#define BStepPin 3
#define CStepPin 4




#define ADirPin 5
#define BDirPin 6
#define CDirPin 7




#define enablePin 8




#define ALimitPin 11
#define BLimitPin 9
#define CLimitPin 10




// Define movement limits
#define X_MIN -122.909
#define X_MAX 170
#define Y_MIN -122.909
#define Y_MAX 122.909
#define Z_MIN -430
#define Z_MAX -210




// Define motor angle limits (in degrees)
#define THETA1_MIN -31.24
#define THETA1_MAX 94.97
#define THETA2_MIN -42.3
#define THETA2_MAX 95.98
#define THETA3_MIN -42.3
#define THETA3_MAX 95.98




#define HOME_X 0
#define HOME_Y 0
#define HOME_Z -209.235
// Create DeltaKinematics object with specified dimensions
DeltaKinematics DK(180, 284, 60, 111);  // in millimeters
//ArmLength(rf), RodLength(re), BassTri(e) and PlatformTri(f)
// Create AccelStepper objects for each motor
AccelStepper stepperA(AccelStepper::DRIVER, AStepPin, ADirPin);
AccelStepper stepperB(AccelStepper::DRIVER, BStepPin, BDirPin);
AccelStepper stepperC(AccelStepper::DRIVER, CStepPin, CDirPin);




void setup() {
  Serial.begin(9600);




  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);// move  clockwise




  configureStepper(stepperA);
  configureStepper(stepperB);
  configureStepper(stepperC);




  Serial.println("START");
  digitalWrite(enablePin, LOW);// enable the motor
  Serial.println("HOME");
  HomeMachine();
  //moveToPosition(0,0 , -300);


    Serial.println("SETTING");
  //circleHome();
  //testWorkSpace();
  //moveObject(120, -120, -430);
}




void loop() {
  if (Serial.available() > 0) {
    Serial.println("Enter coordinates in the format: x y z");




    double x = Serial.parseFloat();
    double y = Serial.parseFloat();
    double z = Serial.parseFloat();




    if (Serial.read() == '\n') {  // Ensure the user input ends with a newline
      moveToPosition(x, y, z);
    } else {
      Serial.println("Error: Invalid input format");
      Serial.flush();  // Clear the input buffer
    }
  }
}




void configureStepper(AccelStepper &stepper) {
  stepper.setMaxSpeed(Speed);
  stepper.setSpeed(Speed);
  stepper.setCurrentPosition(0);
  stepper.setAcceleration(Acceleration);
}




void moveToPosition(double x, double y, double z) {
  if (x < X_MIN || x > X_MAX || y < Y_MIN || y > Y_MAX || z < Z_MIN || z > Z_MAX) {
    Serial.println("Error: Position out of bounds");
    return;
  }




  DK.x = x;
  DK.y = y;
  DK.z = z;




 // int error = DK.inverse();
  //if (error != no_error) {
   // Serial.print("Error: ");
    //Serial.println(error);
    //return;
 // }




  if (DK.a < THETA1_MIN || DK.a > THETA1_MAX || DK.b < THETA2_MIN || DK.b > THETA2_MAX || DK.c < THETA3_MIN || DK.c > THETA3_MAX) {
    Serial.println("Error: Motor angle out of bounds");
    return;
  }




  setMotors();
}




void setMotors() {
  double A = DK.a * Multiplier;
  double B = DK.b * Multiplier;
  double C = DK.c * Multiplier;




  Serial.println(DK.a);
  Serial.println(DK.b);
  Serial.println(DK.c);
  Serial.println(DK.x);
  Serial.println(DK.y);
  Serial.println(DK.z);
  Serial.println();




  stepperA.moveTo(A);
  stepperB.moveTo(B);
  stepperC.moveTo(C);




  while (stepperA.distanceToGo() != 0 || stepperB.distanceToGo() != 0 || stepperC.distanceToGo() != 0) {
    stepperA.run();
    stepperB.run();
    stepperC.run();
  }
}








void testWorkSpace() {
  // Define an array of test positions (x, y, z)
  double testPositions[][3] = {
    {X_MAX, Y_MAX, Z_MAX},
    {X_MIN, Y_MIN, Z_MIN},
    {X_MAX, Y_MIN, Z_MIN},
    {X_MIN, Y_MAX, Z_MAX},
    {HOME_X, HOME_Y, Z_MAX},
    {HOME_X, HOME_Y, Z_MIN},
    {HOME_X, Y_MAX, HOME_Z},
    {X_MAX, HOME_Y, HOME_Z},
    {HOME_X, HOME_Y, HOME_Z}  // Back to home position
  };


  int numPositions = sizeof(testPositions) / sizeof(testPositions[0]);


  // Loop through each test position
  for (int i = 0; i < numPositions; i++) {
    double x = testPositions[i][0];
    double y = testPositions[i][1];
    double z = testPositions[i][2];


    Serial.print("Testing position: ");
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(", Y: ");
    Serial.print(y);
    Serial.print(", Z: ");
    Serial.println(z);


    // Move to the test position
    moveToPosition(x, y, z);


    // Delay to observe the movement
    delay(1000);


    // Check limit switch status
    checkLimitSwitches();
  }


  Serial.println("Workspace test complete.");
}


void checkLimitSwitches() {
  if (digitalRead(ALimitPin) == HIGH) {
    Serial.println("A Limit Switch: Activated");
    stepperA.stop();  // Stop the motor
  } else {
    Serial.println("A Limit Switch: Not Activated");
  }


  if (digitalRead(BLimitPin) == HIGH) {
    Serial.println("B Limit Switch: Activated");
    stepperB.stop();  // Stop the motor
  } else {
    Serial.println("B Limit Switch: Not Activated");
  }


  if (digitalRead(CLimitPin) == HIGH) {
    Serial.println("C Limit Switch: Activated");
    stepperC.stop();  // Stop the motor
  } else {
    Serial.println("C Limit Switch: Not Activated");
  }


  Serial.println("------------------------");
}




void circleHome() {
  // Move Delta Robot to home position
  HomeMachine();
  delay(3000);




  // Perform circular motion at home
  int radius = 50;       // Radius of the circular motion (adjust as needed)
  int centerX = HOME_X;  // Center X coordinate of the circle
  int centerY = HOME_Y;  // Center Y coordinate of the circle
  int steps = 100;       // Number of steps to complete the circle (adjust as needed)




  // Move in a circular path
  for (int i = 0; i < steps; i++) {
    double angle = 2 * PI * i / steps;
    int x = centerX + radius * cos(angle);
    int y = centerY + radius * sin(angle);
    moveToPosition(x, y, -300);
    delay(50);  // Adjust delay for smooth motion
  }
}






// Move to the stored home position
void HomeMachine() {
  bool aHomed = false;
  bool bHomed = false;
  bool cHomed = false;


  while (!aHomed || !bHomed || !cHomed) {
    if (!aHomed && digitalRead(ALimitPin) == HIGH) {
      stepperA.move(-20);  // Move in the direction to trigger the limit switch
      stepperA.run();
    } else {
      aHomed = true;
    }


    if (!bHomed && digitalRead(BLimitPin) == HIGH) {
      stepperB.move(-20);  // Move in the direction to trigger the limit switch
      stepperB.run();
    } else {
      bHomed = true;
    }


    if (!cHomed && digitalRead(CLimitPin) == HIGH) {
      stepperC.move(-20);  // Move in the direction to trigger the limit switch
      stepperC.run();
    } else {
      cHomed = true;
    }
  }
 
  stepperA.setCurrentPosition(HomeAngle * Multiplier);
  stepperB.setCurrentPosition(HomeAngle * Multiplier);
  stepperC.setCurrentPosition(HomeAngle * Multiplier);


  Serial.println("Homing complete, current position set to home");
}

void moveObject(double destX, double destY, double destZ) {
  // Move to the center position
   moveToPosition(0, 0, -430);
  //delay(1000);

  // Print current position
  Serial.println("At home position, preparing to move to destination");

  // Move to the specified destination position
  moveToPosition(0, 0, -330);
  moveToPosition(destX, destY, destZ);
  delay(1000);
  Serial.println("Reached destination position, simulating interaction");
  //delay(1000);

  // Simulate interaction (e.g., pick up object)
  // Add any specific interaction logic here if needed
  Serial.println("Interaction complete, returning to home position");
  //moveToPosition(0, 0, -330);
  //delay(500);
  moveToPosition(0, 0, -430);
  // Move back to the home position
  Serial.println("Object moved back to home position");
}







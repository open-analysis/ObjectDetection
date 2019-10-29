// File:			RoboVision.cpp
// Date:			10/18/19
// Description:		Program will allow an e-puck robot to "see" its environment to guide itself through it (eg navigating a cave or tunnel).
// Author:			Josh Chica
// Modifications:	           Many

#include <stdio.h>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>

// Macros
#define TIME_STEP 64
#define MAX_SPEED 6.28

// namespaces
using namespace webots;

// forward declarations
void checkSpeed(double &, double &);

// global variables
const double MIN_DIST = 90.0;


// MAIN
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  // int timeStep = (int)robot->getBasicTimeStep();

  // init'ing sensors
  DistanceSensor *ps[8];
  char psNames[8][4] = { "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7" };
  
  for (int i = 0; i < 8; i++) {
    ps[i] = robot->getDistanceSensor(psNames[i]);
    if (ps[i] == NULL){
      printf("Distance Sensor not found\n");
      return -1;
    }
    ps[i]->enable(TIME_STEP);
  }
  
  // init'ing actuators
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  // setting the speed that will be assigned to the motors
  double leftSpeed = .5 * MAX_SPEED;
  double rightSpeed = .5 * MAX_SPEED;
  
  // Main loop:
  while (robot->step(TIME_STEP) != -1) {
  
    //printf("Motor Speeds: %.2f, %.2f\n", leftSpeed, rightSpeed);

    // Read the sensors:
    double psValues[8];
    for (int i = 0; i < 8; i++)
      psValues[i] = ps[i]->getValue();

    // Process sensor data here:
  
    bool front_obstacle = psValues[0] > MIN_DIST || psValues[7] > MIN_DIST;
    bool left_obstacle = psValues[4] < MIN_DIST || psValues[5] < MIN_DIST || psValues[6] < MIN_DIST;
    bool right_obstacle = psValues[1] < MIN_DIST || psValues[2] < MIN_DIST || psValues[3] < MIN_DIST;

    printf("Front Sensors: %.3f, %.3f\n", psValues[0], psValues[7]);    
    // determines if there's something in front of the epuck
    if (front_obstacle){
      printf("Something's infront of me\n");
      // determines if it's closer to the right side
      if (psValues[0] > psValues[7]){
        // checks to see if the left side is open to move
        if (left_obstacle){
          printf("Gonna go left\n");
          leftSpeed += 0.5 * MAX_SPEED;
          rightSpeed -= 0.5 * MAX_SPEED;
        }
       // determines if it's closer to the left side
      } else if (psValues[0] < psValues[7]){
        // checks to see if the right side is open to move
        if (right_obstacle){
          printf("Gonna go right\n");
          leftSpeed -= 0.5 * MAX_SPEED;
          rightSpeed += 0.5 * MAX_SPEED;
        }
      }
      // if it's perfectly infront of the epuck checks left then right, otherwise quits
      else{
        
        if (left_obstacle){
          printf("Gonna go left\n");
          leftSpeed += 0.5 * MAX_SPEED;
          rightSpeed -= 0.5 * MAX_SPEED;
        }
        else if (right_obstacle){
          printf("Gonna go right\n");
          leftSpeed -= 0.5 * MAX_SPEED;
          rightSpeed += 0.5 * MAX_SPEED;
        }
        else
          break;
      }
    }

    // making sure the velocities set aren't higher than the max
    checkSpeed(leftSpeed, rightSpeed);    
    // Actuator commands:
      
    // actually setting the motors' velocities based on the above data
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
    
  };

  // Enter here exit cleanup code.
  printf("Done\n Deleting robot\n");
  delete robot;
  return 0;
}


void checkSpeed(double &leftSpeed, double &rightSpeed){
  if (leftSpeed > MAX_SPEED)
    leftSpeed = MAX_SPEED;
  if (rightSpeed > MAX_SPEED)
    rightSpeed = MAX_SPEED;
}
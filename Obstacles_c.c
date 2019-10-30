// File:			RoboVision.cpp
// Date:			10/29/19
// Description:		Program will allow an e-puck robot to "see" its environment to guide itself through it (eg navigating a cave or tunnel).
// Author:			Josh Chica
// Modifications:	           Many

#include <stdio.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>

// Macros
#define TIME_STEP 64
#define MAX_SPEED 6.28

// forward declarations
void checkSpeed(double *, double *);

// global variables
const double MIN_DIST = 100.0;
const double SPD_MULT = 0.75;


// MAIN
int main(int argc, char **argv) {
  // create the Robot instance.
  wb_robot_init();

  // get the time step of the current world.
  // int timeStep = (int)robot->getBasicTimeStep();

  // init'ing sensors
  WbDeviceTag ps[8];
  char psNames[8][4] = { "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7" };
  
  for (int i = 0; i < 8; i++) {
    ps[i] = wb_robot_get_device(psNames[i]);
    wb_distance_sensor_enable(ps[i], 1);
  }
  
  // init'ing actuators
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_velocity(left_motor, .3*MAX_SPEED);
  wb_motor_set_velocity(right_motor, .3*MAX_SPEED);
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  
  // setting the speed that will be assigned to the motors
  double leftSpeed = .5 * MAX_SPEED;
  double rightSpeed = .5 * MAX_SPEED;
  
  // Main loop:
  while (wb_robot_step(TIME_STEP) != -1) {
  
    //printf("Motor Speeds: %.2f, %.2f\n", leftSpeed, rightSpeed);

    // Read the sensors:
    double psValues[8];
    for (int i = 0; i < 8; i++)
      psValues[i] = wb_distance_sensor_get_value(ps[i]);

    // Process sensor data here:
  
    bool front_obstacle = (psValues[0] > MIN_DIST || psValues[7] > MIN_DIST);
    bool left_obstacle = (psValues[4] < MIN_DIST || psValues[5] < MIN_DIST || psValues[6] < MIN_DIST);
    bool right_obstacle = (psValues[1] < MIN_DIST || psValues[2] < MIN_DIST || psValues[3] < MIN_DIST);

    printf("Front Sensors: %.3f, %.3f\n", psValues[0], psValues[7]);    
    // determines if there's something in front of the epuck
    if (front_obstacle){
      printf("Something's infront of me\n");
      // determines if it's closer to the right side
      if (psValues[0] > psValues[7]){
        // checks to see if the left side is open to move
        if (left_obstacle){
          printf("Gonna go left\n");
          leftSpeed += (SPD_MULT * MAX_SPEED);
          rightSpeed -= (SPD_MULT * MAX_SPEED);
        } else if (right_obstacle){
          printf("Gonna go right\n");
          leftSpeed -= (SPD_MULT * MAX_SPEED);
          rightSpeed += (SPD_MULT * MAX_SPEED);
        }
       // determines if it's closer to the left side
      } else if (psValues[0] < psValues[7]){
        // checks to see if the right side is open to move
        if (right_obstacle){
          printf("Gonna go right\n");
          leftSpeed -= (SPD_MULT * MAX_SPEED);
          rightSpeed += (SPD_MULT * MAX_SPEED);
        } else if (left_obstacle){
          printf("Gonna go left\n");
          leftSpeed += (SPD_MULT * MAX_SPEED);
          rightSpeed -= (SPD_MULT * MAX_SPEED);
        }
      }
      // if it's perfectly infront of the epuck checks left then right, otherwise quits
      else{
        
        if (left_obstacle){
          printf("Gonna go left\n");
          leftSpeed += (SPD_MULT * MAX_SPEED);
          rightSpeed -= (SPD_MULT * MAX_SPEED);
        }
        else if (right_obstacle){
          printf("Gonna go right\n");
          leftSpeed -= (SPD_MULT * MAX_SPEED);
          rightSpeed += (SPD_MULT * MAX_SPEED);
        }
        else
          break;
      }
    } else{
      leftSpeed = 0.5 * MAX_SPEED;
      rightSpeed = 0.5 * MAX_SPEED;
    }

    // making sure the velocities set aren't higher than the max
    double *left = &leftSpeed;
    double *right = &rightSpeed;
    printf("Before: %f, %f\t", leftSpeed, rightSpeed);
    checkSpeed(left, right);    
    printf("After: %f, %f\n", leftSpeed, rightSpeed);
    
    // Actuator commands:
      
    // actually setting the motors' velocities based on the above data
    wb_motor_set_velocity(left_motor, leftSpeed);
    wb_motor_set_velocity(right_motor, rightSpeed);
    
  };

  // Enter here exit cleanup code.
  printf("Done\n Deleting robot\n");
  wb_robot_cleanup();
  return 0;
}


void checkSpeed(double *leftSpeed, double *rightSpeed){
  if (*leftSpeed > MAX_SPEED)
    *leftSpeed = MAX_SPEED;
  if (*rightSpeed > MAX_SPEED)
    *rightSpeed = MAX_SPEED;
}

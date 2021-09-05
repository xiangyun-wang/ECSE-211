package ca.mcgill.ecse211.final_project.Odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import static ca.mcgill.ecse211.final_project.finalproject.Resources.*;

/**
 * This class is used to localize the robot while it is traveling on the field. 
 * It can tell the robot what its current location. 
 * 
 * @author Xiangyun Wang
 * @author Younggue Kim
 *
 */

public class Odometer extends OdometerData implements Runnable {
  /**
   * Data of Odometer
   */
  public static OdometerData odoData;
  
  /**
   * Odometer of the robot
   */
  public static Odometer odo = null; // Returned as singleton

  /**
   * Tacho Count of the left motor
   */
  private int leftMotorTachoCount;
  
  /**
   * Tacho Count of the right motor
   */
  private int rightMotorTachoCount;
  
  /**
   * Current direction of the robot
   */
  private static double theta;
  
  /**
   * Old tacho count of the left motor
   */
  private int old_lmtc = 0;
  
  /**
   * Old tacho count of the right motor
   */
  private int old_rmtc = 0;
  
  /**
   * Displacement of the left motor
   */
  private double displacement_L;
  
  /**
   * Displacement of the right motor
   */
  private double displacement_R;
  
  /**
   * The updating period of the odometer
   */
  private static final long ODOMETER_PERIOD = 30; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param LEFT_MOTOR	left motor of the robot
   * @param RIGHT_MOTOR right motor of the robot 
   * @throws OdometerExceptions 
   */
  private Odometer() throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation method

    // Reset the values of x, y and z to 0
    //odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param LEFT_MOTOR left motor of the robot
   * @param RIGHT_MOTOR right motor of the robot
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer();
      return odo;
    }
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    while (true) {
    	double dx, dy, dt, angle, dd;
      updateStart = System.currentTimeMillis();

      leftMotorTachoCount = LEFT_MOTOR.getTachoCount();
      rightMotorTachoCount = RIGHT_MOTOR.getTachoCount();

      displacement_L = Math.PI*WHEEL_RAD*(leftMotorTachoCount - old_lmtc)/180;
      displacement_R = Math.PI*WHEEL_RAD*(rightMotorTachoCount - old_rmtc)/180;
      
      old_lmtc = leftMotorTachoCount;
      old_rmtc = rightMotorTachoCount;
      
      dd = (displacement_L+displacement_R)/2;
      angle = (displacement_L-displacement_R)/TRACK; 
      theta = OdometerData.theta+Math.toDegrees(angle);
      //System.out.println(theta);
      dx = Math.sin(Math.toRadians(theta)) * dd;
      dy = Math.cos(Math.toRadians(theta)) * dd;
      dt = angle*180/Math.PI;
      
      odo.update(dx, dy, dt);

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }
  
  /**
   * This method gets two motors in an array
   * @return two motor of the robot
   */
  public EV3LargeRegulatedMotor[] getMotor() {
	  EV3LargeRegulatedMotor[] motors = {LEFT_MOTOR, RIGHT_MOTOR};
	  return motors;
  }
  
  /**
   * This method sets the speed for both motors
   * @param target speed of the robot
   */
  public void setMotorSpeeds (int speed) {
	  LEFT_MOTOR.setSpeed(speed);
	  RIGHT_MOTOR.setSpeed(speed);
  }
  
  /**
   * This method makes the robot move forward
   */
  public void moveforward() {
	  LEFT_MOTOR.forward();
	  RIGHT_MOTOR.forward();
  }
  
  /**
   * This method makes the robot stop
   */
  public void stop() {
	  LEFT_MOTOR.stop(true);
	  RIGHT_MOTOR.stop();
  }
  
  /**
   * This method makes the robot move backwards
   */
  public void movebackward() {
	  LEFT_MOTOR.backward();
	  RIGHT_MOTOR.backward();
  }
  
  /**
   * This method makes the robot rotate clockwise if 
   * the boolean is true or counterclockwise if the boolean is false
   * 
   * @param dir determine the direction to spin (true for clockwise, false for counterclockwise)
   */
  public void spin(boolean dir) {
	  if(dir) {
		  LEFT_MOTOR.forward();
		  RIGHT_MOTOR.backward();
	  }else {
		  LEFT_MOTOR.backward();
		  RIGHT_MOTOR.forward();
	  }
  }
  
}

package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 30;

  private final int bandCenter;
  private final int bandWidth;
  private int distance=20;
  private int filterControl;

  private static final int MAXCHANGE_left = 45;				//set a maximum change of speed to the left wheel
  private static final int MAXCHANGE_right=160;				//set a maximum change of speed to the right wheel
  private static int  error = 0;							//use to hold the error of the car, preset to 0
  private static int calc_change;							//the calculated change of speed
  private static final int pconstant = 4;					//p-constant
  
  
  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 100 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 200) {				//limit the maximum distance that can be passed to the following program
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
    	//if(distance>200) {						//if greater than 200, set it to 200
    		this.distance = 200;
    	//}else {
    		//this.distance = distance;
    	//}
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }
    
    //
    
    error = this.distance - this.bandCenter;				//error between the actual path and the idea path
    int abs = Math.abs(error);								//absolute value of the error
    if(error>=this.bandWidth) {							//change it to bandwidth?
    	calc_change = calc_correction(abs);			//left turn
    	if(calc_change>MAXCHANGE_left) {			//limit the change of the speed not be too big
    		calc_change = MAXCHANGE_left;
    	}
    	/*if(this.distance>=100&&check_front==false) {
    		WallFollowingLab.leftMotor.rotate(45);
    		WallFollowingLab.rightMotor.rotate(-45);
    		check_front = true;
    	}else if(this.distance>=100&&check_front==true) {
			WallFollowingLab.leftMotor.rotate(-30);
			WallFollowingLab.rightMotor.rotate(30);
			check_front = false;
		}*/
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED-calc_change-40);	//minus a constant so that it can curve
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
	    WallFollowingLab.rightMotor.forward();
    }else{
    	calc_change = 3*calc_correction(abs);			//right turn, we times an extra 3 to the calc_correction, the p-constant for making right turn might not be the same as making a left turn
    	
    	if(calc_change>=MAXCHANGE_right) {				//limit the change of the speed not be too big
    		calc_change = MAXCHANGE_right;
    		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED/2);			//make a great change slowing, otherwise might miss the data received, and affect the detection of the wall
    		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED/2);
    		WallFollowingLab.leftMotor.forward();
    	    WallFollowingLab.rightMotor.backward();
    	}else {
    		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED+60);					//add an extra 60 to the left motor, and minus an extra 40 degrees to the right motor, so the car will shake and have a greater chance of detecting the front wall
    		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED-calc_change-40);
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.forward();
    	}
    }
    //
    // TODO: process a movement based on the us distance passed in (P style)
  }
  
  
  //this method is used to calculate the change of each wheel using the p-constant
  private int calc_correction(int input) {
	  int correct = 0;
	  correct = pconstant * input;
	  return correct;
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}

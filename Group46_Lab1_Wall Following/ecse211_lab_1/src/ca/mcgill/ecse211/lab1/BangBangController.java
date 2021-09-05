package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;
  
  private int filtercontrol=0;
  private int filterout = 40;

  
  
  

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    //WallFollowingLab.leftMotor.setSpeed(0); // Start robot moving forward
    //WallFollowingLab.rightMotor.setSpeed(0);
    //WallFollowingLab.leftMotor.forward();
    //WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
    //filter to the received data
    if(this.distance >= 100&&filtercontrol<filterout) {
    	filtercontrol++;
    }else if(distance>=100) {
    	this.distance = distance;
    }else {
    	filtercontrol = 0;
    	this.distance = distance;
    }
    
    int error = this.distance-bandCenter;			//calculate the error of the car
    int abs = Math.abs(error);						//take the absolute value of the error
    
    if(abs<8&&abs>bandwidth) {						//under the condition of out of the error tolerance but within positive or negative 10
    	if(error<0) {								//if error is negative, rotate the car to the right, then move forward again
    		WallFollowingLab.leftMotor.setSpeed(motorHigh+50);
    		WallFollowingLab.rightMotor.setSpeed(motorHigh+50);
    		//WallFollowingLab.leftMotor.rotate(40);
    		//WallFollowingLab.rightMotor.rotate(-40);
    		WallFollowingLab.leftMotor.forward();
    	    //WallFollowingLab.rightMotor.forward();
    		
    	    //WallFollowingLab.rightMotor.backward();
    		//WallFollowingLab.leftMotor.forward();
    	    WallFollowingLab.rightMotor.backward();
    	}else {									//if the error is positive, make the car to move towards left
    		WallFollowingLab.leftMotor.setSpeed(motorLow+15);
    		WallFollowingLab.rightMotor.setSpeed(motorHigh);
    		WallFollowingLab.leftMotor.forward();
    	    WallFollowingLab.rightMotor.forward();
    	}
    }else if(abs>=8&&error<0) {				//if it is getting too close to the wall, make a great turn by rotating the left wheel forward and the right wheel backward
    	WallFollowingLab.leftMotor.setSpeed(motorHigh);
    	WallFollowingLab.rightMotor.setSpeed(motorHigh);
    	WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.backward();
    		//backward
    }else if(abs>=8&&error>0){					//if it is getting too far away from the wall, doing the same as adjusting the the left
    	WallFollowingLab.leftMotor.setSpeed(motorLow+25);
    	WallFollowingLab.rightMotor.setSpeed(motorHigh);
    	WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
    }else {										//otherwise do not make any changes to the wheels
    	/*
    		WallFollowingLab.leftMotor.setSpeed(motorHigh);	
    		WallFollowingLab.rightMotor.setSpeed(motorHigh);		
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.forward();
    		*/
    	}
  }
    
   

  
    
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
  

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}

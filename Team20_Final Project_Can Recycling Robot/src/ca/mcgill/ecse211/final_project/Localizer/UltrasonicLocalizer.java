package ca.mcgill.ecse211.final_project.Localizer;

import static ca.mcgill.ecse211.final_project.finalproject.Resources.*;

import ca.mcgill.ecse211.final_project.Navigation.Navigation;

/**
 * 
 * This class uses ultrasonic sensor to correct the angle of the robot. 
 * 
 * @author Xiangyun Wang
 * @author Younggue Kim
 *
 */

public class UltrasonicLocalizer {
	
	/**
	 * Speed of rotation while doing the localization
	 */
	private static final int ROTATE_SPEED = 300;
  	
  	/**
  	 * Direction of the robot when first time detecting a wall
  	 */
	static double angle1;
	
	/**
	 * Direction of the robot when second time detecting a wall
	 */
	static double angle2;
	
	/**
	 * Whether or not extra angle is needed to turn, so that misreading of the ultrasonic parameter can be avoided
	 */
  	static boolean need_extra = false;
  	
  	/**
  	 * The method falling Edge is used when the robot is NOT facing the wall. 
  	 * In that case, it will check for a reduction of distance and sweep for angle
  	 * @throws InterruptedException 
  	 */
  	public static void fallingEdge() throws InterruptedException {
  		LEFT_MOTOR.setSpeed(ROTATE_SPEED);
  		RIGHT_MOTOR.setSpeed(ROTATE_SPEED);
  		Thread.sleep(1000);
  		while(myusData.getDistance() < 40 ) {			//while distance is below 30cm (if the robot is not well located)
  			
  			RIGHT_MOTOR.backward();		//rotate
  			LEFT_MOTOR.forward();
  			need_extra = true;
  		}
  		if(need_extra) {
  			RIGHT_MOTOR.rotate(-100,true);
  	  		LEFT_MOTOR.rotate(100);
  		}
  
  		while(myusData.getDistance() > 35 ) {			//then, while distance stays over 30cm
  			RIGHT_MOTOR.backward();		//still rotate
  			LEFT_MOTOR.forward();
  			
  		}
  		
  		RIGHT_MOTOR.stop(true);			//still rotate
		LEFT_MOTOR.stop();
  		
  		angle1 = (odo.getXYT())[2];		//note angle
  		
  		while(myusData.getDistance() < 40) {			//keep rotating in the other way
  			
  			RIGHT_MOTOR.forward();
  			LEFT_MOTOR.backward();
  		}
  		
  		RIGHT_MOTOR.rotate(100,true);		//still rotate
		LEFT_MOTOR.rotate(-100);
  		
		
  		while(myusData.getDistance() > 35)  {
  			
  			RIGHT_MOTOR.forward();			
  			LEFT_MOTOR.backward();
  		}
  		LEFT_MOTOR.stop(true);
  		RIGHT_MOTOR.stop();
  		
  		angle2 = (odo.getXYT())[2];			//get second angle
  		double angle_correction;
  		if(angle1<angle2) {					//calculate correction angle
			angle_correction = (225-(angle1 + angle2)/2);	//225 needs to be tested
		}else {
			angle_correction = (45 - (angle1 + angle2)/2);	//45 needs to be tested
		}
		
		odo.setTheta(odo.getXYT()[2]+angle_correction);		//update odometer
  		Navigation.turnTo(0);
  		
  	}
  	
  	
  	 /**
  	  * The method getData gets the distance from ultrasonic sensor
  	  * @return result data from the ultrasonic sensor
  	  */
  	/*
  	 public static float getData() {

  		 	US_VALUE.fetchSample(US_DATA, 0);
  			float distance = (int)(US_DATA[0]*100.0);
  			float result = 0;
  			if (distance > 50 && filterControl < filterOut) {
  				filterControl ++;
  				result = lastDistance;
  			} else if (distance > 255){
  				result = 50; 
  			} else {
  				filterControl = 0;
  				result = distance;
  			}
  			lastDistance = distance;
  			
  			return result;
  		}
  	 */
}

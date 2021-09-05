package ca.mcgill.ecse211.final_project.Localizer;

import static ca.mcgill.ecse211.final_project.finalproject.Resources.*;

import ca.mcgill.ecse211.final_project.Navigation.Navigation;
import ca.mcgill.ecse211.final_project.finalproject.FinalProject;


/**
 * 
 * This class uses light sensor to localize the position of the robot. 
 * It also do angle correction, just in case that the ultrasonic localization does not work perfectly. 
 * 
 * @author Xiangyun Wang
 *
 */

public class LightLocalizer_new {
	
	/**
	 * The following method is divided in three parts:
	 * First compare the intensity of the floor and the intensity of what the light sensor is scanning.
	 * If the difference between the color of the floor and what the light sensor is currently measuring is smaller than 100
	 * then both readings are on the floor. If the difference is more than 100, the light sensor is on a black line.
	 * 
	 * Second, we check the values of the light sensor while the robot is going backwards:
	 * if one of the two light sensors detects a black line, we make the wheel on its side stop.
	 * the robot keeps turning backwards until the remaining light sensor also detects a black line. This allows the robot to 
	 * correct its angle and its position relatively to the chosen axis.
	 * 
	 * Thirdly, the same process is repeated after the robot turns to the direction along the other axis.
	 * 
	 * Finally, according to the starting corner, the angle and position. 
	 * @throws InterruptedException
	 */
	public static void localize() throws InterruptedException {
		LEFT_MOTOR.setSpeed(150);
		RIGHT_MOTOR.setSpeed(150);
		FinalProject.line_correction(150);
		odo.setY(LS_LENGTH);
		odo.setTheta(0);
		Navigation.goBackward(LS_LENGTH);
		turnTo(90);
		FinalProject.line_correction(150);
		odo.setX(LS_LENGTH);
		odo.setTheta(90);
		Navigation.goBackward(LS_LENGTH);
		turnTo(0);
		corner_correct();
	}
	
	
	
	/** 
	 * Takes the new heading as input and make the robot turn to it
	 * 
	 * @param: double theta that represents an angle in radians
	 */
	public static void turnTo(double theta) {
		
		double angle = getMinAngle(theta-odo.getXYT()[2]);
		
		LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, angle),true);
		RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, angle),false);
	}

	/**
	 * Gets the minimum value between 180 and negative 180. 
	 * @param angle turning angle
	 * @return angle minimum turning angle
	 */
	public static double getMinAngle(double angle){
		if (angle > 180) {  //Pi = 180 degrees
			angle -= 2*180; 
		} else if (angle < -180) {
			angle = angle + 2*180;
		}
		return angle;
	}
	
	/**
	 * 
	 * This method takes the radius of wheels and the distance that needs to be traveled, and calculate 
	 * the rotation of the wheels need to make to travel that distance
	 * 
	 * @param radius is the wheel radius
	 * @param distance is the distance needs to be traveled
	 * @return rotation of the wheels
	 */
	private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * This method take the radius of wheels, width of the robot and the angle needs to be rotated by the robot, 
	 * to calculate the rotation needs to make by wheels to turn a certain angle to a certain direction
	 * 
	 * @param radius is the radius of wheels
	 * @param width is the width of the robot
	 * @param angle is the angle needs to be turned by the robot
	 * @return rotation of wheels
	 */
	private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	/**
	 * This method gets the intensity measured by the two light sensors
	 * we record these values in an array with the slot 0 for the left 
	 * light sensor and 1 for the right light sensor.
	 * @return color
	 */
	public static int[] getLightData() {
		LIGHT_VALUE_L.fetchSample(lightDataL,0);
		LIGHT_VALUE_R.fetchSample(lightDataR,0);
		int[] color = new int[2];
		color[0] = (int)(lightDataL[0] * 1000);
		color[1] = (int)(lightDataR[0] * 1000);
		
		return color;
	}
	
	/**
	 * This method correct the odometer by Corner parameter, which is specified using wifi class. 
	 */
	public static void corner_correct(){
		if(Corner == 0) {
			odo.setXYT(TILE_LENGTH, TILE_LENGTH, 0);
		}else if(Corner == 1) {
			odo.setXYT(14*TILE_LENGTH, TILE_LENGTH, 270);
		}else if (Corner == 2){
			odo.setXYT(14*TILE_LENGTH, 8*TILE_LENGTH, 180);
		}else {
			odo.setXYT(TILE_LENGTH, 8*TILE_LENGTH, 90);
		}
	}
}


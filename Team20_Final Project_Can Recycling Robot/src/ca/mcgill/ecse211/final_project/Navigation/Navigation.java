package ca.mcgill.ecse211.final_project.Navigation;

import static ca.mcgill.ecse211.final_project.finalproject.Resources.*;

/**
 * This class contains methods which can make robot travel on the field. 
 * 
 * @author Xiangyun Wang
 * @author Younggue Kim
 *
 */

public class Navigation extends Thread{

	public static void headForward(int speed) {
		LEFT_MOTOR.setSpeed(speed);
		RIGHT_MOTOR.setSpeed(speed);
		LEFT_MOTOR.forward();
		RIGHT_MOTOR.forward();
	}
	
	/**
	 * The following method is used to make the robot go forward
	 * @param x amount of distance to go forward
	 * 
	 */
	public static void goForward(double x, boolean immediate) {
		LEFT_MOTOR.setSpeed(300);
		RIGHT_MOTOR.setSpeed(300);
		LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD,x),true);
		RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD,x),immediate);
	}
	
	/**
	 * The following method is used to make the robot go to backwards
	 * @param lsLength amount of distance to go backward
	 */
	public static void goBackward(double lsLength) {
		LEFT_MOTOR.setSpeed(200);
		RIGHT_MOTOR.setSpeed(200);
		LEFT_MOTOR.rotate(-convertDistance(WHEEL_RAD,lsLength),true);
		RIGHT_MOTOR.rotate(-convertDistance(WHEEL_RAD,lsLength));
	}
	
	/**
	 * The following method is used to make the robot turn backwards
	 */
	public static void turnBackward() {
		LEFT_MOTOR.setSpeed(200);
		RIGHT_MOTOR.setSpeed(200);
		LEFT_MOTOR.rotate(-convertAngle(WHEEL_RAD,TRACK,180),true);
		RIGHT_MOTOR.rotate(convertAngle(WHEEL_RAD,TRACK,180));
	}
	
	/**
	 * The following method is used to make the robot go to a preset location
	 * We first calculate the distance that needs to be traveled 
	 * and the angle we are traveling towards
	 * by taking the difference between the position we want to go to and our
	 * current position
	 * 
	 * @param x x value of the target position
	 * @param y y value of the target position
	 * @param immediate whether or not to return immediately
	 * @param speed speed while traveling
	 */
	public static void travelTo(double x, double y, boolean immediate, int speed) {
	
		double dX = x - odo.getXYT()[0]; //remaining x distance
		double dY = y - odo.getXYT()[1]; //remaining y distance
		double turn_angle = Math.toDegrees(Math.atan2(dX, dY));
		turnTo(turn_angle);
		LEFT_MOTOR.setSpeed(speed);
		RIGHT_MOTOR.setSpeed(speed);
		//turnTo(turn_angle);
		
		double distance = Math.hypot(dX, dY);
		
		LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD,distance),true);
		RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD,distance),immediate);
	}
	
	
	/**
	 * This method allows us to get the minimum angle and turn to it so that our robot can travel to the next point
	 * by doing the difference between theta and the current angle
	 * 
	 * @param theta target direction
	 * 
	 */
	
	 public static void turnTo(double theta) {
		odo.setMotorSpeeds(150);
		double angle = getMinAngle(theta-odo.getXYT()[2]);
	
		LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, angle),true);
		RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, angle),false);
	}
	 
	 /**
	  * This method makes the robot turn clockwise or counter clockwise.
	  * @param dir direction to spin (true for clockwise, false for counterclockwise)
	  * @param angle amount of angle to spin
	  */
	 public static void spin(Boolean dir, int angle, int speed, boolean immediate){
		 odo.setMotorSpeeds(speed);
		 if(dir) {
			LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, angle),true);
			RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, angle),immediate);
		 }else {
			LEFT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, angle),true);
			RIGHT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, angle),immediate);
		 }
	 }

	 /**
	  * Gets the minimum value between 180 and negative 180. 
	  * @param angle input angle
	  * @return angle minimum angle to rotate
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
	
}

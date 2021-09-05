package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class SimpleNavigation extends Thread{
	private Odometer odometer;								//odometer for the navigation process
	private EV3LargeRegulatedMotor leftMotor;				//left motor of the robot
	private EV3LargeRegulatedMotor rightMotor;				//right motor of the robot			

	
	private static final EV3ColorSensor lSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));	//light sensor for making final correction
	  private SampleProvider lscolor = lSensor.getMode("Red");
	  private float[] lsData;
	
	static int distance; 
	private static final int ROTATE_SPEED = 150;
	private static final int FORWARD_SPEED = 120;
	public static final double WHEEL_RAD = 2.25; //wheel radius
	public static final double TRACK = 17.0;	//distance between two wheels
	private static boolean navigating = false;	//show navigation is running or not
	private static final double TILE_LENGTH = 30.48;	
	
	

	public void run() {
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(1000);
		}
		
		travelTo(Lab3.first_point[0]*TILE_LENGTH,Lab3.first_point[1]*TILE_LENGTH); //will be set
		travelTo(Lab3.second_point[0]*TILE_LENGTH,Lab3.second_point[1]*TILE_LENGTH); //will be set
		travelTo(Lab3.third_point[0]*TILE_LENGTH,Lab3.third_point[1]*TILE_LENGTH); //will be set
		travelTo(Lab3.forth_point[0]*TILE_LENGTH,Lab3.forth_point[1]*TILE_LENGTH); //will be set
		travelTo(Lab3.last_point[0]*TILE_LENGTH,Lab3.last_point[1]*TILE_LENGTH); //will be set
		turnTo(0);
		
	}
	public SimpleNavigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) throws OdometerExceptions{
		odometer = Odometer.getOdometer();
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

	}
	
	/*
	 * The following method is used to make the robot go to a preset location
	 */
	void travelTo(double x, double y) {
		
		navigating = true;
		
		double pathX = x-odometer.getXYT()[0];		//calculate x displacement 
		double pathY = y-odometer.getXYT()[1];		//calculate y displacement
		double path = Math.hypot(pathX, pathY);		//calculate path length
		double pathAngle;
		if(pathX<0&&pathY>=0) {						//calculate turning angle
			pathAngle = 360-(Math.toDegrees(Math.atan2(pathY,pathX))-90);
		}else {
			pathAngle = 90-Math.toDegrees(Math.atan2(pathY,pathX));
		}
		
		turnTo(pathAngle);							//turn to the direction angle
		
		
		
		leftMotor.setSpeed(FORWARD_SPEED);			//set speed
		rightMotor.setSpeed(FORWARD_SPEED);
		
		leftMotor.rotate(convertDistance(WHEEL_RAD,path),true);		//go to the directed point
		rightMotor.rotate(convertDistance(WHEEL_RAD,path),false);
		
		
		
		leftMotor.stop(true);							//stop the motor
		rightMotor.stop();
		navigating = false;
	}
	
	
	/*
	 * The following method is used to turn to the preset direction of the destination point
	 */
	void turnTo(double theta) {
		double turn_angle = theta-odometer.getXYT()[2];
		if (turn_angle > 180) {
			turn_angle = turn_angle-360;
		}else if(turn_angle<(-180)) {
			turn_angle = turn_angle+360;
		}
		leftMotor.setSpeed(ROTATE_SPEED);
	    rightMotor.setSpeed(ROTATE_SPEED);
	    
	    leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, turn_angle),true);
	    rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, turn_angle),false);
	    
	    leftMotor.stop(true);
	    rightMotor.stop();
	    
	}
	
	/*
	 * The following method is used to determine whether or not the robot is rotating or not
	 */
	boolean isNavigation() {
		return navigating;
	}
	
	
	/*
	 * This method is used to convert distance to rotating angle
	 */
	
	private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	/*
	 * This method is used to convert turning angle to rotating angle
	 */
	private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	
}

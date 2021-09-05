package ca.mcgill.ecse211.lab3;


import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class Navigation extends Thread{
	
	private Odometer odometer;								//odometer for the navigation process
	private EV3LargeRegulatedMotor leftMotor;				//left motor of the robot
	private EV3LargeRegulatedMotor rightMotor;				//right motor of the robot
	private SampleProvider us;								
	public static float[] usData;
	int bandCenter = 4;										//band center of p controller
	int bandWidth = 1;										//band width of p controller
	private double[] obstacle_position;						//position of the robot when it detects the block
	static double minimum_angle = 0;
	static double pangle=0;									//angle to turn when robot meets the blcok
	static int distance_from_wall = 5;						//distance for detecting block
	private double turnAngle;
	
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
			motor.stop();							//set the motor to stop
			motor.setAcceleration(1000);			//set acceleration
		}
		
		travelTo(Lab3.first_point[0]*TILE_LENGTH,Lab3.first_point[1]*TILE_LENGTH); //will be set
		travelTo(Lab3.second_point[0]*TILE_LENGTH,Lab3.second_point[1]*TILE_LENGTH); //will be set
		travelTo(Lab3.third_point[0]*TILE_LENGTH,Lab3.third_point[1]*TILE_LENGTH); //will be set
		travelTo(Lab3.forth_point[0]*TILE_LENGTH,Lab3.forth_point[1]*TILE_LENGTH); //will be set
		travelTo(Lab3.last_point[0]*TILE_LENGTH,Lab3.last_point[1]*TILE_LENGTH); //will be set
		turnTo(0);
		
	}
	
	/**
	 * constructor of the navigation class
	 * 
	 */
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, /*EV3MediumRegulatedMotor sensorMotor,*/ SampleProvider us, float[] usData) throws OdometerExceptions{
		odometer = Odometer.getOdometer();					//get odometer
		odometer.setXYT(0, 0, 0);
		this.leftMotor = leftMotor;							//get left motor
		this.rightMotor = rightMotor;						//get right motor
		//this.sensorMotor = sensorMotor;
		this.us = us;										
		this.usData = usData;

	}
	
	
	/*
	 * The following method is used to make the robot go to a preset location
	 */
	void travelTo(double x, double y) {
		
		navigating = true;							//whether or not the robot is navigating
		
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
		
		leftMotor.rotate(convertDistance(WHEEL_RAD,path),true);				//go to the directed point
		rightMotor.rotate(convertDistance(WHEEL_RAD,path),true);
		
		
		while(leftMotor.isMoving() || rightMotor.isMoving()) {				//when the robot is moving
			us.fetchSample(usData,0);										//scan if there is an obstacle
			distance = (int) (usData[0]*100);
			if(distance<=8) {												//if yes, stop
				leftMotor.stop(true);
				rightMotor.stop();
				navigating = false;
				obstacle_position = odometer.getXYT();						//record current position
			}
		}
		if(!navigating) {													//if there is an obstacle
			simple_avoid();													//avoid the obstacle
			navigating = true;												
			travelTo(x,y);													//keep going to the point
		}
		leftMotor.stop(true);												//when arrived, stop
		rightMotor.stop();
		navigating = false;
		
		
	}
	
	/*
	 * The following method is used to avoid object
	 */
	void simple_avoid() {
		turnAngle = obstacle_position[2] + 90;				//turn right 90 degrees
		if(turnAngle>=360) {
			turnAngle = turnAngle - 360;
		}else if(pangle<0) {
			turnAngle = turnAngle + 360;
		}
		turnTo(turnAngle);
		leftMotor.rotate(convertDistance(WHEEL_RAD,25),true);			//go straight for 25cm
		rightMotor.rotate(convertDistance(WHEEL_RAD,25),false);
		leftMotor.stop(true);											//stop
		rightMotor.stop();
		turnTo(obstacle_position[2]);									//turn back to the original direction
		leftMotor.rotate(convertDistance(WHEEL_RAD,43),true);			//go straight for 40 cm
		rightMotor.rotate(convertDistance(WHEEL_RAD,43),false);
		leftMotor.stop(true);											//stop
		rightMotor.stop();
		turnAngle = obstacle_position[2] - 90;							//turn left 90 degrees
		if(turnAngle>=360) {
			turnAngle = turnAngle - 360;
		}else if(pangle<0) {
			turnAngle = turnAngle + 360;
		}
		turnTo(turnAngle);
		while(!back_to_track()) {							//if not back to track, keep going
			leftMotor.forward();
			rightMotor.forward();
		}
		leftMotor.stop(true);								//when back to track, stop
		rightMotor.stop();
		
	}
	
	/*
	 * The following method is used to turn to the preset direction of the destination point
	 */
	void turnTo(double theta) {
		double turn_angle = theta-odometer.getXYT()[2];			//calculate angle needs to turn
		if (turn_angle > 180) {
			turn_angle = turn_angle-360;
		}else if(turn_angle<(-180)) {
			turn_angle = turn_angle+360;
		}
		leftMotor.setSpeed(ROTATE_SPEED);					//set motor speed
	    rightMotor.setSpeed(ROTATE_SPEED);
	    if(turn_angle==360) {
	    	turn_angle = 0;
	    }
	    leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, turn_angle),true);			//turn to that direction
	    rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, turn_angle),false);
	    
	    leftMotor.stop(true);							//stop the motor
	    rightMotor.stop();
	    
	}
	/*
	 * The following method is used to determine whether or not the robot is rotating or not
	 */
	boolean isNavigation() {
		return navigating;
	}
	
	/*
	 * the following method is used to determine if the robot is back to track or not
	 */
	boolean back_to_track() {
		double pathX = odometer.getXYT()[0]-obstacle_position[0];		//get current position
		double pathY = odometer.getXYT()[1]-obstacle_position[1];
		double pathAngle;
		if(pathX<0&&pathY>=0) {
			pathAngle = 360-(Math.toDegrees(Math.atan2(pathY,pathX))-90);		//calculate the angle of the obstacle position and current position
		}else {
			pathAngle = 90-Math.toDegrees(Math.atan2(pathY,pathX));
		}
		
		if(pathAngle<=(obstacle_position[2]+1) && pathAngle>=(obstacle_position[2]-1)) {		//if the angle is within the range near the original direction
			return true;																		//it means it is back to track
		}else {	
			return false;					//otherwise, no
		}	
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

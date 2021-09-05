package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class USLocalizer {
	public enum LocalizationOption {FALLING_EDGE, RISING_EDGE};		//two types of localization
	private static final float DETECT_WALL = 50;					//distance detects what there is a wall
	public static int ROTATION_SPEED = 70;							//motor speed
	
	
	private Odometer odo;										//odometer
	private SampleProvider usSensor;							//get ultrasonic sensor
	private float[] usData;										//ultrasonic sensor data
	private LocalizationOption localType;						
	private int distance = 50;
	private int count = 0;										//a variable to make sure there is a wall
	private double angle_correction;							//correction of angle to the odometer
	private boolean rotate_extra = false;						//to know whether or not the robot to rotate an extra angle
	
	/**
	 * constructor of USLocalizer class
	 * @param odo	is the odometer
	 * @param usSensor	is the ultrasonic sensor
	 * @param usData	is ultrasonic sensor data
	 * @param localType	is type of locating used
	 */
	public USLocalizer(Odometer odo, SampleProvider usSensor, float[] usData, LocalizationOption localType) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.localType = localType;
		
	}
	
	/**
	 * the following method does not return anything, but will do localization
	 */
	public void Localize() {
		
		//double[] position = new double[3];
		double angle_one, angle_two;		//two angles are recorded to make calculation
		
		if(localType == LocalizationOption.FALLING_EDGE) {		//when falling_edge is selected
			if(hasWall()) {										//if there is a wall, need to rotate extra
				rotate_extra = true;
			}
			odo.setMotorSpeeds(ROTATION_SPEED);					//set motor speed
			while(hasWall()) {									//if there is a wall, keep spinning
				odo.spin(true);
			}
			if(rotate_extra) {								//if rotate_extra = true, rotate extra
				odo.getMotor()[0].rotate(360,true);
				odo.getMotor()[1].rotate(-360);
				rotate_extra = false;
			}
			odo.setMotorSpeeds(0);							//set speed to 0
			
			try {Thread.sleep(500);}catch(InterruptedException e) {}		//sleep for 500ms
			
			while(!hasWall()) {								//if there is no wall
				odo.setMotorSpeeds(ROTATION_SPEED);			//rotating clockwise
				odo.spin(true);
			}
			odo.setMotorSpeeds(0);							//set speed to 0
			Sound.beep();									//beep	
			angle_one = odo.getXYT()[2];					//record the angle
			
			odo.setMotorSpeeds(ROTATION_SPEED);				//set speed
			while(hasWall()) {								//if there is wall, rotate counterclockwise
				odo.spin(false);
			}
			
			while(!hasWall()) {								//keep rotating counterclockwise
				odo.setMotorSpeeds(ROTATION_SPEED);
				odo.spin(false);
			}
			odo.setMotorSpeeds(0);							//set speed to 0
			Sound.beep();									//beep
			angle_two = odo.getXYT()[2];					//get the second angle
			
			
			if(angle_one<angle_two) {						//calculate correction angle
				angle_correction = (225-(angle_one + angle_two)/2);		//225 needs to be tested
			}else {
				angle_correction = (45 - (angle_one + angle_two)/2);	//45 needs to be tested
			}
			
			odo.setTheta(odo.getXYT()[2]+angle_correction);			//update odometer
			
		}else {										//if rising edge is selected
			odo.setMotorSpeeds(ROTATION_SPEED);		//set speed
			if(!hasWall()) {						//if no wall, need to rotate extra angle
				rotate_extra = true;
			}
			while(!hasWall()) {						//if no wall, robot spin clockwise
				odo.spin(true);
			}
			if(rotate_extra) {						//need to rotate extra, rotate extra
				odo.getMotor()[0].rotate(360,true);
				odo.getMotor()[1].rotate(-360);
				rotate_extra = false;
			}
			while(hasWall()) {						//if there is a wall
				odo.setMotorSpeeds(ROTATION_SPEED);		//keep rotating clockwise
				odo.spin(true);
			}
			odo.setMotorSpeeds(0);						//set speed to 0
			Sound.beep();								//beep
			angle_one = odo.getXYT()[2];				//record angle one
			
			odo.setMotorSpeeds(ROTATION_SPEED);			//rotate counterclockwise
			while(!hasWall()) {
				odo.spin(false);
			}
			
			while(hasWall()) {							//if there is a wall, keep rotating
				odo.setMotorSpeeds(ROTATION_SPEED);
				odo.spin(false);
			}
			odo.setMotorSpeeds(0);						//set speed to 0
			Sound.beep();								//beep
			angle_two = odo.getXYT()[2];				//get second angle
			
			if(angle_one<angle_two) {						//calculate correction
				angle_correction = (45-(angle_one + angle_two)/2);   //225 needs to be tested
			}else {
				angle_correction = (225 - (angle_one + angle_two)/2);	//45 needs to be tested
			}
			
			odo.setTheta(odo.getXYT()[2]+angle_correction);			//update odometer
		}
	}
	
	/**
	 * a method used to determine if there is a wall
	 * @return true if there is a wall, false if there is not a wall
	 */
	private boolean hasWall() {
		return (getFilteredData() < DETECT_WALL);
	}
	
	
	/**
	 * this method is used to get data from ultrasonic data
	 * @return filtered data
	 */
	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		if(usData[0]*100>=DETECT_WALL&&count<=3) {
			count++;
			return distance;
		}else if(usData[0]*100>=DETECT_WALL&&count>3) {
			return DETECT_WALL;
		}else {
			count = 0;
			distance = (int)usData[0]*100;
			return distance;
		}
	}
	
}

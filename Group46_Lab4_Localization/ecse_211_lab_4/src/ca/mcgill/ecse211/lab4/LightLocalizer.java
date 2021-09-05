package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class LightLocalizer {
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;
	
	private static final double LS_LENGTH = 4;			//distance from the center of the robot to the light sensor
	
	private static final double BLACK_LINE = 250.0;		//black line light intensity
	
	private static final double TRACK = 17;				//track of the robot
	private static final double WHEEL_RAD = 2.25;		//wheel radius
	
	double last_color;									//last color the light  sensor detected
	
	/**
	 * 
	 * construction of the class
	 * 
	 * @param odo is the odometer
	 * @param colorSensor is the light sensor
	 * @param colorData is the light sensor data
	 */
	public LightLocalizer (Odometer odo, SampleProvider colorSensor, float[] colorData) {
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
	}
	
	/**
	 * 
	 * this method makes the robot to localize using color sensor
	 * 
	 * @throws InterruptedException
	 */
	public void Localize() throws InterruptedException {
		int countline = 0;					//count how many line are detected
		double angles[] = new double[4];	//angle for each line
		
		turnTo(45);							//turn to 45 degree direction
		Lab4.lcd.drawString("Color: " + getLightData(),0,4);
		while(getLightData() >BLACK_LINE) {					//keep going until there is a black line
			Lab4.lcd.drawString("Color: " + getLightData(),0,4);
			odo.getMotor()[0].forward();
			odo.getMotor()[1].forward();
		}
		
		odo.getMotor()[0].rotate(180,true);				//move forward for some extra distance
		odo.getMotor()[1].rotate(180);
		odo.setMotorSpeeds(0);							//stop
		
		last_color = getLightData();					//get color data
		
		odo.setMotorSpeeds(70);
		while (countline < 4) {							//needs to detect four lines
			odo.spin(true);								//robot spin clockwise
			if(getLightData() <= BLACK_LINE) {			//if black line detected
				double detect_theta = odo.getXYT()[2];	//get the current angle
				angles[countline] = detect_theta;		//record the angle
				countline++;							//update angle count
				last_color = BLACK_LINE;				//update last color
				Sound.beep();							//beep
				Thread.sleep(1000);						//sleep for 1 sec, to make sure that the same line will not be detected twice
			}else {
				last_color = getLightData();			//otherwise, update color
			}
		}
		
		odo.setMotorSpeeds(0);				
		
		double angle_x, angle_y;
		double position_x, position_y;
		
		angle_y = Math.abs(angles[0]-angles[2]);			//calculate angle for y correction
		angle_x = Math.abs(angles[1]-angles[3]);			//calculate angle for x correction
		
		position_x = (-1)*LS_LENGTH*Math.cos(Math.toRadians(angle_x)/2);    //calculate correct x position
		position_y = (-1)*LS_LENGTH*Math.cos(Math.toRadians(angle_y)/2);	//calculate correct y position
		
		odo.setX(position_x);							//update x
		odo.setY(position_y);							//update y
	}
	
	/**
	 * 
	 * this method makes robot to turn to a specific direction
	 * 
	 * @param theta is the target direction
	 */
	
	private void turnTo(double theta) {
		double turn_angle = theta-odo.getXYT()[2];		
		if (turn_angle > 180) {
			turn_angle = turn_angle-360;
		}else if(turn_angle<(-180)) {
			turn_angle = turn_angle+360;
		}
		
	    if(turn_angle==360) {
	    	turn_angle = 0;
	    }
	    
	    odo.setMotorSpeeds(70);
	    
	    odo.getMotor()[0].rotate(convertAngle(WHEEL_RAD, TRACK, turn_angle),true);			//turn to that direction
	    odo.getMotor()[1].rotate(-convertAngle(WHEEL_RAD, TRACK, turn_angle),false);
	    
	    odo.getMotor()[0].stop(true);							//stop the motor
	    odo.getMotor()[1].stop();
	    
	}
	
	/**
	 * 
	 * this method takes the radius of wheels and the distance that needs to be traveled, and calculate 
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
	 * 
	 * this method take the radius of wheels, width of the robot and the angle needs to be rotated by the robot, 
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
	 * 
	 * this method is used to get data from light sensor
	 * 
	 * @return color data
	 */
	public double getLightData() {
		colorSensor.fetchSample(colorData,0);
		double color = colorData[0] * 1000;
		
		return color;
	}
	
}


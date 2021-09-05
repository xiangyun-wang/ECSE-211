package ca.mcgill.ecse211.lab4;



import ca.mcgill.ecse211.lab4.Odometer;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab4 {
	private static final EV3LargeRegulatedMotor leftmotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));		//left motor
	private static final EV3LargeRegulatedMotor rightmotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));		//right motor
	private static final Port usPort = LocalEV3.get().getPort("S1");				//port for ultrasonic sensor
	private static final Port colorPort = LocalEV3.get().getPort("S2");				//port for color sensor
	private static final double TRACK = 17;							//track of the robot
	private static final double WHEEL_RAD = 2.25;					//wheel radius of the robot
	static Odometer odo ;											//odometer
	
	public static TextLCD lcd = LocalEV3.get().getTextLCD();		//lcd display
	
	public static void main(String[] args) throws OdometerExceptions, InterruptedException {
		
		int buttonChoice;
		
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);			//get the ultrasonic sensor
		SampleProvider usValue = usSensor.getMode("Distance");			
		float[] usData = new float[usValue.sampleSize()];	
		
		SensorModes colorSensor = new EV3ColorSensor(colorPort);		//get the color sensor
		SampleProvider colorValue = colorSensor.getMode("Red");
		float[] colorData = new float[colorValue.sampleSize()];		
		
		odo = Odometer.getOdometer(leftmotor, rightmotor, TRACK, WHEEL_RAD); 		//get odometer
		
		Display odometryDisplay = new Display(lcd); 					//get lcd display
		
		
	    
	    
	    
	    do {
	        // clear the display
	        lcd.clear();
	        
	        // ask the user whether the motors should locate in falling or rising
	        lcd.drawString("< Left | Right >", 0, 0);
	        lcd.drawString("       |        ", 0, 1);
	        lcd.drawString("Falling| Rising", 0, 2);
	        lcd.drawString("	   | 		 ", 0, 3);
	        lcd.drawString("       | 		 ", 0, 4);

	        buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
	      } while (buttonChoice != Button.ID_RIGHT&&buttonChoice != Button.ID_LEFT);
	    
	    Thread odoThread = new Thread(odo);			//start the odometer
	    odoThread.start();
	    Thread odoDisplayThread = new Thread(odometryDisplay);		//display odometer data
	    odoDisplayThread.start();
	    
	    if(buttonChoice== Button.ID_RIGHT) {			//if right button is pressed, use rising edge to locate
	    	USLocalizer usr = new USLocalizer(odo, usValue, usData, USLocalizer.LocalizationOption.RISING_EDGE);
	    	usr.Localize();
	    	lcd.drawString("Finished US Localization", 0, 3);
	    }else {											//otherwise, use falling edge to locate
	    	USLocalizer usl = new USLocalizer(odo, usValue, usData, USLocalizer.LocalizationOption.FALLING_EDGE);
	    	usl.Localize();
	    	lcd.drawString("Finished US Localization", 0, 3);
	    }
	    turnTo(0);										//turn to 0 degree
	    Button.waitForAnyPress();						//wait for further instruction
	    lcd.drawString("CS Localization Started", 0, 4);		//indicate that light sensor localization is initiated
	    LightLocalizer lsl = new LightLocalizer (odo, colorValue, colorData);		//use light(color) sensor to localize
	    lsl.Localize();
	    travelTo(0,0);										//travel to 0,0
	    turnTo(0);											//turn to 0
	    lcd.drawString("Finished CS Localization", 0, 4);
	    
	    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
	    
	    System.exit(0);
	    
	}
	
	/**
	 * 
	 * this method takes a coordinate as an input and make the robot to travel to that point
	 * 
	 * @param x is the x position
	 * @param y is the y position
	 */
static void travelTo(double x, double y) {
		
		double pathX = x-odo.getXYT()[0];		//calculate x displacement 
		double pathY = y-odo.getXYT()[1];		//calculate y displacement
		double path = Math.hypot(pathX, pathY);		//calculate path length
		double pathAngle;
		if(pathX<0&&pathY>=0) {						//calculate turning angle
			pathAngle = 360-(Math.toDegrees(Math.atan2(pathY,pathX))-90);
		}else {
			pathAngle = 90-Math.toDegrees(Math.atan2(pathY,pathX));
		}
		
		turnTo(pathAngle);							//turn to the direction angle
		
		
		
		leftmotor.setSpeed(70);			//set speed
		rightmotor.setSpeed(70);
		
		leftmotor.rotate(convertDistance(WHEEL_RAD,path),true);		//go to the directed point
		rightmotor.rotate(convertDistance(WHEEL_RAD,path),false);
		
		
		
		leftmotor.stop(true);							//stop the motor
		rightmotor.stop();
	}
	

/**
 * 
 * this method makes robot to turn to a specific direction
 * 
 * @param theta is the target direction
 */
	static void turnTo(double theta) {
		double turn_angle = theta-odo.getXYT()[2];			//calculate angle needs to turn
		if (turn_angle > 180) {
			turn_angle = turn_angle-360;
		}else if(turn_angle<(-180)) {
			turn_angle = turn_angle+360;
		}
		leftmotor.setSpeed(70);					//set motor speed
	    rightmotor.setSpeed(70);
	    if(turn_angle==360) {
	    	turn_angle = 0;
	    }
	    leftmotor.rotate(convertAngle(WHEEL_RAD, TRACK, turn_angle),true);			//turn to that direction
	    rightmotor.rotate(-convertAngle(WHEEL_RAD, TRACK, turn_angle),false);
	    
	    leftmotor.stop(true);							//stop the motor
	    rightmotor.stop();
	    
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
	
}

package ca.mcgill.ecse211.final_project.finalproject;

import ca.mcgill.ecse211.final_project.Odometer.Odometer;
import ca.mcgill.ecse211.final_project.Sensor.ColorSensor;
import ca.mcgill.ecse211.final_project.Sensor.UltrasonicPoller;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * 
 * This class contains variables used in the final project
 * 
 * @author Xiangyun Wang
 * @author Younggue Kim
 *
 */
public class Resources {
	/**
	 * The left motor.
	 */
	public static final EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	
	/**
	 * The right motor. 
	 */
	public static final EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	
	/**
	 * The claw motor. 
	 */
	public static final EV3LargeRegulatedMotor CLAW_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	/**
	 * The sensor motor. 
	 */
	public static final EV3MediumRegulatedMotor SENSOR_MOTOR = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D"));
	/**
	 * Allocate a port for the ultrasonic sensor 
	 */
	public static final Port US_PORT = LocalEV3.get().getPort("S3");
	/**
	 * Allocate a port for the light sensor 
	 */
	public static final Port LS_PORT_1 = LocalEV3.get().getPort("S1");
	/**
	 * Allocate a port for the light sensor 
	 */
	public static final Port LS_PORT_4 = LocalEV3.get().getPort("S4");
	/**
	 * Allocate a port for the color sensor
	 */
	public static final Port CS_PORT = LocalEV3.get().getPort("S2");
	/**
	 * Create an instance for the ultrasonic sensor 
	 */
	public static final EV3UltrasonicSensor US_SENSOR = new EV3UltrasonicSensor(US_PORT);
	/**
	 * Create an instance for the left light sensor
	 */
	public static final EV3ColorSensor LIGHTSENSOR_L = new EV3ColorSensor(LS_PORT_1);
	/**
	 * Create an instance for the right light sensor 
	 */
	public static final EV3ColorSensor LIGHTSENSOR_R = new EV3ColorSensor(LS_PORT_4);
	/**
	 * Create an instance for the color  sensor
	 */
	public static final EV3ColorSensor COLOR_SENSOR = new EV3ColorSensor(CS_PORT);
	/**
	 * object instanced once by this class 
	 */
	public static final TextLCD LCD = LocalEV3.get().getTextLCD();
	/**
	 * Get an instance of ultrasonic sensor in measurement mode 
	 */
	public static final SampleProvider US_VALUE = US_SENSOR.getMode("Distance");
	/**
	 * Allocate buffers for data return 
	 */
	public static float[] US_DATA = new float[US_VALUE.sampleSize()];
	/**
	 * Get an instance of the left light sensor 
	 */
	public static final SampleProvider LIGHT_VALUE_L = LIGHTSENSOR_L.getMode("Red");
	/**
	 * Allocate buffer for data return
	 */
	public static float[] lightDataL = new float[LIGHT_VALUE_L.sampleSize()];
	/**
	 * Get an instance of the right light sensor
	 */
	public static final SampleProvider LIGHT_VALUE_R = LIGHTSENSOR_R.getMode("Red");
	/**
	 * Allocate buffer for data return
	 */
	public static float[] lightDataR = new float[LIGHT_VALUE_R.sampleSize()];
	/**
	 * Get an instance of the color sensor 
	 */
	public static final SampleProvider COLOR_VALUE = COLOR_SENSOR.getMode("RGB");
	/**
	 * Allocate buffer for data return 
	 */
	public float[] colorData = new float[COLOR_VALUE.sampleSize()];
	/**
	 * Ultrasonic thread 
	 */
	public static final UltrasonicPoller myusData = new UltrasonicPoller(US_VALUE);
	/**
	 * Color sensor thread for can detection
	 */
	public static final ColorSensor mycsData = new ColorSensor();
	/**
	 * Tile Length 
	 */
	public static final double TILE_LENGTH = 30.48;
	/**
	 * The distance between the wheels  
	 */
	public static final double TRACK = 13.5;  //13.55 is too large
	/**
	 * The wheel radius
	 */
	public static final double WHEEL_RAD = 2.1;
	/**
	 * odometer
	 */
	public static Odometer odo;
	/**
	 *the distance of between the robot and the light sensor 
	 */
	public static final double LS_LENGTH = 9.5;
	/**
	 * Checking whether the target is detected 
	 */
	public static boolean target_detected = false;
	/**
	 * the current color detected 
	 */
	public static int detected_color = 0;
	/**
	 * shows if the can is heavy or not
	 */
	public static boolean heavy = false;
	/**
	 * shows if there is a can detected
	 */
	public static boolean can_detected = false;
	/**
	 * scanning point when reaches the searching area
	 */
	//public static int[] scanning_point = new int[2];
	/**
	 * Lower left x of the starting corner
	 */
	public static int Start_LLx;
	/**
	 * Lower left y of the starting corner
	 */
	public static int Start_LLy;
	/**
	 * Upper Right x of the starting corner
	 */
	public static int Start_URx;
	/**
	 * upper right y of the starting corner
	 */
	public static int Start_URy;
	/**
	 * lower left x of tunnel
	 */
	public static int TN_LLx;
	/**
	 * lower left y of tunnel
	 */
	public static int TN_LLy;
	/**
	 * upper right x of tunnel
	 */
	public static int TN_URx;
	/**
	 * upper right y of tunnel
	 */
	public static int TN_URy;
	/**
	 * lower left x of searching zone
	 */
	public static int SZ_LLx;
	/**
	 * lower left y of searching zone
	 */
	public static int SZ_LLy;
	/**
	 * upper right x of searching zone
	 */
	public static int SZ_URx;
	/**
	 * upper right y of searching zone
	 */
	public static int SZ_URy;
	/**
	 * starting corner of the robot
	 */
	public static int Corner;
	/**
	 * coordinate for lower left x of the island 
	 */
	public static int Island_LLx = 0; //= WiFi.data_value[12];
	/**
	 * coordinate for lower left y of the island 
	 */
	public static int Island_LLy = 0; //= WiFi.data_value[13];
	/**
	 * coordinate for upper right x of the island 
	 */
	public static int Island_URx = 0; //= WiFi.data_value[14];
	/**
	 * coordinate for upper right y of the island 
	 */
	public static int Island_URy = 0; //= WiFi.data_value[15];
	/**
	 * tunnel direction, true if along x, false if along y
	 */
	public static boolean TN_Dir;
	/**
	 * travel direction of the tunnel, true if up and right, false if down and  left
	 */
	public static boolean TN_TR;
	/**
	 * scan direction at the scanning point, true if clockwise, false if counter clockwise
	 */
	public static boolean SCAN_Dir;
	/**
	 * direction the robot faces before starting the scan
	 */
	public static int FACE_Dir;
	/**
	 * scanning point of the searching zone (where to spin and scan)
	 */
	public static int[] SCAN_Pt1;
	/**
	 * Whether or not a backup scan is initiated. This can be true only when the searching area is retangular. 
	 */
	public static boolean backup_scan = false;
	/**
	 * The backup scanning point, will only be defined if the searching area is rectangular
	 */
	public static int[] SCAN_Pt2;
	/**
	 * Indicate if the searching area is rectangular, 1 if it's rectangular along x, -1 if it's along y, 0 if it is a square, 
	 */
	public static int rect = 0;
	/**
	 * How many lines needs to be passed along x in order to get to the tunnel
	 */
	public static int count_x = 0;
	/**
	 * How many lines needs to be passed along y in order to get to the tunnel
	 */
	public static int count_y = 0;
	public static boolean first_scan = true;
	
	/**
	 * This method update the map information for red or green team. Team color will be assigned by the server.  
	 * If the red team number is the same as ours, we just get the parameter for the red team, otherwise, we will get the parameter
	 * for the blue team.
	 * @param input data array, should get from the wifi class 
	 */
	public static void update (int[] input) {
		Island_LLx = input[12];Island_LLy = input[13];Island_URx = input[14];Island_URy = input[15];
		if(input[0] == 20) {
			Corner = input[2];Start_LLx = input[4];Start_LLy = input[5]; Start_URx = input[8]; Start_URy = input[9];TN_LLx = input[16];TN_LLy = input[17];
			TN_URx = input[18];TN_URy = input[19]; SZ_LLx = input[24];SZ_LLy = input[25];SZ_URx = input[26];SZ_URy = input[27];
		}else {
			Corner = input[3];Start_LLx = input[6];Start_LLy = input[7]; Start_URx = input[10]; Start_URy = input[11];TN_LLx = input[20];TN_LLy = input[21];
			TN_URx = input[22];TN_URy = input[23]; SZ_LLx = input[28];SZ_LLy = input[29];SZ_URx = input[30];SZ_URy = input[31];
		}
	}
}

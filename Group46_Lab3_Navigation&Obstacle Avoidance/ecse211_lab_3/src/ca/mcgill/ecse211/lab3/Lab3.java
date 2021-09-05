// Lab2.java
package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {

  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  private static final Port usPort = LocalEV3.get().getPort("S3");
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  
  public static final double WHEEL_RAD = 2.3; //2.2 or 2.3 for ours
  public static final double TRACK = 17.0;// 17.0
  
  	static int[] first_point= {0,1};					//coordinates for the path
	static int[] second_point= {1,2};
	static int[] third_point= {1,0};
	static int[] forth_point= {2,1};
	static int[] last_point= {2,2};

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;
    
    SensorModes usSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
	SampleProvider usDistance = usSensor.getMode("Distance");	// usDistance provides samples from this instance
	float[] usData = new float[usDistance.sampleSize()];		// usData is the buffer in which data are returned
    
	// Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
    //OdometryCorrection odometryCorrection = new OdometryCorrection(); // TODO Complete
                                                                      // implementation
    Display odometryDisplay = new Display(lcd); // No need to change
    
   
    


    do {
      // clear the display
      lcd.clear();

      // ask the user whether the motors should drive in a square or float
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("       |        ", 0, 1);
      lcd.drawString(" Simple| Navigate", 0, 2);
      lcd.drawString("Navigation| 		   ", 0, 3);
      lcd.drawString("       | 		   ", 0, 4);

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_RIGHT&&buttonChoice != Button.ID_LEFT);

    if (buttonChoice == Button.ID_RIGHT) {

      // Display changes in position
      // the untrasonic sensor is used to detect block
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      Navigation nav = new Navigation(leftMotor,rightMotor,/*sensorMotor,*/usDistance,usData);
      nav.start();
    } else  {
    	// Display changes in position
    	//simpleNavigation is created to run the robot
    	Thread odoThread = new Thread(odometer);
        odoThread.start();
        Thread odoDisplayThread = new Thread(odometryDisplay);
        odoDisplayThread.start();
        SimpleNavigation simnav = new SimpleNavigation(leftMotor,rightMotor);
        simnav.start();
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}

package ca.mcgill.ecse211.final_project.IdentifyCan;

import static ca.mcgill.ecse211.final_project.finalproject.Resources.*;

import ca.mcgill.ecse211.final_project.Sensor.ColorDetection;
import lejos.hardware.Sound;

/**
 * 
 * This class is used to identify cans
 * 
 * @author Xiangyun Wang
 *
 */
public class IdentifyCan {
	
	/** 
	 * This method is used to detect the color of the can and update it to the detected_color
	 * Each time a sample scan is taken a color is assigned to it. 
	 * The color array is used to keep track of the number of occurrences of a color 
	 * The color that occurred the most times, thus the color having the highest value in the array,
	 * is the color printed to the LCD string
	 * 
	 * @param myData This is the light sensor data provider
	 */
	public static void detect_color() {
		SENSOR_MOTOR.setSpeed(70);
		SENSOR_MOTOR.rotate(-200,true);
		int[] color = {0,0,0,0,0};
		while(SENSOR_MOTOR.isMoving()) {
			int result = ColorDetection.printColor(mycsData.RGBVal);
			color[result]++;
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		
		int index_number = 4;
		int largest = color[4];	
		for (int i = 1; i<color.length-1;i++) {
			if(largest<color[i]) {
				index_number = i;
				largest = color[i];
			}
		}
		SENSOR_MOTOR.rotate(200);
		detected_color = index_number;			//set to detected_color
	}

	/**
	 * This method allows the robot the can to be weighted:
	 * Before getting the tacho count and the time, we set the claw motors to float.
	 * To figure out if the can is light or heavy, we record the amount of time t1 when the tacho count is 0,
	 * then records t2, the time it takes for the motor to reach a tacho count of 50 degrees.
	 * 
	 * Then it calculates the difference and if it is smaller than 300ms, it is a heavy can
	 * if it is bigger than 300ms, it is light can.
	 * 
	 * @return true for heavy, false for light
	 */
	public static void weight(){
		CLAW_MOTOR.setSpeed(50);
		CLAW_MOTOR.rotate(-230); 					//arbitrary angle: please do some testing to get the can high enough (for Mert)
		//CLAW_MOTOR.setSpeed(80);
		//CLAW_MOTOR.stop();						//block the motor so the can doesn't fall off
		CLAW_MOTOR.resetTachoCount(); 			//set tacho count to 0
		long t1 = System.currentTimeMillis(); 	//time before can goes down
		CLAW_MOTOR.flt(); 						//let motors go free
		//----------might need to be changed
		while(CLAW_MOTOR.getTachoCount()<100);	//again, here 50 is an arbitrary tacho count. we need to know the exact tacho count that it takes for the rotation we want (testing for Mert)
												//we are waiting for the motor to reach the target tacho count, then get the time.
		long t2 = System.currentTimeMillis();	//time once can is down
		CLAW_MOTOR.stop(); 						//we block the motor again
		int deltaT = (int) (t2-t1); 			//interval of time it took for the can to go down
		LCD.drawInt((deltaT), 3, 0); 			//print time
		
		if(deltaT<550) {						//if it took less than XXXms for the can to go down, it is heavy
			LCD.drawString("Can is heavy", 3, 1);
			heavy=true;
		}
		else {									//otherwise it is light
			LCD.drawString("Can is light", 3, 1);
			heavy=false;
		}
		CLAW_MOTOR.rotate((-1)*CLAW_MOTOR.getTachoCount());
		CLAW_MOTOR.setSpeed(80);
	}
	
	/**
	 * This method identify the type of the can, and make certain beeps according to the project instruction
	 * If the can is heavy, make long beeps, if the can is light, make short beeps. 
	 * If red, make 4 beeps; if yellow, make 3 beeps; if green, make 2 beeps; if blue, make 1 beep. 
	 * @param heavy whether or not the can is heavy (true if heavy, false if light)
	 * @param color color of the can
	 * @throws InterruptedException 
	 */
	public static void identify_can_beep(boolean heavy, int color) throws InterruptedException{
		if(heavy) {
			for(int i = 0; i < color; i++) {
				Sound.playTone(1000, 1000);
				Thread.sleep(200);
			}
		}else {
			for(int i = 0; i < color; i++) {
				Sound.playTone(1000, 500);
				Thread.sleep(200);
			}
		}
	}
}

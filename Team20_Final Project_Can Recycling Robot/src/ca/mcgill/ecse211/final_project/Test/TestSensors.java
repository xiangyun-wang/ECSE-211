package ca.mcgill.ecse211.final_project.Test;

import static ca.mcgill.ecse211.final_project.finalproject.Resources.*;

import ca.mcgill.ecse211.final_project.Sensor.UltrasonicPoller;
import ca.mcgill.ecse211.final_project.finalproject.FinalProject;

/**
 * This class is used to sumbit the sensors of the three teams
 * to different conditions. This way, we can figure out which of
 * the sensors would be more efficient for a certain task.
 * @author Katrina Poulin
 * @author Xiangyun Wang
 */
public class TestSensors {
	final static UltrasonicPoller myusData = new UltrasonicPoller(US_VALUE);
	public void main() {
		myusData.start();
		DistanceTest();
		BlackLinesTest();
	}
	/**
	 * This method will be used to determine the accuracy of the ultrasonic sensors.
	 */
	public static int DistanceTest() {
		return myusData.getDistance();
	}
	/** 
	 * This method will call a method that will allow the light sensor
	 * to detect black lines. We will then test this method with each of
	 * the light sensors so that we can determine which one is the most
	 * accurate for this task.
	 */
	public static void BlackLinesTest() {
		int[] floor_color = FinalProject.getLightData();
		int[] current_color = FinalProject.getLightData();
		odo.setMotorSpeeds(50);
		while(Math.abs(floor_color[0]-current_color[0])<100&&Math.abs(floor_color[1]-current_color[1])<100) {
			odo.moveforward();
			//Thread.sleep(50);
			current_color = FinalProject.getLightData();
		}
		odo.stop();
	}
}

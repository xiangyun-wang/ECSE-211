package ca.mcgill.ecse211.final_project.Test;
import static ca.mcgill.ecse211.final_project.finalproject.Resources.*;

import ca.mcgill.ecse211.final_project.Sensor.UltrasonicPoller;
import ca.mcgill.ecse211.final_project.finalproject.FinalProject;

/**
 * This class is used to test if the robot can detect cans successfully with the ultrasonic sensor. 
 * @author Katrina Poulin
 * @author Xiangyun Wang
 *
 */
public class TestScanning {
	final static UltrasonicPoller myusData = new UltrasonicPoller(US_VALUE);
	/**
	 * In the main method, we call the testScan method, which polls for a can,
	 * then returns the distance of a detected can. We can determine the accuracy
	 * of the sensor by knowing the actual distance between the sensor and the can.
	 * @param args
	 * @throws InterruptedException 
	 */
	public static void main(String[] args) throws InterruptedException {
		myusData.start();
		test_Scan();
	}
	/**
	 * This method searches for a can, then returns the distance from the sensor to this can.
	 * @throws InterruptedException 
	 */
	public static void test_Scan() throws InterruptedException {
		FinalProject.search_can(0,true, 100);
	}
}

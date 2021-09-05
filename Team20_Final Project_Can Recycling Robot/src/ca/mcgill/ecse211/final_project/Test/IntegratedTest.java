package ca.mcgill.ecse211.final_project.Test;

import ca.mcgill.ecse211.final_project.Localizer.LightLocalizer_new;
import ca.mcgill.ecse211.final_project.Localizer.UltrasonicLocalizer;
import ca.mcgill.ecse211.final_project.MapData.GridMap;
import ca.mcgill.ecse211.final_project.MapData.WiFi;
import ca.mcgill.ecse211.final_project.Navigation.Navigation;
import ca.mcgill.ecse211.final_project.Odometer.Odometer;
import ca.mcgill.ecse211.final_project.Odometer.OdometerExceptions;
import ca.mcgill.ecse211.final_project.finalproject.FinalProject;
import lejos.hardware.Button;

import static ca.mcgill.ecse211.final_project.finalproject.Resources.*;

import ca.mcgill.ecse211.final_project.IdentifyCan.IdentifyCan;

/**
 * 
 * This class is used to do integrated tests
 * 
 * @author Xiangyun Wang
 *
 */

public class IntegratedTest {
	
	public static void main(String[] args) throws InterruptedException, OdometerExceptions {
		odo = Odometer.getOdometer();
		Thread odoThread = new Thread(odo);
		odo.setTheta(0);
		odoThread.start();
		LEFT_MOTOR.setAcceleration(1000);
		RIGHT_MOTOR.setAcceleration(1000);
		CLAW_MOTOR.setSpeed(80);
		testWiFi();
		System.out.println("ready");
		Button.waitForAnyPress();
		testLocalization();
		Button.waitForAnyPress();
		testTravelTN(true);
		testCompleteSearch();
		if(backup_scan) {
			Navigation.travelTo(SCAN_Pt2[0]*TILE_LENGTH, SCAN_Pt2[1]*TILE_LENGTH, false, 300);
			backup_scan = false;
		}
		testColor();
		testWeigh();
		testIdentifyBeep();
		testTravel_SZ();
		testTravelTN_back();
		testRelease();
		 
	}
	/**
	 * This method test ultrasonic localization and light localization
	 * @throws InterruptedException
	 */
	private static void testLocalization() throws InterruptedException {
		myusData.start();
		UltrasonicLocalizer.fallingEdge();
		LightLocalizer_new.localize();
	}
	
	/**
	 * This method test travel_TN method. The robot should travel to the entrance of the tunnel and travel through the tunnel
	 * @param start
	 * @throws InterruptedException
	 */
	private static void testTravelTN(boolean start) throws InterruptedException {
		FinalProject.travel_TN(TN_LLx, TN_LLy, TN_URx, TN_URy, TN_Dir, TN_TR,start);
	}
	
	/**
	 * This method is used to test if the wifi class is working or not. The robot should get the parameter from the server
	 * and initialize the grid map
	 */
	private static void testWiFi() {
		update(WiFi.getMapInfo());
		GridMap.update_map();
	}
	
	/**
	 * This method is used to test whether or not the robot can travel to the scanning point of the search zone. 
	 * @throws InterruptedException
	 */
	private static void testTravel_SZ() throws InterruptedException {
		FinalProject.travel_SZ(SCAN_Pt1[0], SCAN_Pt1[1]);
	}
	
	/**
	 * This method test the searching algorithm of the robot. Details are explained in the original method in FinalProject
	 * @throws InterruptedException
	 */
	private static void testCompleteSearch() throws InterruptedException {
		FinalProject.complete_search();
	}
	
	/**
	 * This method is used to test if the robot can identify the weight of a can
	 */
	private static void testWeigh() {
		FinalProject.open_claw();
		IdentifyCan.weight();
	}
	/**
	 * This method is used to test if the robot can identify the color of a can
	 */
	private static void testColor() {
		mycsData.start();
		IdentifyCan.detect_color();
	}
	/**
	 * This method is used to test if the robot can travel back throught the tunnel
	 * @throws InterruptedException
	 */
	private static void testTravelTN_back() throws InterruptedException {
		FinalProject.travel_TN(TN_LLx, TN_LLy, TN_URx, TN_URy, TN_Dir, !TN_TR,false);
	}
	/**
	 * This method is used to test if the robot can travel to the starting point and release the can. 
	 * @throws InterruptedException
	 */
	private static void testRelease() throws InterruptedException {
		FinalProject.release_can();
	}
	/**
	 * This method is sued to test if the robot can travel to the entrance of the tunnel. 
	 * @throws InterruptedException
	 */
	@SuppressWarnings("unused")
	private static void testTravel_EN() throws InterruptedException {
		FinalProject.travel_TN_EN(TN_LLx, TN_LLy, TN_URx, TN_URy, TN_Dir, TN_TR);
	}
	/**
	 * This method is used to test if the robot can beep correctly according to the weight and color of the can. 
	 * @throws InterruptedException 
	 */
	private static void testIdentifyBeep() throws InterruptedException {
		IdentifyCan.identify_can_beep(heavy, detected_color);
	}
}

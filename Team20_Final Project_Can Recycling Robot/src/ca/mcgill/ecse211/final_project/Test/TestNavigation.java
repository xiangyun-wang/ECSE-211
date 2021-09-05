package ca.mcgill.ecse211.final_project.Test;
import static ca.mcgill.ecse211.final_project.finalproject.Resources.*;

import ca.mcgill.ecse211.final_project.finalproject.FinalProject;

/**
 * This class makes the robot travel to the search zone step by step.
 * It goes to the tunnel, through the tunnel, and to the lower left
 * corner of the search zone.
 * @author Katrina Poulin
 * @author Xiangyun Wang
 *
 */
public class TestNavigation {
	/**
	 * The main method calls the methods to go to and through the tunnel,
	 * and the method to travel to the searching zone.
	 * @param args
	 * @throws InterruptedException
	 */
	
	public static void main (String[] args) throws InterruptedException {
		test_TN();
		test_SZ();
		
	}
	
	/**
	 * This method calls travel_TN (travel through tunnel) method to see if the method works or not. 
	 * @throws InterruptedException
	 */
	public static void test_TN() throws InterruptedException {
		FinalProject.travel_TN(TN_LLx, TN_LLy, TN_URx, TN_URy, true, true, true);
	}
	
	/**
	 * This method calls travel_SZ (travel to the searching zone) method to see if the mothod works or not. 
	 * @throws InterruptedException
	 */
	public static void test_SZ() throws InterruptedException {
		FinalProject.travel_SZ(SCAN_Pt1[0],SCAN_Pt1[1]);
	}
	
	
}

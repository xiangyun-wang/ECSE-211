package ca.mcgill.ecse211.final_project.Test;
import ca.mcgill.ecse211.final_project.finalproject.FinalProject;

/**
 * The TestLifting class will test the efficiency of the lifting mechanism.
 * It will do that by calling a method that grabs a can, lifts it and puts it back on the ground.
 * 
 * @author Katrina Poulin
 * @author Xiangyun Wang
 *
 */
public class TestLifting {
	/**
	 * the main method will call testGrab and testRelease, which are performed one after the
	 * other to see if the robot can grab a can and release it.
	 * @param args
	 * @throws InterruptedException 
	 */
	public static void main (String[] args) throws InterruptedException {	
		testRelease();
		testOpen();
	}
	/**
	 * This method lowers the can and releases its grip on it.
	 * @throws InterruptedException 
	 */
	public static void testRelease () throws InterruptedException {
		FinalProject.release_can();
	}
	
	public static void testOpen() {
		FinalProject.open_claw();
	}

}


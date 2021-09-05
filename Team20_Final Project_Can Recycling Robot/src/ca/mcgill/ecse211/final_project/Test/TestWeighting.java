package ca.mcgill.ecse211.final_project.Test;

import ca.mcgill.ecse211.final_project.IdentifyCan.IdentifyCan;

/**
 * This class is used to determine whether if the can can determine accurately if a can is light or heavy.
 * @author Katrina Poulin
 * @author Xiangyun Wang
 *
 */
public class TestWeighting {
	/**
	 * the main method calls testWeight, which performs the weight detection.
	 * @param args
	 */
	public static void main(String[] args) {
		test_Weight();
	}
	/**
	 * This method is used to perform the weight of the can and return a boolean (light or heavy).
	 * Knowing the actual weight of the can, we can determine if the mechanism is accurate or not. 
	 */
	public static void test_Weight() {
		IdentifyCan.weight();
	}
}

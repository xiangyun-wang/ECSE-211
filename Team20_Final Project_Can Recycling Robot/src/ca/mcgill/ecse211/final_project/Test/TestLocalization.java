package ca.mcgill.ecse211.final_project.Test;


import ca.mcgill.ecse211.final_project.Localizer.*;

/**
 * This class is used to test the Localization sub-system in the project.
 * It will determine the best way to localize with the ultrasonic sensor.
 * It will also test the accuracy of the light localization.
 * @author Katrina Poulin
 */
public class TestLocalization {
	/** 
	 * The main method will call the methods that contain the different
	 * localization routines from the localization classes. Then, we will
	 * be able to determine the accuracy of each routine separately from
	 * the rest of the project.
	 * @throws InterruptedException 
	 */
	public void main(String[] args) throws InterruptedException {
		//UltrasonicLocalizer.fallingEdge();
		performUSTest();
		performLightTest();
	}
	/**
	 * This method calls the Ultrasonic Localization routine from the
	 * Localizer class. It will allow us to run only the Ultrasonic
	 * Localization in order to test it properly.
	 * @throws InterruptedException
	 */
	private void performUSTest() throws InterruptedException {
		
		//we call the falling edge method
		UltrasonicLocalizer.fallingEdge();
	}
	/**
	 * This method calls the Light Localization routine from the
	 * Localizer class. It will allow us to run only the Light
	 * Localization in order to test it properly.
	 * @throws InterruptedException
	 */
	private void performLightTest() throws InterruptedException {
		//we perform the light localization 
		LightLocalizer_new.localize();
		
	}
	
}

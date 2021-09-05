package ca.mcgill.ecse211.final_project.Sensor;

import static ca.mcgill.ecse211.final_project.finalproject.Resources.*;

/**
 * This class get the RGB value of a color sample
 * @author Xiangyun Wang
 * @author Younggue Kim
 *
 */
public class ColorSensor extends Thread{
	
	/**
	 * Color data from color sensor
	 */
	private float[] colorData;
	
	/**
	 * RGB value of a sample
	 */
	public double [] RGBVal;
	
	
	/**
	 * constructor of the class
	 */
	public ColorSensor() {
	    this.colorData = new float[3];
	}
    
	/**
	 * Update the RGB data after each 100ms
	 */
	public void run() {
		while(true) {
			try {
				RGBVal = getRGB();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
    
    /**
     * This method calculate the normalized RGB value and return them as a double array
     * @return A double array of normalized RGB values
     * @throws InterruptedException
     */
	public double[] getRGB() throws InterruptedException {
		
		COLOR_VALUE.fetchSample(colorData, 0);
		double normalized_R =colorData[0]/Math.sqrt(colorData[0]*colorData[0]+colorData[1]*colorData[1]+colorData[2]*colorData[2]);
		double nomorlized_G =colorData[1]/Math.sqrt(colorData[0]*colorData[0]+colorData[1]*colorData[1]+colorData[2]*colorData[2]);
		double normalized_B =colorData[2]/Math.sqrt(colorData[0]*colorData[0]+colorData[1]*colorData[1]+colorData[2]*colorData[2]);
		
		return new double[] {normalized_R, nomorlized_G, normalized_B};
   }
}

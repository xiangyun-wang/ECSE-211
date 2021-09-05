/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private static final double TILE_LENGTH = 30.48;
  private static final EV3ColorSensor lSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
  private SampleProvider lscolor = lSensor.getMode("Red");
  private float[] lsData;
  private int countline_x;
  private int countline_y;
  private double[] position;
  
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.lsData = new float[lSensor.sampleSize()];
    odometer = Odometer.getOdometer();
    this.countline_x=0;
    this.countline_y=0;
    
  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    float intensity;////////////////////////
   
    while (true) {
      correctionStart = System.currentTimeMillis();
      lscolor.fetchSample(lsData, 0);
      intensity = lsData[0]*1000;
      // TODO Trigger correction (When do I have information to correct?)
      // TODO Calculate new (accurate) robot position

      // TODO Update odometer with new calculated (and more accurate) vales
     
     if(intensity < 250) {
    	 position = odometer.getXYT();
         double theta = position[2];
    	 Sound.beep();  //change a sound?
    	 if(theta < 15 || theta > 345) {
    		 
    		 odometer.setY(countline_y*TILE_LENGTH);
    		 countline_y++;
    	 }else if(theta < 105 && theta > 75) {
    		 
    		 odometer.setX(countline_x*TILE_LENGTH);
    		 countline_x++;
    	 }else if(theta < 195 && theta > 165) {
    		 odometer.setY((countline_y-1)*TILE_LENGTH);
    		 countline_y--;
    	 }else if(theta <285  && theta > 255) {
    		 odometer.setX((countline_x-1)*TILE_LENGTH);
    		 countline_x--;
    	 }else {
    		 
    	 }
    	 
     }

      //odometer.setXYT(0.3, 19.23, 5.0);

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}

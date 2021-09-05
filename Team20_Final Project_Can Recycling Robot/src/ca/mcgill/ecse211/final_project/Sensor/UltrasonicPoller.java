package ca.mcgill.ecse211.final_project.Sensor;

import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;

/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller thread. The while
 * loop at the bottom executes in a loop. Assuming that the us.fetchSample, and cont.processUSData
 * methods operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
 * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS
 * or about 14 Hz.
 * 
 * @author Xiangyun Wang
 * @author Younggue Kim
 * 
 */
public class UltrasonicPoller extends Thread {
	
  /**
   * Distance value from ultrasonic sensor
   */
  private int distance;
  
  /**
   * Ultrasonic Data
   */
  private float[] usData;
  
  /**
   * number of sample of the ultrasonic data
   */
  private final int NUM_SAMPLE = 5;
  
  /**
   * median filter for the ultrasonic data
   */
  MedianFilter mf;

  public UltrasonicPoller(SampleProvider us) {
   usData = new float[us.sampleSize()*NUM_SAMPLE];
    mf = new MedianFilter(us, NUM_SAMPLE);
  }

  /**
   * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
   * [0,255]
   * 
   */
  public void run() {
    while (true) {
      mf.fetchSample(usData, 0); // acquire data
      distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
      try {
        Thread.sleep(20);
      } catch (Exception e) {
      } 
    }
  }
  
  /**
   * get the distance value from the ultrasonic sensor
   * @return distance read from the ultrasonic sensor
   */
  public int getDistance() {
	  return distance;
  }

}

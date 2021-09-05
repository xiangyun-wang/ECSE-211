package ca.mcgill.ecse211.sensor;

import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;

/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller thread. The while
 * loop at the bottom executes in a loop. Assuming that the us.fetchSample, and cont.processUSData
 * methods operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
 * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS
 * or about 14 Hz.
 */
public class UltrasonicPoller extends Thread {
  private int distance;
  private float[] usData;
  private final int NUM_SAMPLE = 5;
  MedianFilter mf;

  public UltrasonicPoller(SampleProvider us) {
   usData = new float[us.sampleSize()*NUM_SAMPLE];
    mf = new MedianFilter(us, NUM_SAMPLE);
    //this.usData = usData;
  }

  /*
   * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
   * [0,255] (non-Javadoc)
   * 
   * @see java.lang.Thread#run()
   */
  public void run() {
    while (true) {
      mf.fetchSample(usData, 0); // acquire data
      distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      } // Poor man's timed sampling
    }
  }
  
  public int getDistance() {
	  return distance;
  }

}

package ca.mcgill.ecse211.final_project.Odometer;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;


/**
 * This class controls the data of the odometer. 
 * It has methods that can get, update and override the data of the odometer. 
 * 
 * @author Xiangyun Wang
 * @author Younggue Kim
 *
 */
public class OdometerData {

  /**
   * Current x position of the odometer
   */
  private volatile double x; // x-axis position
  
  /**
   * Current y position of the odometer
   */
  private volatile double y; // y-axis position
  
  /**
   * Current direction of the odometer
   */
  protected static double theta; // Head angle

  /**
   * Track the number of odometer creacted
   */
  private volatile static int numberOfIntances = 0; // Number of OdometerData
                                                    // objects instantiated
                                                    // so far
  /**
   * The maximum number of odometer allowed
   */
  private static final int MAX_INSTANCES = 1; // Maximum number of
                                              // OdometerData instances

  // Thread control tools
  private static Lock lock = new ReentrantLock(true); // Fair lock for
                                                      // concurrent writing
  
  /**
   * Whether the thread is reseting position parameter
   */
  private volatile boolean isReseting = false; // Indicates if a thread is
                                               // trying to reset any
                                               // position parameters
  
  /**
   * Whether or not the thread finished reseting parameters
   */
  private Condition doneReseting = lock.newCondition(); // Let other threads
                                                        // know that a reset
                                                        // operation is
                                                        // over.
  /**
   * Instance of OdometerData
   */
  private static OdometerData odoData = null;

  /**
   * Default constructor. The constructor is private. A factory is used instead such that only one
   * instance of this class is ever created.
   */
  protected OdometerData() {
    this.x = 0;
    this.y = 0;
    OdometerData.theta = 0;
  }

  /**
   * OdometerData factory. Returns an OdometerData instance and makes sure that only one instance is
   * ever created. If the user tries to instantiate multiple objects, the method throws a
   * MultipleOdometerDataException.
   * 
   * @return An OdometerData object
   * @throws OdometerExceptions
   */
  public synchronized static OdometerData getOdometerData() throws OdometerExceptions {
    if (odoData != null) { // Return existing object
      return odoData;
    } else if (numberOfIntances < MAX_INSTANCES) { // create object and
                                                   // return it
      odoData = new OdometerData();
      numberOfIntances += 1;
      return odoData;
    } else {
      throw new OdometerExceptions("Only one intance of the Odometer can be created.");
    }

  }

  /**
   * Return the Odomometer data.
   * <p>
   * Writes the current position and orientation of the robot onto the odoData array. odoData[0] =
   * x, odoData[1] = y; odoData[2] = theta;
   * 
   * @param position the array to store the odometer data
   * @return the odometer data.
   */
  public double[] getXYT() {
    double[] position = new double[3];
    lock.lock();
    try {
      while (isReseting) { // If a reset operation is being executed, wait
        // until it is over.
        doneReseting.await(); // Using await() is lighter on the CPU
        // than simple busy wait.
      }

      position[0] = x;
      position[1] = y;
      position[2] = theta;

    } catch (InterruptedException e) {
      // Print exception to screen
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return position;

  }

  /**
   * Adds dx, dy and dtheta to the current values of x, y and theta, respectively. 
   * Data of the odometer is updated
   * 
   * @param dx
   * @param dy
   * @param dtheta
   */
  public void update(double dx, double dy, double dtheta) {
    lock.lock();
    isReseting = true;
    try {
      x += dx;
      y += dy;
      theta = (theta + (360 + dtheta) % 360) % 360; // keeps the updates
                                                    // within 360
                                                    // degrees
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }

  }

  /**
   * Overrides the values of x, y and theta. Use for odometry correction.
   * 
   * @param x the value of x
   * @param y the value of y
   * @param theta the value of theta
   */
  public void setXYT(double x, double y, double theta) {
    lock.lock();
    isReseting = true;
    try {
      this.x = x;
      this.y = y;
      OdometerData.theta = theta;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overrides x. Use for odometry correction.
   * 
   * @param x the value of x
   */
  public void setX(double x) {
    lock.lock();
    isReseting = true;
    try {
      this.x = x;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overrides y. Use for odometry correction.
   * 
   * @param y the value of y
   */
  public void setY(double y) {
    lock.lock();
    isReseting = true;
    try {
      this.y = y;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overrides theta. Use for odometry correction.
   * 
   * @param theta the value of theta
   */
  public void setTheta(double theta) {
    lock.lock();
    isReseting = true;
    try {
      OdometerData.theta = theta;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }

}

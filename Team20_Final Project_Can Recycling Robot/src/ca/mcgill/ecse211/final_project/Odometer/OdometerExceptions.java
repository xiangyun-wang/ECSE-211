package ca.mcgill.ecse211.final_project.Odometer;

/**
 * This class is used to handle errors regarding the singleton pattern used for the odometer and
 * odometerData
 *
 * @author Xiangyun Wang
 * @author Younggue Kim
 */
@SuppressWarnings("serial")
public class OdometerExceptions extends Exception {

  public OdometerExceptions(String Error) {
    super(Error);
  }

}

package ca.mcgill.ecse211.final_project.MapData;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;

/**
 * 
 * This is the wifi class which will be used to get map information from the sever
 * 
 * @auther Xiangyun Wang
 * @author Younggue Kim
 */
public class WiFi {
/**
 * ip address
 */
  private static final String SERVER_IP = "192.168.2.4";
  /**
   * team number
   */
  private static final int TEAM_NUMBER = 20;
  /**
   * data array
   */
  public static int[] data_value = new int[32];
  // Enable/disable printing of debug info from the WiFi class
  private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
  //public static int[] data_value = new int[32];
  @SuppressWarnings("rawtypes")
  
  /**
   *  this method get the map information through wifi and return these data as an array
   * @return array contains map information
   */
  public static int[] getMapInfo() {
    System.out.println("Running..");

    // Initialize WifiConnection class
    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

    // Connect to server and get the data, catching any errors that might occur
    try {
      /*
       * getData() will connect to the server and wait until the user/TA presses the "Start" button
       * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
       * pressing the upper left hand corner button (back/escape) on the EV3. getData() will throw
       * exceptions if it can't connect to the server (e.g. wrong IP address, server not running on
       * laptop, not connected to WiFi router, etc.). It will also throw an exception if it connects
       * but receives corrupted data or a message from the server saying something went wrong. For
       * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot
       * will receive a message saying an invalid team number was specified and getData() will throw
       * an exception letting you know.
       */
      Map data = conn.getData();
      data_value[0] = ((Long)data.get("RedTeam")).intValue();	//Red_Team
      data_value[1] = ((Long) data.get("GreenTeam")).intValue();	//Green_Team
      data_value[2] = ((Long) data.get("RedCorner")).intValue();	//Red_Corner
      data_value[3] = ((Long) data.get("GreenCorner")).intValue();//Green_Corner
      data_value[4] = ((Long) data.get("Red_LL_x")).intValue();//Red_LLx
      data_value[5] = ((Long) data.get("Red_LL_y")).intValue();//Red_LLy
      data_value[6] = ((Long) data.get("Green_LL_x")).intValue();//Green_LLx
      data_value[7] = ((Long) data.get("Green_LL_y")).intValue();//Green_LLy
      data_value[8] = ((Long) data.get("Red_UR_x")).intValue();//Red_URx
      data_value[9] = ((Long) data.get("Red_UR_y")).intValue();//Red_URy
      data_value[10] = ((Long) data.get("Green_UR_x")).intValue();//Green_URx
      data_value[11] = ((Long) data.get("Green_UR_y")).intValue();//Green_URy
      data_value[12] = ((Long) data.get("Island_LL_x")).intValue();//Island_LLx
      data_value[13] = ((Long) data.get("Island_LL_y")).intValue();//Island_LLy
      data_value[14] = ((Long) data.get("Island_UR_x")).intValue();//Island_URx
      data_value[15] = ((Long) data.get("Island_UR_y")).intValue();//Island_URy
      data_value[16] = ((Long) data.get("TNR_LL_x")).intValue();//TNR_LLx
      data_value[17] = ((Long) data.get("TNR_LL_y")).intValue();//TNR_URx
      data_value[18] = ((Long) data.get("TNR_UR_x")).intValue();//TNR_URx
      data_value[19] = ((Long) data.get("TNR_UR_y")).intValue();//TNR_URy
      data_value[20] = ((Long) data.get("TNG_LL_x")).intValue();//TNG_LLx
      data_value[21] = ((Long) data.get("TNG_LL_y")).intValue();//TNG_LLy
      data_value[22] = ((Long) data.get("TNG_UR_x")).intValue();//TNG_URx
      data_value[23] = ((Long) data.get("TNG_UR_y")).intValue();//TNG_URy
      data_value[24] = ((Long) data.get("SZR_LL_x")).intValue();//SZR_LLx
      data_value[25] = ((Long) data.get("SZR_LL_y")).intValue();//SZR_LLy
      data_value[26] = ((Long) data.get("SZR_UR_x")).intValue();//SZR_URx
      data_value[27] = ((Long) data.get("SZR_UR_y")).intValue();//SZR_URy
      data_value[28] = ((Long) data.get("SZG_LL_x")).intValue();//SZG_LLx
      data_value[29] = ((Long) data.get("SZG_LL_y")).intValue();//SZG_LLy
      data_value[30] = ((Long) data.get("SZG_UR_x")).intValue();//SZG_URx
      data_value[31] = ((Long) data.get("SZG_UR_y")).intValue();//SZG_URy
      
    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }
    System.out.println("Finished");
    return data_value;
  }
}

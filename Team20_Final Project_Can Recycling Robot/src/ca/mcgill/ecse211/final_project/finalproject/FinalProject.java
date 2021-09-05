package ca.mcgill.ecse211.final_project.finalproject;

//import ca.mcgill.ecse211.final_project.Sensor.ColorDetection;
//import ca.mcgill.ecse211.final_project.Sensor.ColorSensor;
//import ca.mcgill.ecse211.final_project.Sensor.UltrasonicPoller;

import lejos.hardware.Sound;

import static ca.mcgill.ecse211.final_project.finalproject.Resources.*;

import ca.mcgill.ecse211.final_project.IdentifyCan.IdentifyCan;
import ca.mcgill.ecse211.final_project.Localizer.LightLocalizer_new;
import ca.mcgill.ecse211.final_project.Localizer.UltrasonicLocalizer;
import ca.mcgill.ecse211.final_project.MapData.GridMap;
import ca.mcgill.ecse211.final_project.MapData.WiFi;
import ca.mcgill.ecse211.final_project.Navigation.Navigation;
//import ca.mcgill.ecse211.final_project.Odometer.Display;
import ca.mcgill.ecse211.final_project.Odometer.Odometer;
import ca.mcgill.ecse211.final_project.Odometer.OdometerExceptions;

/**
 * This class controls the robot to localize at the stating corner, travel through the tunnel, 
 * search for cans, determine the weight and color of the can, and bring it back to the stating corner. 
 * 
 * @author Xiangyun Wang
 * @author Younggue Kim
 *
 */
public class FinalProject {

	/**
	 * The main method calls starts the odometer and starts the navigation
	 * The entire process of the final project is in this method
	 * 
	 * @param args initial input
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */

	public static void main(String[] args) throws OdometerExceptions, InterruptedException {
		odo = Odometer.getOdometer();
		Thread odoThread = new Thread(odo);
		odoThread.start();
		myusData.start();
		update(WiFi.getMapInfo());
		GridMap.update_map();
		LEFT_MOTOR.setAcceleration(1000);
		RIGHT_MOTOR.setAcceleration(1000);
		UltrasonicLocalizer.fallingEdge();
		Navigation.turnTo(0);
		LightLocalizer_new.localize(); 
		advanced_beep(3);
		mycsData.start();
		CLAW_MOTOR.setSpeed(80);
		travel_TN(TN_LLx, TN_LLy, TN_URx, TN_URy, TN_Dir, TN_TR,true);
		 while(true) {
			first_scan = true;
			can_detected = false;
			Thread.sleep(200);
			while(!can_detected) {
				if(backup_scan) {
					Navigation.travelTo(SCAN_Pt2[0]*TILE_LENGTH, SCAN_Pt2[1]*TILE_LENGTH, false, 300);
				}
				complete_search();
			}
			
			IdentifyCan.detect_color();
			open_claw();
			IdentifyCan.weight();
			IdentifyCan.identify_can_beep(heavy,detected_color);

			if(backup_scan) {
				Navigation.travelTo(SCAN_Pt2[0]*TILE_LENGTH, SCAN_Pt2[1]*TILE_LENGTH, false, 300);
				backup_scan = false;
			}
			travel_SZ(SCAN_Pt1[0],SCAN_Pt1[1]);
			//--------------------------------------------
			travel_TN(TN_LLx, TN_LLy, TN_URx, TN_URy, TN_Dir, !TN_TR, false);
			//-------------------------------------------
			release_can();
			if(Corner == 0) {
				Navigation.turnTo(0);
			}else if(Corner == 1) {
				Navigation.turnTo(270);
			}else if(Corner == 2) {
				Navigation.turnTo(180);
			}else {
				Navigation.turnTo(90);
			}
			LightLocalizer_new.localize();
			travel_TN(TN_LLx, TN_LLy, TN_URx, TN_URy, TN_Dir, TN_TR,true);
		 }
	}
	
	
	/**
	 * This method is used to search cans in the searching zone. 
	 * If no can is detected in the first scan, and if the searching area has a rectangular shape, 
	 * travel to the second searching point and scan again. 
	 * If no can is detected in the first scan and if the searching area is a square, travel to the center of the square
	 * and make a 270-degree scan. 
	 * 
	 * @throws InterruptedException
	 */
	public static void complete_search() throws InterruptedException{
		travel_SZ(SCAN_Pt1[0],SCAN_Pt1[1]);
		if(backup_scan) {
			backup_scan = false;
		}
		if(first_scan) {
			advanced_beep(3);
			first_scan = false;
		}
		search_can(FACE_Dir,SCAN_Dir,100);
		if(!can_detected&&rect!=0) {
			backup_scan = true;
			travel_SZ(SCAN_Pt2[0],SCAN_Pt2[1]);
			search_can(FACE_Dir,SCAN_Dir,100);
		}else if(!can_detected&&rect==0) {
			travel_SZ((SZ_URx+SZ_LLx)/2,(SZ_URy+SZ_LLy)/2);
			search_can(correct_angle(FACE_Dir-90),SCAN_Dir,280);
		}
	}
	
	/**
	 * This method makes the robot travel to the search point, then it relocalizes itself.
	 * @param LLx X component of the lower left corner
	 * @param LLy Y component of the lower left corner
	 * @throws InterruptedException
	 */
	public static void travel_SZ(int LLx, int LLy) throws InterruptedException {
		Navigation.travelTo(((double)LLx)*TILE_LENGTH, ((double)LLy)*TILE_LENGTH, false, 300);
		relocalize_cross(LLx, LLy);
		
	}
	
	/**
	 * This method makes the robot travel to the entrance of the tunnel along paths parallel to x and y axis. 
	 * This method calculate how many lines needs to be passed in x and y axis, and the light sensor will use these values
	 * to correct itself while travelling. 
	 * 
	 * @param TN_LLx Low left x of the tunnel
	 * @param TN_LLy Low left y of the tunnel
	 * @param TN_URx Upper right x of the tunnel
	 * @param TN_URy Upper right y of the tunnel
	 * @param tunnel_dir In which direction if the tunnel, true if along x, false if along y
	 * @param travel_dir Travel direction through the tunnel, true if along positive x or y, false if along negative x or y
	 * @throws InterruptedException
	 */
	public static void travel_TN_EN(int TN_LLx, int TN_LLy, int TN_URx, int TN_URy,boolean tunnel_dir ,boolean travel_dir) throws InterruptedException {
		int count = 0;
		//int count_x;
		//int count_y;
		double target_x;
		double target_y;
		if(tunnel_dir&&travel_dir) {
			count_y = (int) (Math.abs(odo.getXYT()[1]-(TN_URy-0.5)*TILE_LENGTH)/TILE_LENGTH);
			count_x = (int) (Math.abs(odo.getXYT()[0]-(TN_LLx-0.5)*TILE_LENGTH)/TILE_LENGTH);
			target_y = (TN_URy-0.5)*TILE_LENGTH;
			target_x = (TN_LLx-0.5)*TILE_LENGTH;
		}else if(tunnel_dir&&!travel_dir){
			count_y = (int) (Math.abs(odo.getXYT()[1]-(TN_URy-0.5)*TILE_LENGTH)/TILE_LENGTH);
			count_x = (int) (Math.abs(odo.getXYT()[0]-(TN_URx+0.5)*TILE_LENGTH)/TILE_LENGTH);
			target_y = (TN_URy-0.5)*TILE_LENGTH;
			target_x = (TN_URx+0.5)*TILE_LENGTH;
		}else if(!tunnel_dir&&travel_dir) {
			count_y = (int) (Math.abs(odo.getXYT()[1]-(TN_LLy-0.5)*TILE_LENGTH)/TILE_LENGTH);
			count_x = (int) (Math.abs(odo.getXYT()[0]-(TN_LLx+0.5)*TILE_LENGTH)/TILE_LENGTH);
			target_y = (TN_LLy-0.5)*TILE_LENGTH;
			target_x = (TN_LLx+0.5)*TILE_LENGTH;
		}else {
			count_y = (int) (Math.abs(odo.getXYT()[1]-(TN_URy+0.5)*TILE_LENGTH)/TILE_LENGTH);
			count_x = (int) (Math.abs(odo.getXYT()[0]-(TN_URx-0.5)*TILE_LENGTH)/TILE_LENGTH);
			target_y = (TN_URy+0.5)*TILE_LENGTH;
			target_x = (TN_URx-0.5)*TILE_LENGTH;
		}
		
		int[] floor_color = getLightData();
		int[] current_color = getLightData();
		Navigation.travelTo(odo.getXYT()[0], target_y, true, 300);
		while(count<count_y) {
			current_color = getLightData();
			if(Math.abs(floor_color[0]-current_color[0])>100) {
				long t1 = System.currentTimeMillis();
				while(Math.abs(floor_color[0]-current_color[0])>100) {
					current_color = getLightData();
				}
				long t2 = System.currentTimeMillis();
				if((t2-t1)>65) {
					count++;
				}
				Thread.sleep(500);
			}
		}
		if(count_y==0) {
			Navigation.goForward(5,false);
		}else {
			Navigation.goForward(15,false);
		}
		line_correction(80);
		angle_correction();
		if(odo.getXYT()[2]<10||odo.getXYT()[2]>350) {
			odo.setY((int)(target_y/TILE_LENGTH)*TILE_LENGTH+LS_LENGTH);
		}else {
			odo.setY(((int)(target_y/TILE_LENGTH)+1)*TILE_LENGTH-LS_LENGTH);
		}
		Navigation.goForward(TILE_LENGTH/2-LS_LENGTH,false);
		count = 0;
		
		floor_color = getLightData();
		current_color = getLightData();
		Navigation.travelTo(target_x, odo.getXYT()[1], true, 300);
		while(count<count_x) {
			current_color = getLightData();
			if(Math.abs(floor_color[0]-current_color[0])>100) {
				long t1 = System.currentTimeMillis();
				while(Math.abs(floor_color[0]-current_color[0])>100) {
					current_color = getLightData();
				}
				long t2 = System.currentTimeMillis();
				if((t2-t1)>65) {
					count++;
				}
				Thread.sleep(500);
			}
		}
		if(count_x==0) {
			Navigation.goForward(5,false);
		}else {
			Navigation.goForward(15,false);
		}
		line_correction(80);
		angle_correction();
		if(odo.getXYT()[2]<135&&odo.getXYT()[2]>45) {
			odo.setX((int)(target_x/TILE_LENGTH)*TILE_LENGTH+LS_LENGTH);
		}else {
			odo.setX(((int)(target_x/TILE_LENGTH)+1)*TILE_LENGTH-LS_LENGTH);
		}
		Navigation.goForward(TILE_LENGTH/2-LS_LENGTH,false);
		
	}

	/**
	 * This method tells the robot in which direction the tunnel is (following the x axis or y axis)
	 * and then it computes the location of the middle of the tile that is located before the entrance of the tunnel
	 * so that the robot can travel to it after the localization. It will call the relocalize() method described further, 
	 * then travels through the tunnel. 
	 * 
	 * @param TNR_LLx Lower left x of tunnel
	 * @param TNR_LLy Lower left y of tunnel
	 * @param TNR_URx Upper right x of tunnel
	 * @param TNR_URy Upper right y of tunnel
	 * @param tunnel_dir direction of the tunnel, true if along x axis, false if along y axis
	 * @param travel_dir direction of travel, true if to the right or up, false if left or down
	 * @param start If travel to the searching zone, true; if travel back to the starting corver, fasle
	 * @throws InterruptedException
	 */
	public static void travel_TN(int TN_LLx, int TN_LLy, int TN_URx, int TN_URy,boolean tunnel_dir ,boolean travel_dir, boolean start) throws InterruptedException {
		if(start) {
			travel_TN_EN(TN_LLx, TN_LLy, TN_URx, TN_URy,tunnel_dir ,travel_dir);
			relocalize(Corner,tunnel_dir, travel_dir);
		}else {
			if(tunnel_dir&&travel_dir) {
				Navigation.travelTo((TN_LLx-0.5)*TILE_LENGTH, (TN_URy-0.5)*TILE_LENGTH, false, 300);
			}else if(tunnel_dir&&!travel_dir){
				Navigation.travelTo((TN_URx+0.5)*TILE_LENGTH, (TN_URy-0.5)*TILE_LENGTH, false, 300);
			}else if(!tunnel_dir&&travel_dir) {
				Navigation.travelTo((TN_LLx+0.5)*TILE_LENGTH, (TN_LLy-0.5)*TILE_LENGTH, false, 300);
			}else {
				Navigation.travelTo((TN_URx-0.5)*TILE_LENGTH, (TN_URy+0.5)*TILE_LENGTH, false, 300);
			}
			relocalize(Corner,tunnel_dir, travel_dir);
		}
		if(tunnel_dir&&travel_dir) {
			Navigation.travelTo((TN_URx+0.6)*TILE_LENGTH, (TN_URy-0.5)*TILE_LENGTH, false, 300);
		}else if(tunnel_dir&&!travel_dir){
			Navigation.travelTo((TN_LLx-0.6)*TILE_LENGTH, (TN_URy-0.5)*TILE_LENGTH, false, 300);
		}else if(!tunnel_dir&&travel_dir) {
			Navigation.travelTo((TN_LLx+0.5)*TILE_LENGTH, (TN_URy+0.6)*TILE_LENGTH, false, 300);
		}else {
			Navigation.travelTo((TN_URx-0.5)*TILE_LENGTH, (TN_LLy-0.6)*TILE_LENGTH, false, 300);
		}
	}
	
	/**
	 * The following method is divided in three parts:
	 * First compare the intensity of the floor and the intensity of what the light sensor is scanning.
	 * If the difference between the color of the floor and what the light sensor is currently measuring is smaller than 100
	 * then both readings are on the floor. If the difference is more than 100, the light sensor is on a black line.
	 * 
	 * Second, we check the values of the light sensor while the robot is going forward:
	 * if one of the two light sensors detects a black line, we make the wheel on its side stop.
	 * the robot keeps turning forward until the remaining light sensor also detects a black line. This allows the robot to 
	 * correct its angle and its position relatively to the chosen axis.
	 * 
	 * Thirdly, the same process is repeated after the robot turns to the direction along the other axis.
	 * For example, if the tunnel is on the X axis, it starts by localizing according to the Y axis, then the X axis.
	 * If the tunnel is on the Y axis, it localizes first according to the X axis, then according to the Y axis. 
	 * 
	 * @param corner starting corner of the robot
	 * @param direction direction of the tunnel
	 * @param TN_tr direction of travel, true if to the right or up, false if left or down
	 * @throws InterruptedException
	 */
	static void relocalize(int corner, boolean direction, boolean TN_tr) throws InterruptedException {
		int x = (int)(odo.getXYT()[0]/TILE_LENGTH);
		int y = (int)(odo.getXYT()[1]/TILE_LENGTH);
		if(direction) {
			if(corner==0||corner==1) {
				Navigation.turnTo(180);
			}else {
				Navigation.turnTo(0);
			}
		}else {
			if(corner==0||corner==3) {
				Navigation.turnTo(270);
			}else {
				Navigation.turnTo(90);
			}
		}
		
		Navigation.goBackward((int)TILE_LENGTH*0.6);
		line_correction(80);
		odo.setMotorSpeeds(100);
		if(direction) {
			if(corner==0||corner==1) {
				odo.setY((y+1)*TILE_LENGTH-LS_LENGTH);
				odo.setTheta(180);
			}else {
				odo.setY((y)*TILE_LENGTH+LS_LENGTH);
				odo.setTheta(0);
			}
		}else {
			if(corner==0||corner==3) {
				odo.setX((x+1)*TILE_LENGTH-LS_LENGTH);
				odo.setTheta(270);
			}else {
				odo.setX((x)*TILE_LENGTH+LS_LENGTH);
				odo.setTheta(90);
			}
		}
		Navigation.goForward(TILE_LENGTH/2-LS_LENGTH, false);
		if(direction) {
			if(TN_tr) {
				Navigation.turnTo(90);
			}else {
				Navigation.turnTo(270);
			}
		}else {
			if(TN_tr) {
				Navigation.turnTo(0);
			}else {
				Navigation.turnTo(180);
			}
		}
		Navigation.goBackward((int)TILE_LENGTH*0.6);
		line_correction(80);
		if(direction) {
			if(TN_tr) {
				odo.setX(x*TILE_LENGTH+LS_LENGTH);
			}else {
				odo.setX((x+1)*TILE_LENGTH-LS_LENGTH);
			}
		}else {
			if(TN_tr) {
				odo.setY(y*TILE_LENGTH+LS_LENGTH);
			}else {
				odo.setY((y+1)*TILE_LENGTH-LS_LENGTH);
			}
		}
		angle_correction();
		odo.setMotorSpeeds(100);
		
		Navigation.goForward(TILE_LENGTH/2-LS_LENGTH,false);
	}
	
	/**
	 * This method is very similar to the relocalize() method, except that we call it when we are on an intersection (lines crossing)
	 * @param x X component of the intersection
	 * @param y Y component of the intersection
	 * @throws InterruptedException
	 */
	static void relocalize_cross(double x, double y) throws InterruptedException{
		Navigation.turnTo(0);
		//Navigation.goBackward(TILE_LENGTH/3);
		line_correction(80);
		//odo.setMotorSpeeds(100);
		odo.setY(y*TILE_LENGTH+LS_LENGTH);
		odo.setTheta(0);
		Navigation.goBackward(LS_LENGTH);
		Navigation.turnTo(90);
		//Navigation.goBackward(TILE_LENGTH/3);
		line_correction(80);
		odo.setMotorSpeeds(100);
		odo.setX(x*TILE_LENGTH+LS_LENGTH);
		odo.setTheta(90);
		Navigation.goBackward(LS_LENGTH);
	}
	
	/**
	 * The robot reads the distances from the cans detected and records the smallest one and the position it was at when it read it.
	 * After scanning the area, it travels to the closest can.
	 * Through the half of the distance, scan the area again to make sure that the robot is going in the right direction.
	 * The robot will do a third scan if the target is still 20 cm or above away from the robot
	 * 
	 * @param face direction the robot faces before scanning
	 * @param dir scan direction, true if clockwise, false if counter clockwise
	 * @param searching_angle angle needs to be scaned
	 * @throws InterruptedException
	 */
	public static void search_can( int face, boolean dir, int search_angle) throws InterruptedException {
		can_detected = false;
		Navigation.turnTo(face); 
		int max_distance = (int)((Math.min(SZ_URx - SZ_LLx, SZ_URy - SZ_LLy)+0.5)*TILE_LENGTH);
		int[] target_position = {max_distance, correct_angle((int)odo.getXYT()[2]+30)};
		Navigation.spin(dir,search_angle,40,true);
		Boolean can_control = false;
		int[] possible_target_position = {max_distance, (int) correct_angle((int)odo.getXYT()[2]+30)};
		int filter = 0;
		while(odo.getMotor()[0].isMoving()) {
			int tmp_dis = myusData.getDistance();
			int tmp_ang = (int)odo.getXYT()[2];
			if(target_position[0]-tmp_dis>2&&!can_control) {
				can_control = true;
				filter += 1;
				possible_target_position[0] = tmp_dis;
				if(SCAN_Dir) {
					possible_target_position[1] = correct_angle(tmp_ang+5);
				}else {
					possible_target_position[1] = correct_angle(tmp_ang-5);
				}
			}else if(can_control&&filter<15) {
				if(Math.abs(tmp_dis-possible_target_position[0])<=2) {
					filter++;
				}else {
					can_control = false;
					filter = 0;
				}
			}else if (filter == 15){
				can_detected = true;
				target_position[0] = possible_target_position[0];
				target_position[1] = possible_target_position[1];
				can_control = false;
				filter = 0;
			}
		}
		if(!can_detected) {
			return;
		}
		Navigation.turnTo(target_position[1]);
		Navigation.goForward(target_position[0]/2,false);
		Navigation.spin(!dir,50,100,false);
		Navigation.spin(dir,100,40, true);
		while(odo.getMotor()[0].isMoving()) {
			int tmp_dis = myusData.getDistance();
			int tmp_ang = (int)odo.getXYT()[2];
			if(target_position[0]-tmp_dis>2&&!can_control) {
				can_control = true;
				filter += 1;
				possible_target_position[0] = tmp_dis;
				if(SCAN_Dir) {
					possible_target_position[1] = correct_angle(tmp_ang+5);
				}else {
					possible_target_position[1] = correct_angle(tmp_ang-5);
				}
			}else if(can_control&&filter<15) {
				if(Math.abs(tmp_dis-possible_target_position[0])<=2) {
					filter++;
				}else {
					can_control = false;
					filter = 0;
				}
			}else {
				target_position[0] = possible_target_position[0];
				target_position[1] = possible_target_position[1];
				can_control = false;
				filter = 0;
			}
		}
		Navigation.turnTo(target_position[1]);
		if(target_position[0]>20) {
			Navigation.goForward(target_position[0]/2,false);
			Navigation.spin(!dir,45,100,false);
			Navigation.spin(dir,90,50, true);
			while(odo.getMotor()[0].isMoving()) {
				int tmp_dis = myusData.getDistance();
				int tmp_ang = (int)odo.getXYT()[2];
				if(target_position[0]-tmp_dis>2&&!can_control) {
					can_control = true;
					filter += 1;
					possible_target_position[0] = tmp_dis;
					if(SCAN_Dir) {
						possible_target_position[1] = correct_angle(tmp_ang+5);
					}else {
						possible_target_position[1] = correct_angle(tmp_ang-5);
					}
				}else if(can_control&&filter<15) {
					if(Math.abs(tmp_dis-possible_target_position[0])<=2) {
						filter++;
					}else {
						can_control = false;
						filter = 0;
					}
				}else {
					target_position[0] = possible_target_position[0];
					target_position[1] = possible_target_position[1];
					can_control = false;
					filter = 0;
				}
			}
			Navigation.turnTo(target_position[1]);
		}
		
		Navigation.goForward(target_position[0]*2,true);
		while(LEFT_MOTOR.isMoving()) {
			if(myusData.getDistance()<=4) {
				can_detected = true;
				odo.stop();
			}else {
				can_detected = false;
			}
		}
	}
	
	/**
	 * This method makes the robot go to the starting corner and release the can. 
	 * It makes the robot to travel to the starting corner following paths along x and y axis. 
	 * @throws InterruptedException 
	 */
	public static void release_can() throws InterruptedException{
		int target_x;
		int target_y;
		if(Corner == 0) {
			target_x = 1;
			target_y = 1;
		}else if(Corner == 1) {
			target_x = 14;
			target_y = 1;
		}else if(Corner == 2) {
			target_x = 14;
			target_y = 8;
		}else {
			target_x = 1;
			target_y = 8;
		}
		int count = 0;
		int[] floor_color = getLightData();
		int[] current_color = getLightData();
		if(TN_Dir) {
			Navigation.travelTo(target_x*TILE_LENGTH, odo.getXYT()[1], true, 300);
			while(count<count_x) {
				current_color = getLightData();
				if(Math.abs(floor_color[0]-current_color[0])>100) {
					long t1 = System.currentTimeMillis();
					while(Math.abs(floor_color[0]-current_color[0])>100) {
						current_color = getLightData();
					}
					long t2 = System.currentTimeMillis();
					if((t2-t1)>65) {
						count++;
					}
					Thread.sleep(500);
				}
			}
			if(count_x==0) {
				Navigation.goForward(5,false);
			}else {
				Navigation.goForward(15,false);
			}
			line_correction(80);
			angle_correction();
			if(odo.getXYT()[2]<135&&odo.getXYT()[2]>45) {
				odo.setX(target_x*TILE_LENGTH+LS_LENGTH);
			}else {
				odo.setX((target_x)*TILE_LENGTH-LS_LENGTH);
			}
			Navigation.goBackward(LS_LENGTH);
		}else {
			Navigation.travelTo(odo.getXYT()[0], target_y*TILE_LENGTH, true, 300);
			while(count<count_y) {
				current_color = getLightData();
				if(Math.abs(floor_color[0]-current_color[0])>100) {
					long t1 = System.currentTimeMillis();
					while(Math.abs(floor_color[0]-current_color[0])>100) {
						current_color = getLightData();
					}
					long t2 = System.currentTimeMillis();
					if((t2-t1)>65) {
						count++;
					}
					Thread.sleep(500);
				}
			}
			if(count_y==0) {
				Navigation.goForward(5,false);
			}else {
				Navigation.goForward(15,false);
			}
			line_correction(80);
			angle_correction();
			if(odo.getXYT()[2]<10||odo.getXYT()[2]>350) {
				odo.setY((target_y)*TILE_LENGTH+LS_LENGTH);
			}else {
				odo.setY((target_y)*TILE_LENGTH-LS_LENGTH);
			}
			Navigation.goBackward(LS_LENGTH);
		}
		count = 0;
		floor_color = getLightData();
		current_color = getLightData();
		if(TN_Dir) {
			Navigation.travelTo(odo.getXYT()[0], target_y*TILE_LENGTH, true, 300);
			Thread.sleep(750);
			while(count<count_y) {
				current_color = getLightData();
				if(Math.abs(floor_color[0]-current_color[0])>100) {
					long t1 = System.currentTimeMillis();
					while(Math.abs(floor_color[0]-current_color[0])>100) {
						current_color = getLightData();
					}
					long t2 = System.currentTimeMillis();
					if((t2-t1)>65) {
						count++;
					}
					Thread.sleep(500);
				}
			}
			if(count_y==0) {
				Navigation.goForward(5,false);
			}else {
				Navigation.goForward(15,false);
			}
			line_correction(80);
			angle_correction();
			if(odo.getXYT()[2]<10||odo.getXYT()[2]>350) {
				odo.setY((target_y)*TILE_LENGTH+LS_LENGTH);
			}else {
				odo.setY((target_y)*TILE_LENGTH-LS_LENGTH);
			}
			Navigation.goBackward(LS_LENGTH);
		}else {
			Navigation.travelTo(target_x*TILE_LENGTH, odo.getXYT()[1], true, 300);
			Thread.sleep(750);
			while(count<count_x) {
				current_color = getLightData();
				if(Math.abs(floor_color[0]-current_color[0])>100) {
					long t1 = System.currentTimeMillis();
					while(Math.abs(floor_color[0]-current_color[0])>100) {
						current_color = getLightData();
					}
					long t2 = System.currentTimeMillis();
					if((t2-t1)>65) {
						count++;
					}
					Thread.sleep(500);
				}
			}
			if(count_x==0) {
				Navigation.goForward(5,false);
			}else {
				Navigation.goForward(15,false);
			}
			line_correction(80);
			angle_correction();
			if(odo.getXYT()[2]<135&&odo.getXYT()[2]>45) {
				odo.setX(target_x*TILE_LENGTH+LS_LENGTH);
			}else {
				odo.setX((target_x)*TILE_LENGTH-LS_LENGTH);
			}
			Navigation.goBackward(LS_LENGTH);
		}
		
		if(Corner == 0) {
			Navigation.turnTo(45);
		}else if(Corner == 1) {
			Navigation.turnTo(315);
		}else if(Corner == 2) {
			Navigation.turnTo(225);
		}else {
			Navigation.turnTo(135);
		}
		CLAW_MOTOR.rotate(230); 
		advanced_beep(5);
		Thread.sleep(500);
		Navigation.goForward(10,false); 
		CLAW_MOTOR.rotate(-250,false); 
		Navigation.goBackward(10); 
	}
	
	/**
	 * This method make the robot turn backward, open the claw and go backward to make the can in position to be lifted
	 */
	public static void open_claw(){
		CLAW_MOTOR.rotate(250,true);
		Navigation.goBackward(20);
		Navigation.turnBackward();
		Navigation.goBackward(10);
	}

	/**
	 * This method gets the intensity measured by the two light sensors
	 * we record these values in an array with the slot 0 for the left 
	 * light sensor and 1 for the right light sensor.
	 * 
	 * @return light intensity array
	 */
	public static int[] getLightData() {
		LIGHT_VALUE_L.fetchSample(lightDataL,0);
		LIGHT_VALUE_R.fetchSample(lightDataR,0);
		int[] color = new int[2];
		color[0] = (int)(lightDataL[0] * 1000);
		color[1] = (int)(lightDataR[0] * 1000);
		
		return color;
	}

	/**
	 * The following method allows the robot to turn to the minimum angle,
	 * thus if the angle is bigger than 360, we subtract 360 from the current angle.
	 * If it is smaller than 0, we add 360.
	 * 
	 * @param angle input angle
	 * @return angle corrected angle
	 */
	public static int correct_angle(int angle){
		if(angle>=360) {
			return angle-360;
		}else if(angle<0) {
			return angle + 360;
		}else {
			return angle;
		}
	}
	
	/**
	 * This method contains a for loop and a times variables. the robot will beep the number of times taken in argument by advanced_beep
	 * @param times Number of times we want the robot to beep.
	 * @throws InterruptedException 
	 */
	public static void advanced_beep(int times) throws InterruptedException{
		for(int i = 0; i<times; i++) {
			Sound.beep();
			Thread.sleep(100);
		}
	}
	
	/**
	 * 
	 * This method makes position correction using the two light sensor at the back. If one the the light sensor detects a line, 
	 * the wheel on the same side will stop and the wheel on the other side will keep moving, until the second light sensor also 
	 * hit the line. In this way, both the direction and the position can be corrected. 
	 * 
	 * @param speed
	 * @throws InterruptedException
	 */
	public static void line_correction(int speed) throws InterruptedException{
		int[] floor_color = getLightData();
		int[] current_color = getLightData();
		odo.setMotorSpeeds(speed);
		while(Math.abs(floor_color[0]-current_color[0])<100&&Math.abs(floor_color[1]-current_color[1])<100) {
			odo.moveforward();
			Thread.sleep(50);
			current_color = getLightData();
		}
		odo.stop();
		if(Math.abs(floor_color[0]-current_color[0])<100) {
			while(Math.abs(floor_color[0]-current_color[0])<100) {
				odo.getMotor()[0].forward();
				Thread.sleep(50);
				current_color = getLightData();
			}
		}else if(Math.abs(floor_color[1]-current_color[1])<100){
			while(Math.abs(floor_color[1]-current_color[1])<100) {
				odo.getMotor()[1].forward();
				Thread.sleep(50);
				current_color = getLightData();
			}
		}
		odo.stop();
	}
	
	/**
	 * This method make angle corrections to the odometer. If the angle is samller than 45 or greater than 315, set the angle ot 0. 
	 * If the angle is between 45 and 135, set the angle to 90. 
	 * If the angle is between 225 and 135, set the angle to 180. 
	 * If the angle is between 315 and 225, set the angle to 270. 
	 */
	public static void angle_correction() {
		if(odo.getXYT()[2]<45||odo.getXYT()[2]>315) {
			odo.setTheta(0);
		}else if(odo.getXYT()[2]<135) {
			odo.setTheta(90);
		}else if(odo.getXYT()[2]<225) {
			odo.setTheta(180);
		}else {
			odo.setTheta(270);
		}
	}
}

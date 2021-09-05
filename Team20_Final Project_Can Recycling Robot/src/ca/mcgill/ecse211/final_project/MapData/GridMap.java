package ca.mcgill.ecse211.final_project.MapData;

import static ca.mcgill.ecse211.final_project.finalproject.Resources.*;
/**
 * This class determines more specific parameters of the map, including tunnel direction, 
 * scanning points at the search zone, orientation before starting the scan, direction of scanning, 
 * and the travel direction of the tunnel. 
 * 
 * @author Xiangyun Wang
 *
 */
public class GridMap {
	/**
	 * This method update TN_Dir, SCAN_Pt1, SCAN_Pt2, FACE_Dir, SCAN_Dir, TN_TR, rect
	 */
	public static void update_map(){
		if((SZ_URx - SZ_LLx)>(SZ_URy - SZ_LLy)) {
			rect = 1;
		}else if((SZ_URx - SZ_LLx)<(SZ_URy - SZ_LLy)) {
			rect = -1;
		}
		
		if(TN_URx-TN_LLx==1) {
			TN_Dir = false;
		}else {
			TN_Dir = true;
		}
		if(Corner == 0) {
			if(TN_Dir) {
				SCAN_Pt1 = new int[]{SZ_LLx, SZ_URy};
				FACE_Dir = 70;
				SCAN_Dir = true;
				TN_TR = true;
			}else {
				SCAN_Pt1 = new int[]{SZ_URx, SZ_LLy};
				FACE_Dir = 20;
				SCAN_Dir = false;
				TN_TR = true;
			}
		}else if(Corner == 1) {
			if(TN_Dir) {
				SCAN_Pt1 = new int[]{SZ_URx, SZ_URy};
				FACE_Dir = 290;
				SCAN_Dir = false;
				TN_TR = false;
			}else {
				SCAN_Pt1 = new int[]{SZ_LLx, SZ_LLy};
				FACE_Dir = 340;
				SCAN_Dir = true;
				TN_TR = true;
			}
		}else if(Corner == 2) {
			if(TN_Dir) {
				SCAN_Pt1 = new int[]{SZ_URx, SZ_LLy};
				FACE_Dir = 250;
				SCAN_Dir = true;
				TN_TR = false;
			}else {
				SCAN_Pt1 = new int[]{SZ_LLx, SZ_URy};
				FACE_Dir = 200;
				SCAN_Dir = false;
				TN_TR = false;
			}
		}else {
			if(TN_Dir) {
				SCAN_Pt1 = new int[]{SZ_LLx, SZ_LLy};
				FACE_Dir = 110;
				SCAN_Dir = false;
				TN_TR = true;
			}else {
				SCAN_Pt1 = new int[]{SZ_URx, SZ_URy};
				FACE_Dir = 160;
				SCAN_Dir = true;
				TN_TR = false;
			}
		}
		if(rect == 1) {
			SCAN_Pt2 = new int[]{(SZ_URx+SZ_LLx)/2, SCAN_Pt1[1]};
			if(TN_TR) {
				if(Math.hypot(TN_URx-SCAN_Pt2[0], TN_URy-SCAN_Pt2[1])<Math.hypot(TN_URx-SCAN_Pt1[0], TN_URy-SCAN_Pt1[1])) {
					swap();
				}
			}else {
				if(Math.hypot(TN_LLx-SCAN_Pt2[0], TN_LLy-SCAN_Pt2[1])<Math.hypot(TN_LLx-SCAN_Pt1[0], TN_LLy-SCAN_Pt1[1])) {
					swap();
				}
			}
		}else if (rect == -1){
			SCAN_Pt2 = new int[]{SCAN_Pt1[0],(SZ_URy+SZ_LLy)/2};
			if(TN_TR) {
				if(Math.hypot(TN_URx-SCAN_Pt2[0], TN_URy-SCAN_Pt2[1])<Math.hypot(TN_URx-SCAN_Pt1[0], TN_URy-SCAN_Pt1[1])) {
					swap();
				}
			}else {
				if(Math.hypot(TN_LLx-SCAN_Pt2[0], TN_LLy-SCAN_Pt2[1])<Math.hypot(TN_LLx-SCAN_Pt1[0], TN_LLy-SCAN_Pt1[1])) {
					swap();
				}
			}
		}
	}
	
	/**
	 * This method swap the contant of SCAN_Pt1 and SCAN_Pt2
	 */
	private static void swap() {
		int[] tmp = new int[] {SCAN_Pt1[0], SCAN_Pt1[1]};
		SCAN_Pt1 = SCAN_Pt2;
		SCAN_Pt2 = tmp;
	}
	
}

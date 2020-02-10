package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.Resources;
public class Navigation {

	
	public Navigation() {
		
	}
	
	public static void travelTo(double x, double y) {
		Resources.leftMotor.setSpeed(100);
		Resources.rightMotor.setSpeed(100);
		x = x*Resources.TILE_SIZE;
		y = y*Resources.TILE_SIZE;
		double[] currentXYT = Resources.odometer.getXyt();
		double currentX = currentXYT[0];
		double currentY = currentXYT[1];
		double currentT = currentXYT[2];
		
		double deltaX = x - currentX;
		double deltaY = y - currentY;
		double distance = Math.sqrt(deltaX* deltaX + deltaY * deltaY);
		
		double targetAngle;
		
		if(deltaX == 0) targetAngle = 0;
		if(deltaY == 0) targetAngle = (deltaX > 0)? 270 : 90; 
		
		System.out.println("DeltaX is "+ deltaX + "deltaY is "+ deltaY);
		
		if(deltaX > 0) {
			if (deltaY > 0) {
				targetAngle = 360 - 57.3* Math.atan(deltaX/deltaY);
			}else targetAngle = 180 +57.3* Math.atan(deltaX/(-deltaY)); 
		}else {
			if (deltaY > 0) {
				targetAngle = 57.3*Math.atan(-deltaX/deltaY);
			}else targetAngle = 180 - 57.3*Math.atan(deltaX/deltaY); 
		}
		
		System.out.println("Target angle: "+ targetAngle + "Distance " + distance);
		turnTo(targetAngle, currentT);
		CircleTurningDriver.moveStraightFor(distance);
	}
	
	public static void turnTo (double targetAngle, double currentAngle) {
		double theta = targetAngle - currentAngle;
		if(theta > 0) {
			if(theta > 180) {
				theta = theta - 360;
			}
		}else {
			if( - theta > 180) {
				theta = 360 + theta;
			}
		}
		CircleTurningDriver.turnBy(-theta);
	}
}

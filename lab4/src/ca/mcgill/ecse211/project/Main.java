package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.internal.ev3.EV3Audio;
import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.project.USDriver;
import ca.mcgill.ecse211.project.CircleTurningDriver;

/**
 * The main driver class for the lab.
 */

public class Main {
  /**
   * The main entry point.
   * 
   * @param args not used
   */
  public static void main(String[] args) {
	 ColorSensor colorSensorController = new ColorSensor();
	 Thread odo = new Thread(Resources.odometer);
	 Thread csThread = new Thread(colorSensorController);     
	 while(Button.waitForAnyPress() != Button.ID_ENTER);	  
	 odo.start();
	 CircleTurningDriver.rotateClockwise();
	 System.out.println("Start rotating");
	 csThread.run();
	 
	 while(Button.waitForAnyPress() != Button.ID_ENTER);	  
	 Navigation.travelTo(1, 2);
	 Navigation.travelTo(2, 2);
	 Navigation.travelTo(3, 3);
	 Navigation.travelTo(3, 2);
	 Navigation.travelTo(2, 1);
	 
	 Thread csThread2 = new Thread(colorSensorController);    
	 csThread2.run();
//	 boolean isOnLine = false;
//	 boolean preOnLine= false;
//	 double[] angles = new double[4];
//	 int count = 0;
//	 while(true) {
//		 isOnLine = colorSensorController.isOnLine();
//		 System.out.println("Count: " + count);
//		 if(!preOnLine && isOnLine) {
//			 angles[count] = odometer.getXyt()[2];
//			 if(count ==3) {
//				 CircleTurningDriver.stopMotors();
//				 break;
//			 }
//			 count ++;
//		 }
//		 preOnLine = isOnLine;
//		 sleepFor(10);
//	 }
//	 
//	 double deltax = LIGHT_RADIUS * Math.cos((angles[3] - angles[1])/2);
//	 
//	 double deltay = LIGHT_RADIUS * Math.cos((angles[2] - angles[0])/2);
//
//	 double thetay = angles[3] - angles[1];
//	 
//	 CircleTurningDriver.turnBy(90 - thetay/2);
//	 
//	 CircleTurningDriver.moveStraightFor(deltay);
//	 CircleTurningDriver.turnBy(90);
//	 CircleTurningDriver.moveStraightFor(-deltax);
//	 CircleTurningDriver.turnBy(-90);
	  
//	  ColorSensor colorSensor=new ColorSensor();
//	  
//	  Thread colorSensorThread = new Thread(colorSensor);
//	  
//	  colorSensorThread.run();
    //Create threads for USDriver and Odometer
//    USDriver usDriver = new USDriver();
//    Thread usThread = new Thread(usDriver);
//    Thread odo = new Thread(Resources.odometer);
//    
//    //Wait enter button pressed to start
//    while(Button.waitForAnyPress() != Button.ID_ENTER);
//    
//    //start USDriver, Odometer and Dispay threads
//    usThread.start();
//    odo.start(); 
//    new Thread(new Display()).start();

//    
    //wait for exit
    while (Button.waitForAnyPress() != Button.ID_ALL) {
    } // do nothing
    
    System.exit(0);
  }

  /**
   * Sleeps current thread for the specified duration.
   * 
   * @param duration sleep duration in milliseconds
   */
  public static void sleepFor(long duration) {
    try {
      Thread.sleep(duration);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
  }
  
  public double calculateDeltaX(double firstAngle, double secondAngle) {
	  double theta = (secondAngle - firstAngle)/2;
	  double dx = LIGHT_RADIUS * Math.cos(theta);
	  if(firstAngle > 180) {
		  return -dx;
	  }else return dx;
  }
  
  public double calculateDeltaY(double firstAngle, double secondAngle) {
	  if(firstAngle < secondAngle) {
		  return LIGHT_RADIUS * Math.cos((secondAngle-firstAngle)/2);
	  }
	  else return -LIGHT_RADIUS * Math.cos((firstAngle - secondAngle)/2);
  }
  
  public void displacementCorrection(double deltax, double deltay) {
	  
	  CircleTurningDriver.moveStraightFor(-deltay);
	  CircleTurningDriver.turnBy(90);
	  CircleTurningDriver.moveStraightFor(-deltax);
	  CircleTurningDriver.turnBy(-90);
  }
  
  
//  public void turnTo(double angle) {
//	  double[] position = odometer.getXyt();
//	  double theta = position[2];
//	  double dTheta;
//	  System.out.println("theta is "+ theta + "angle is "+angle);
//	  if(angle >theta) {
//		  dTheta = angle -theta;
//		  if (dTheta > 180) {
//			  CircleTurningDriver.turnBy(360- dTheta);
//		  }else {
//			  CircleTurningDriver.turnBy( -dTheta);
//		  }
//		  
//	  }else {
//		  dTheta = theta - angle;
//		  if (dTheta > 180) {
//			  CircleTurningDriver.turnBy(dTheta - 360);
//		  }else {
//			  CircleTurningDriver.turnBy(dTheta);
//		  }
//	  }
//  }
  
  public void localize() {
	 CircleTurningDriver.rotateClockwise();
	 ColorSensor colorSensorController = new ColorSensor();
	 Thread csThread = new Thread(colorSensorController);
	 csThread.run();
	 boolean isOnLine = false;
	 boolean preOnLine= false;
	 double[] angles = new double[4];
	 int count = 0;
	 while(true) {
		 isOnLine = colorSensorController.isOnLine();
		 if(!preOnLine && isOnLine) {
			 angles[count] = odometer.getXyt()[2];
			 if(count ==3) {
				 CircleTurningDriver.stopMotors();
				 break;
			 }
			 
		 }
		 sleepFor(20);
	 }
	 
	 double deltax = calculateDeltaX(angles[1], angles[3]);
	 double deltay = calculateDeltaY(angles[0], angles[2]);
	 
	 double thetay = angles[3] - angles[1];
	 
	 CircleTurningDriver.turnBy(90 - thetay/2 - 1);
	 
	 CircleTurningDriver.moveStraightFor(-deltay);
	 CircleTurningDriver.turnBy(90);
	 CircleTurningDriver.moveStraightFor(-deltax);
	 CircleTurningDriver.turnBy(-90);
	 
  }
  
//  public static void 
  
  
  
  
//  public void a
}

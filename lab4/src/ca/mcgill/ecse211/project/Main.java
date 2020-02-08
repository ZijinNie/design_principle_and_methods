package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.internal.ev3.EV3Audio;
import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.project.USDriver;
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

    //Create threads for USDriver and Odometer
    USDriver usDriver = new USDriver();
    Thread usThread = new Thread(usDriver);
    Thread odo = new Thread(Resources.odometer);
    
    //Wait enter button pressed to start
    while(Button.waitForAnyPress() != Button.ID_ENTER);
    
    //start USDriver, Odometer and Dispay threads
    usThread.start();
    odo.start(); 
    new Thread(new Display()).start();
    
    //Set rotate speed
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    //let robot rotate clockwise
    CircleTurningDriver.rotateClockwise();
    
    //wait USDriver stops
    while(!usDriver.isExit());
    
    //thread sleeps for one second
    sleepFor(1000);
    
    //get angles of first and second time of swiping through D
    double firstAngle = usDriver.getFirstAngle(), secondAngle= usDriver.getSecondAngle();
    
    boolean isFirstFromUp = usDriver.isFirstUpRising();
    
    //calculate dTheta based on the rising/falling edge of first D distance
    double dTheta = (isFirstFromUp) ?  315 - (secondAngle - firstAngle) /2  : 135 - ( secondAngle - firstAngle)/2;
    
    //angle correct
    int correction = -7;
    
    //rotate to counter clockwise if angle greater than 180
    if(dTheta > 180) {
      dTheta = dTheta - 360;
    } else {
      dTheta += correction;     //add correction
    }
    
    //turn to 0 degree
    CircleTurningDriver.turnBy(dTheta);
    
    //wait for button press to go to (1,1)
    while (Button.waitForAnyPress() != Button.ID_ENTER);
    
    //get minimum distance from USSensor
    double minDist = usDriver.getMinDist();
    
    //Correction made due to the horizontal distance difference of US sensor and wheel base 
    double distanceCorrection = 3;
    //Moving to (1,1)
    CircleTurningDriver.moveStraightFor(TILE_SIZE - minDist - distanceCorrection);
    CircleTurningDriver.turnBy(90);
    CircleTurningDriver.moveStraightFor(TILE_SIZE - minDist - distanceCorrection);
    CircleTurningDriver.turnBy(-90);
    
    //wait for exit
    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
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
		  return dx;
	  }else return -dx;
  }
  
  public double calculateDeltaY(double firstAngle, double secondAngle) {
	  if(firstAngle < secondAngle) {
		  return LIGHT_RADIUS * Math.cos((secondAngle-firstAngle)/2);
	  }
	  else return -LIGHT_RADIUS * Math.cos((firstAngle - secondAngle)/2);
  }
  
  public void moveTo(int x, int y) {
	  
  }
}

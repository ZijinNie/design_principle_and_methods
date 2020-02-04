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

    
    Sound.beep();
    USDriver usDriver= new USDriver();
    Thread usThread = new Thread(usDriver);
    Thread odo = new Thread(Resources.odometer);
    
    while(Button.waitForAnyPress() != Button.ID_ENTER);
    
    usThread.start();
    odo.start(); 
    new Thread(new Display()).start();
    
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
//    CircleTurningDriver.turnBy(360);
    CircleTurningDriver.rotateClockwise();

    while(!usDriver.isExit());
    sleepFor(1000);
    double firstAngle = usDriver.getFirstAngle(), secondAngle= usDriver.getSecondAngle();
    boolean isFirstFromUp = usDriver.isFirstUpRising();
    double dTheta = (isFirstFromUp) ?  215 - (secondAngle - firstAngle) /2  : 135 - ( secondAngle - firstAngle)/2;
    
    System.out.println(firstAngle + "     "+ secondAngle);
    System.out.println(dTheta);

    
    double minDist = usDriver.getMinDist();
    System.out.println("Forward" + (TILE_SIZE-minDist-2));
    CircleTurningDriver.turnBy(dTheta);
    while (Button.waitForAnyPress() != Button.ID_ENTER);
    
    CircleTurningDriver.moveStraightFor(TILE_SIZE-minDist   -15    );
    CircleTurningDriver.turnBy(90);
    CircleTurningDriver.moveStraightFor(TILE_SIZE - minDist    -15    );
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
  
}

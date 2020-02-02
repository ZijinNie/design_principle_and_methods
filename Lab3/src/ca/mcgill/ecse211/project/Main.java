package ca.mcgill.ecse211.project;

//static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.project.Resources.*;

import lejos.hardware.Button;

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
    float[] record; 

//    Printer.printMainMenu();
    USDriver usDriver= new USDriver();
    CircleTurningDriver circleturningDriver = new CircleTurningDriver();
    Thread usThread = new Thread(usDriver);

    Thread driverThread = new Thread(circleturningDriver);
    while(Button.waitForAnyPress() != Button.ID_ENTER);

    
//    new Thread(new Display()).start();
    usThread.start();
    driverThread.start();
    
    sleepFor(1000);
    
    while(!circleturningDriver.isStopped());
    usDriver.stop();
    
    record = usDriver.getUsRecord();
    
    
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

package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
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
    
    record = usDriver.getUsRecord();
    usDriver.stop();
    SignalAnalyzer sa = new SignalAnalyzer(10,record);
    
    int angle = sa.getInitialAngle();
    float xDist = 10;
    float yDist = 10;
    
    System.out.println(angle);
    
    while (Button.waitForAnyPress() != Button.ID_ENTER);
    
    circleturningDriver.setTurningAngle( (angle+135)%360 );
    circleturningDriver.setxDistance(xDist);
    circleturningDriver.setyDistance(yDist);
    
    circleturningDriver.setStartHeadingDes(true);
    
    
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

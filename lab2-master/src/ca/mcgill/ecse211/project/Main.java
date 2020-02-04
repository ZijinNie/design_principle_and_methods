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
    int buttonChoice;
    new Thread(odometer).start();
    
    buttonChoice = chooseDriveInSquareOrFloatMotors();

    if (buttonChoice == Button.ID_LEFT) {
      floatMotors();
    } else if (buttonChoice == Button.ID_RIGHT) {
      SquareDriver.drive();
    }
    
    new Thread(new Display()).start();
    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
    } // do nothing
    
    System.exit(0);
  }

  /**
   * Floats the motors.
   */
  public static void floatMotors() {
    leftMotor.forward();
    leftMotor.flt();
    rightMotor.forward();
    rightMotor.flt();
  }

  /**
   * Asks the user whether the motors should drive in a square or float.
   * 
   * @return the user choice
   */
  private static int chooseDriveInSquareOrFloatMotors() {
    int buttonChoice;
    Display.showText("< Left | Right >",
                     "       |        ",
                     " Float | Drive  ",
                     "motors | in a   ",
                     "       | square ");
    
    do {
      buttonChoice = Button.waitForAnyPress(); // left or right press
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
    return buttonChoice;
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

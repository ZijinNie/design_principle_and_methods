package ca.mcgill.ecse211.lab1;

import static ca.mcgill.ecse211.lab1.Resources.*;

import lejos.hardware.Button;

/**
 * Main class of the program.
 */
public class Main {
  
  /**
   * The US controller selected by the user (bang-bang or P-type).
   */
  public static UltrasonicController selectedController;
  
  /**
   * String for controller type
   */
  public static String controllerType;

  /**
   * Main entry point - instantiate objects used and set up sensor.
   * 
   * @param args not used
   */
  public static void main(String[] args) {
    // Set up the display on the EV3 screen and wait for a button press. 
    // The button ID (option) determines what type of control to use
    Printer.printMainMenu();
    int option = Button.waitForAnyPress();

    if (option == Button.ID_LEFT) {
      controllerType = "BangBang";
    } else if (option == Button.ID_RIGHT) {
    	controllerType = "PType";
    } else {
      showErrorAndExit("Error - invalid button!");
    }

    // Start the controller and printer threads
    selectedController = new UltrasonicController(controllerType);
    new Thread(selectedController).start();
    new Thread(new Printer()).start();

    // Wait here until button pressed to terminate wall follower
    Button.waitForAnyPress();
    System.exit(0);
  }

  /**
   * Shows error and exits program.
   */
  public static void showErrorAndExit(String errorMessage) {
    TEXT_LCD.clear();
    System.err.println(errorMessage);
    
    // Sleep for 2s so user can read error message
    sleepFor(2000);
    
    System.exit(-1);
  }

  /**
   * Sleeps for the specified duration.
   * @param millis the duration in milliseconds
   */
  public static void sleepFor(long millis) {
    try {
      Thread.sleep(millis);
    } catch (InterruptedException e) {
      // Nothing to do here
    }
  }
  
}

package ca.mcgill.ecse211.lab1;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * Class for static resources (things that stay the same throughout the entire program execution),
 * like constants and hardware.
 * <br><br>
 * Use these resources in other files by adding this line at the top (see examples):<br><br>
 * 
 * {@code import static ca.mcgill.ecse211.project.Resources.*;}
 */
public class Resources {
  //Parameters: adjust these for desired performance
  
  /**
   * Ideal distance between the sensor and the wall (cm).
   */
  public static final int WALL_DIST = 20;
  
  /**
   * Width of the maximum tolerated deviation from the ideal {@code WALL_DIST}, also known as the
   * dead band. This is measured in cm.
   */
  public static final int WALL_DIST_ERR_THRESH = 3;
  
  /**
   * Speed of slower rotating wheel (deg/sec).
   */
  public static final int MOTOR_LOW = 100;
  
  /**
   * Speed of the faster rotating wheel (deg/sec).
   */
  public static final int MOTOR_HIGH = 200;
  
  /**
   * The limit of invalid samples that we read from the US sensor before assuming no obstacle.
   */
  public static final int INVALID_SAMPLE_LIMIT = 20;
  
  /**
   * The poll sleep time, in milliseconds.
   */
  public static final int POLL_SLEEP_TIME = 50;
  
  // Hardware resources
  /**
   * The LCD screen used for displaying text.
   */
  public static final TextLCD TEXT_LCD = LocalEV3.get().getTextLCD();
  
  /**
   * The ultrasonic sensor.
   */
  public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
  
  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
  
  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.D);

}

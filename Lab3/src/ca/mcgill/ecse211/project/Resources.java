package ca.mcgill.ecse211.project;


import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class is used to define static resources in one place for easy access and to avoid 
 * cluttering the rest of the codebase. All resources can be imported at once like this:
 * 
 * <p>{@code import static ca.mcgill.ecse211.lab3.Resources.*;}
 */
public class Resources {
  
  /**
   * The wheel radius in centimeters.
   */
  public static final double WHEEL_RAD = 2.130;
  
  /**
   * The robot width in centimeters.
   */
  public static final double BASE_WIDTH = 14.50;
  
  /**
   * The speed at which the robot moves forward in degrees per second.
   */
  public static final int FORWARD_SPEED = 150;
  
  /**
   * The speed at which the robot rotates in degrees per second.
   */
  public static final int ROTATE_SPEED = 50;
  
  /**
   * The motor acceleration in degrees per second squared.
   */
  public static final int ACCELERATION = 3000;
  
  /**
   * Timeout period in milliseconds.
   */
  public static final int TIMEOUT_PERIOD = 3000;
  
  /**
   * The tile size in centimeters. Note that 30.48 cm = 1 ft.
   */
  public static final double TILE_SIZE = 30.48;
  
  /**
   * The size of the array storing US sensor data
   */
  
  public static final int USRECORD_SIZE = 200;
  
  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);

  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.D);
  
  
  /**
   * The LCD.
   */
  public static final TextLCD lcd = LocalEV3.get().getTextLCD();
  
  /**
   * The limit of invalid samples that we read from the US sensor before assuming no obstacle.
   */
  public static final int INVALID_SAMPLE_LIMIT = 5;
  
  /**
   * The distance limit for detecting a wall
   */
  public static final float TURNING_THRESHOLD = 30;
  
  /**
   * The error threshold for distance limit
   */
  public static final float ZONE_THRESHOLD = (float) 2;
  
  /**
   * The odometer.
   */
  public static final Odometer odometer = Odometer.getOdometer();
  
  /**
   * The Ultrasonic Sensor
   */
  public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
}

package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Main.sleepFor;
import static ca.mcgill.ecse211.project.Resources.*;

/**
 * This class is used to drive the robot on the demo floor.
 */
public class CircleTurningDriver {


   
  /**
   * Rotate the robot continuously clockwise.
   */ 
  public static void rotateClockwise() {
    stopMotors();
    setAcceleration(ACCELERATION);
    setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.backward();
  }
  
  
  /**
   * Moves the robot straight for the given distance.
   * 
   * @param distance in feet (tile sizes), may be negative
   */
  public static void moveStraightForTile(double distance) {
    leftMotor.rotate(convertDistance(distance * TILE_SIZE), true);
    rightMotor.rotate(convertDistance(distance * TILE_SIZE), false);
  }
  
  /**
   * Moves the robot straight for the given distance.
   * 
   * @param distance in cm, may be negative
   */
  public static void moveStraightFor(double distance) {
    leftMotor.rotate(convertDistance(distance), true);
    rightMotor.rotate(convertDistance(distance), false);
  }
  
  /**
   * Turns the robot by a specified angle. Note that this method is different from
   * {@code Navigation.turnTo()}. For example, if the robot is facing 90 degrees, calling
   * {@code turnBy(90)} will make the robot turn to 180 degrees, but calling
   * {@code Navigation.turnTo(90)} should do nothing (since the robot is already at 90 degrees).
   * 
   * @param angle the angle by which to turn, in degrees
   */
  public static void turnBy(double angle) {
    leftMotor.rotate(convertAngle(angle), true);
    rightMotor.rotate(-convertAngle(angle), false);
  }
  
  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance the input distance
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
   * angle.
   * 
   * @param angle the input angle
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * BASE_WIDTH * angle / 360.0);
  }
  
  /**
   * Stops both motors.
   */
  public static void stopMotors() {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    leftMotor.stop();
    rightMotor.stop();
    leftMotor.setSpeed(Resources.ROTATE_SPEED);
    rightMotor.setSpeed(Resources.ROTATE_SPEED);
  }
  
  /**
   * Sets the speed of both motors to the same values.
   * 
   * @param speed the speed in degrees per second
   */
  public static void setSpeed(int speed) {
    setSpeeds(speed, speed);
  }
  
  /**
   * Sets the speed of both motors to different values.
   * 
   * @param leftSpeed the speed of the left motor in degrees per second
   * @param rightSpeed the speed of the right motor in degrees per second
   */
  public static void setSpeeds(int leftSpeed, int rightSpeed) {
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
  }
  
  /**
   * Sets the acceleration of both motors.
   * 
   * @param acceleration the acceleration in degrees per second squared
   */
  public static void setAcceleration(int acceleration) {
    leftMotor.setAcceleration(acceleration);
    rightMotor.setAcceleration(acceleration);
  }

  
}

package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab4.*;

public class LightLocalizer {

  // robot constants
  public final static int ROTATION_SPEED = 100;
  private final static double SENSOR_LENGTH = 14.3;
  private final static int FORWARD_SPEED = 80;
  private final static int ROTATE_SPEED = 100;
  
  private Odometer odometer;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;
  // Instantiate the EV3 ColorSensor
  private static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
  private float sample;

  private SensorMode Color;

  double[] linecount;
  
  private double currentx;
  private double currenty;	
  private double currentTheta;
  private double deltax;
  private double deltay;

  public LightLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {

	this.odometer = odometer;
	this.leftMotor = leftMotor;
	this.rightMotor = rightMotor;

	Color = lightSensor.getRedMode(); // set sensor to red light
	linecount = new double[4];
	}

	/**
	 * This method localizes the robot using the light sensor to precisely move to
	 * the right location
	 * @param sample
	 * @param count
	 */
  public void localize() {

	int count = 0;
	  leftMotor.setSpeed(ROTATION_SPEED);
	  rightMotor.setSpeed(ROTATION_SPEED);

	  // move to as close to origin as possible
	  moveCloseOrigin();

	  //Rotate and scan four lines, record angle respectively
	  //increase the count variable for 1, each time detection of line
	  while (count < 4) {

		leftMotor.forward();
		rightMotor.backward();

		sample = fetchSample();

		//detect lines, 0.37 determine by test and represents low density of reflection
		if (sample < 0.37) {
          linecount[count] = odometer.getXYT()[2];
		  Sound.beep();
		  count++;
			}
		}

		leftMotor.stop(true);
		rightMotor.stop();

	  double deltax, deltay, anglex, angley, deltathetaY;

	  // Get our location from origin using the calculated angles
	  angley = linecount[3] - linecount[1];
	  anglex = linecount[2] - linecount[0];

	  deltax = -SENSOR_LENGTH * Math.cos(Math.toRadians(angley / 2));
	  deltay = -SENSOR_LENGTH * Math.cos(Math.toRadians(anglex / 2));
	  //angle correction from tutorial
	  deltathetaY = (Math.PI / 2.0) - linecount[3] + Math.PI + (angley / 2.0);

	  // travel to origin from current calculated position
	  odometer.setXYT(deltax, deltay, odometer.getXYT()[2]);
	  this.travelTo(0.0, 0.0);

      leftMotor.setSpeed(ROTATION_SPEED / 2);
	  rightMotor.setSpeed(ROTATION_SPEED / 2);

	  // if we are not facing 0.0 then turn ourselves so that we are
	  if (odometer.getXYT()[2] <= 350 && odometer.getXYT()[2] >= 10.0) {
		Sound.beep();
		leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, -odometer.getXYT()[2]+ deltathetaY), true);
		rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, -odometer.getXYT()[2] + deltathetaY), false);
		}

		leftMotor.stop(true);
		rightMotor.stop();

	}

	/**
	 * This method moves the robot close the origin
	 * @param sample
	 */
  public void moveCloseOrigin() {
	  
    this.turnTo(Math.PI / 4);//call the turnTo method

	leftMotor.setSpeed(ROTATION_SPEED);
	rightMotor.setSpeed(ROTATION_SPEED);

	// get sample
	sample = fetchSample();

	// move forward past the origin until light sensor sees the line
	while (sample > 0.37) {
	  sample = fetchSample();
	  leftMotor.forward();
	  rightMotor.forward();

	  }
	  leftMotor.stop(true);
	  rightMotor.stop();
	  Sound.beep();

	  // Move backwards so our origin is close to origin
      leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, -SENSOR_LENGTH), true);
	  rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, -SENSOR_LENGTH), false);

	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
  private static int convertDistance(double radius, double distance) {
	return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of a angle to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @param angle
	 * @return
	 */
  private static int convertAngle(double radius, double width, double angle) {
	return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * This method gets the color value of the light sensor
	 * 
	 */
  private float fetchSample() {
	float[] colorValue = new float[Color.sampleSize()];
	Color.fetchSample(colorValue, 0);
		
	return colorValue[0];
	}

  /**
   * This method make the robot go to the point (x,y)
   * @param x
   * @param y
   * @param currentx
   * @param currenty
   * @param currentTheta
   * @param deltax
   * @param deltay
   * @param mTheta
   */
  public void travelTo(double x, double y) {

	currentx = odometer.getXYT()[0];
	currenty = odometer.getXYT()[1];

	deltax = x - currentx;
	deltay = y - currenty;
	
	// Calculate the angle to turn around
	currentTheta = (odometer.getXYT()[2]) * Math.PI / 180;
	double mTheta = Math.atan2(deltax, deltay) - currentTheta;
	
	double traveldistance = Math.hypot(deltax, deltay);

	// Turn to the correct angle towards the destination
	turnTo(mTheta);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, traveldistance), true);
		rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, traveldistance), false);

		// stop robot
		leftMotor.stop(true);
		rightMotor.stop(true);
	}
	/**
	 * This method make the robot turn the angle theta
     * @param theta
     * @param currentT
     * @param deltatheta
	 */
  public void turnTo(double theta) {
	//get current angle
	double currentT = odometer.getXYT()[2];
    // calculate angle should be turn
	double deltatheta = theta - currentT;
	//confirm the angle is between 0 and 360
    deltatheta = (deltatheta + 360) % 360;
    //make sure robot turn by the minimal angle
    if (Math.abs(deltatheta - 360) < deltatheta) {
      deltatheta -= 360;
	  }
	// let the robot turn by deltaTheta
	leftMotor.setSpeed(ROTATE_SPEED);
	rightMotor.setSpeed(ROTATE_SPEED);
	leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, deltatheta), true);
	rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, deltatheta), false);

  }
	
}
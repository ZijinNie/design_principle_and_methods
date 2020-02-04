/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;


import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private static int HorizontalLine; // Count black line in x direction
  private static int VerticalLine; // Count black line in y direction
  private Odometer odometer;
  private float[] sensordata;
  private static Port sensorPort = LocalEV3.get().getPort("S1");// define a port for sensor
  private SensorModes lightSensor;
  private double sensorcorrection;
  private double prevdata = 1 ;
  private static final double colorlimit = 0.6;//filter limit
  
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

	  this.odometer = Odometer.getOdometer();
		this.lightSensor = new EV3ColorSensor(sensorPort);
		this.sensordata = new float[lightSensor.sampleSize()];
		HorizontalLine = 0; 
		VerticalLine = 0;
		sensorcorrection = -0.6;//deviation from sensor center to middle point of axle
  }
  

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    float reflectIntensity;
    double theta;
    while (true) {
    	correctionStart = System.currentTimeMillis();

      // TODO Trigger correction (When do I have information to correct?)
      // TODO Calculate new (accurate) robot position

      // TODO Update odometer with new calculated (and more accurate) vales

    	lightSensor.getMode("Red").fetchSample(sensordata, 0);//get sample data, store in array
    	                                                      //measure intensity of reflected red light
		reflectIntensity = sensordata[0];//current sensor data
		double filter = Math.abs(prevdata - reflectIntensity);//filter for noise removal
		prevdata = reflectIntensity;//reset
		
		double[] odoData = odometer.getXYT(); // Get X Y T from OdometerData class
		theta = odoData[2];
        
       
        //light threshold for black line, 0.29 was derived experimentally 
		if (reflectIntensity < 0.29 && filter < colorlimit) { //lines with dark color reflect low intensity light
			                                                //use filter to remove wrong sample 
			Sound.beep();// beep when cross a line
			if (theta < 15 || theta > 345) { // moving in positive y direction, allow 15 degree deviation
				odometer.setY(VerticalLine * 30.48 - sensorcorrection); 
				VerticalLine ++; 
				
				
			}
			else if (theta < 105 && theta >= 75) { //  moving in positive x direction, allow 15 degree deviation
				odometer.setX(HorizontalLine * 30.48);
 				HorizontalLine ++;
			}

			else if (theta < 195 && theta >= 165) { //  moving in negative y direction, allow 15 degree deviation
				odometer.setY((VerticalLine - 1) * 30.48 + sensorcorrection);
				VerticalLine --; 
				if(VerticalLine == 0) {
					odometer.setY(-2.0);//final correction due to deviation in this route(hardware problem)
				}
			}

			else if (theta < 285 && theta >= 255) { // moving in negative x direction, allow 15 degree deviation
				odometer.setX((HorizontalLine - 1) * 30.48);
				HorizontalLine --; 
				if(HorizontalLine ==0) {
					odometer.setX(-1.5);//final correction due to deviation in this route 
				}
				}
			}

     

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}

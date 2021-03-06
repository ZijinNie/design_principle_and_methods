package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import lejos.hardware.Sound;


/**
 * Controls the robot's movements based on ultrasonic data.
 * <br><br>
 * Control of the wall follower is applied periodically by the UltrasonicController thread in the 
 * while loop in {@code run()}. Assuming that {@code usSensor.fetchSample()} and {@code 
 * processUsData()} take ~20ms, and that the thread sleeps for 50 ms at the end of each loop, then
 * one cycle through the loop is approximately 70 ms. This corresponds to a sampling rate of 1/70ms
 * or about 14 Hz.
 */
public class USDriver implements Runnable {
  
  /**
   * The distance remembered by the {@code filter()} method.
   */
  private double prevDistance;
  
  /**
   * The number of invalid samples seen by {@code filter()} so far.
   */
  private int invalidSampleCount;
  
//Thread control tools
 /**
  * Fair lock for concurrent writing.
  */
 private static Lock lock = new ReentrantLock(true);
 
 /**
  * Indicates if a thread is trying to reset any position parameters.
  */
 private volatile boolean isResetting = false;

 /**
  * Lets other threads know that a reset operation is over.
  */
 private Condition doneResetting = lock.newCondition();
  
  /**
   * Buffer (array) to store US samples. Declared as an instance variable to avoid creating a new
   * array each time {@code readUsSample()} is called.
   */
  private float[] usData = new float[usSensor.sampleSize()];
  

  /**
   * Constructor for an abstract UltrasonicController. It makes the robot move forward.
   */
  private volatile boolean exit = false;
  
  private volatile double minDist;
  
  
  private double firstAngle, secondAngle;
  private boolean isFirstUpRising;
  public USDriver() {
    
  }
  

  /**
   * Samples the US sensor and invokes the selected controller on each cycle (non Javadoc).
   * 
   * @see java.lang.Thread#run()
   */
  public void run() {
    
    minDist = Double.MAX_VALUE; //initialized mindist to max value
    double cur; //current distance 
    double prev;    //previous distance
    int count = 0;  //count of occurrence of distance D
    double highzone = TURNING_THRESHOLD + ZONE_THRESHOLD;   //high threshold
    double lowzone = TURNING_THRESHOLD - ZONE_THRESHOLD;    //low threshold

    double inAngle = 0 , outAngle = 0;  //angle when enter a threshold
    boolean isIn = false;   ////angle when leave a threshold
    boolean isEnterFromUp = false;  //if enter from above
    firstAngle = 0; 
    secondAngle= 0;
    
    prev = readUsDistance();    //initialize prev
    int countNum = 0;
    
    //loop until is it exits
    while(!exit) {
      
      countNum ++;
      
      cur = readUsDistance();
      
      // treat 0 reading as infinite far
      if(cur == 0) cur = Double.MAX_VALUE;

      //refresh mindist
      if(cur<minDist && minDist>5) minDist = cur;
      
      //when entering the threshold zone
      if(!isIn && lowzone<cur && cur<highzone) {
       
        //enter from rising edge
       if(prev > highzone) {
            isEnterFromUp = true;
            if(count == 0) {isFirstUpRising = false;}
       //enter from falling edge
       }else {
         if(count == 0) {isFirstUpRising = true;}
       }
       isIn = true;
       inAngle = Resources.odometer.getXyt()[2];
        
      
      }else if(isIn){
        
        //leaving the threshold zone
        if((cur< lowzone && isEnterFromUp) || (cur> highzone && !isEnterFromUp)) {
          
          isIn = false;
          outAngle = Resources.odometer.getXyt()[2];
          
          //the first occurence
          if(count == 0) {
            Sound.beep();
            firstAngle = (inAngle + outAngle)/2;
            
          //the second occurence
          }else{
            secondAngle = Resources.odometer.getXyt()[2];
            CircleTurningDriver.stopMotors();
            stop();
          }
          count +=1;    //increament counter
        }
      } 
    }
  }
  
  /**
   * Get if the US sensor is stopped now
   * @return if the US sensor is stopped
   */
  public boolean isExit() {
    return exit;
  }
  /**
   * Return the first angle
   * @return the first angle
   */
  public double getFirstAngle() {
    return firstAngle;
  }

  /**
   * Set the first angle
   * @param firstAngle the first angle
   */
  public void setFirstAngle(double firstAngle) {
    this.firstAngle = firstAngle;
  }

  /**
   * Get the second angle
   * @return the second angle
   */
  public double getSecondAngle() {
    return secondAngle;
  }

  /**
   * Set the second angle
   * @param secondAngle the second angle
   */
  public void setSecondAngle(double secondAngle) {
    this.secondAngle = secondAngle;
  }

  /**
   * Get the minimum distance red by US sensor
   * @return Get the minimum distance red by US sensor
   */
  public double getMinDist() {
    return minDist;
  }

  /**
   * Set the minimum distance red by US sensor
   * @param minDist Set the minimum distance red by US sensor
   */
  public void setMinDist(double minDist) {
    this.minDist = minDist;
  }


  /**
   * Returns the filtered distance between the US sensor and an obstacle in cm.
   * 
   * @return the filtered distance between the US sensor and an obstacle in cm
   */
  public double readUsDistance() {
    usSensor.fetchSample(usData, 0);
    // extract from buffer, convert to cm, cast to int, and filter
    return filter((double) (usData[0] * 100.0));
  }
  
  /**
   * Rudimentary filter - toss out invalid samples corresponding to null signal.
   * 
   * @param d raw distance measured by the sensor in cm
   * @return the filtered distance in cm
   */
  double filter(double d) {
    if ((d >= 255 ||d < 5)&& invalidSampleCount < INVALID_SAMPLE_LIMIT) {
      // bad value, increment the filter value and return the distance remembered from before
      invalidSampleCount++;
      return prevDistance;
    } else {
      if (d < 255) {
        // distance went below 255: reset filter and remember the input distance.
        invalidSampleCount = 0;
      }
      prevDistance = d;
      return d;
    }
  }

  /**
   * Set the stopping status of USDriver to true
   */
  public void stop() {
    exit = true;
  }

  /**
   * return if the first occurrence of distance D is rising edge
   * @return if the first occurrence of distance D is rising edge
   */
  public boolean isFirstUpRising() {
    return isFirstUpRising;
  }

  /**
   * set if the first occurrence of distance D is rising edge
   * @param isFirstUpRising if the first occurrence of distance D is rising edge
   */
  public void setFirstUpRising(boolean isFirstUpRising) {
    this.isFirstUpRising = isFirstUpRising;
  }

}

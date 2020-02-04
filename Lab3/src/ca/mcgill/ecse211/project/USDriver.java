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
  
  private volatile double curDist;
  
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
    
    minDist = Double.MAX_VALUE;
    double cur;
    double prev;
    int count = 0;
    double highzone = TURNING_THRESHOLD + ZONE_THRESHOLD;
    double lowzone = TURNING_THRESHOLD - ZONE_THRESHOLD;
//    double highzone = 75;
//    double lowzone = 65;
    double inAngle = 0 , outAngle = 0;
    boolean isIn = false;
    boolean isEnterFromUp = false;
    firstAngle = 0;
    secondAngle= 0;
    
    prev = readUsDistance();
    int countNum = 0;
    while(!exit) {
      countNum ++;
      cur = readUsDistance();
      if(cur == 0) cur = Double.MAX_VALUE;
      System.out.println(cur);
      
      if(cur<minDist && minDist>5) minDist = cur;
      
//      if(!isIn && lowzone<cur && cur<highzone) {
//        
//       if(prev > highzone) {
//            isEnterFromUp = true;
//            
//            if(count == 0) {isFirstUpRising = false;}
//       }else {
//         if(count == 0) {isFirstUpRising = true;}
//       }
//       isIn = true;
//       inAngle = Resources.odometer.getXyt()[2];
//        
//      }else if(isIn){
//        
//        if((cur< lowzone && isEnterFromUp) || (cur> highzone && !isEnterFromUp)) {
//          
//          isIn = false;
//          outAngle = Resources.odometer.getXyt()[2];
//          
//          if(count == 0) {
//            Sound.beep();
//            firstAngle = (inAngle + outAngle)/2;
//            
//          }else{
//            secondAngle = Resources.odometer.getXyt()[2];
//            CircleTurningDriver.stopMotors();
//            stop();
//          }
//          count +=1;
//        }
//      }
      if(countNum >20) {
        if((cur < TURNING_THRESHOLD && TURNING_THRESHOLD < prev) ||
            (cur > TURNING_THRESHOLD && TURNING_THRESHOLD > prev)) {
          
          
          if(count == 0) {
            if(prev < TURNING_THRESHOLD) {
              isFirstUpRising = true;
            }
            Sound.beep();
            firstAngle = Resources.odometer.getXyt()[2];
          
          }else{
            secondAngle = Resources.odometer.getXyt()[2];
            CircleTurningDriver.stopMotors();
            stop();
          }
          count +=1;
        }
      }
      prev = cur;
      
      try {
        Thread.sleep(50); // make the sensor sampling frequency be 20/s
      } catch (InterruptedException e) {
      }
      
    }
    
    
  }

  public boolean isExit() {
    return exit;
  }


  public void setExit(boolean exit) {
    this.exit = exit;
  }


  public double getFirstAngle() {
    return firstAngle;
  }


  public void setFirstAngle(double firstAngle) {
    this.firstAngle = firstAngle;
  }


  public double getSecondAngle() {
    return secondAngle;
  }


  public void setSecondAngle(double secondAngle) {
    this.secondAngle = secondAngle;
  }


  public double getMinDist() {
    return minDist;
  }


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

  
  public void stop() {
    exit = true;
  }



  public double getCurDist() {    
    double cur = 0;
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }
      cur = curDist;
        
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

      return cur;
  }


  public void setCurDist(double dist) {
    lock.lock();
    isResetting = true;
    try {
      this.curDist = dist;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }


  public boolean isFirstUpRising() {
    return isFirstUpRising;
  }


  public void setFirstUpRising(boolean isFirstUpRising) {
    this.isFirstUpRising = isFirstUpRising;
  }

}

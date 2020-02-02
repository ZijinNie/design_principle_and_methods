package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;


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
   * The current distance.
   */
  
  private int curDistance;
  /**
   * The distance remembered by the {@code filter()} method.
   */
  private int prevDistance;
  
  /**
   * The number of invalid samples seen by {@code filter()} so far.
   */
  private int invalidSampleCount;
  
  /**
   * Buffer (array) to store US samples. Declared as an instance variable to avoid creating a new
   * array each time {@code readUsSample()} is called.
   */
  private float[] usData = new float[usSensor.sampleSize()];
  
  /**
   * The size of the array storing US sensor data
   */
  private float[] usRecord = new float[USRECORD_SIZE];

  /**
   * Constructor for an abstract UltrasonicController. It makes the robot move forward.
   */
  private volatile boolean exit = false;
  
  public USDriver() {
    
  }
  

  /**
   * Samples the US sensor and invokes the selected controller on each cycle (non Javadoc).
   * 
   * @see java.lang.Thread#run()
   */
  public void run() {
    int i = 0;
    while(!exit || i<USRECORD_SIZE) {
      usRecord[i] = readUsDistance();
      i++;
      try {
        Thread.sleep(100); // make the sensor sampling frequency be 10/s
      } catch (InterruptedException e) {
      }
    }
  }

  /**
   * Returns the filtered distance between the US sensor and an obstacle in cm.
   * 
   * @return the filtered distance between the US sensor and an obstacle in cm
   */
  public int readUsDistance() {
    usSensor.fetchSample(usData, 0);
    // extract from buffer, convert to cm, cast to int, and filter
    return filter((int) (usData[0] * 100.0));
  }
  
  /**
   * Rudimentary filter - toss out invalid samples corresponding to null signal.
   * 
   * @param distance raw distance measured by the sensor in cm
   * @return the filtered distance in cm
   */
  int filter(int distance) {
    if (distance >= 255 && invalidSampleCount < INVALID_SAMPLE_LIMIT) {
      // bad value, increment the filter value and return the distance remembered from before
      invalidSampleCount++;
      return prevDistance;
    } else {
      if (distance < 255) {
        // distance went below 255: reset filter and remember the input distance.
        invalidSampleCount = 0;
      }
      prevDistance = distance;
      return distance;
    }
  }

  public int getCurDistance() {
    return curDistance;
  }

  public void setCurDistance(int curDistance) {
    this.curDistance = curDistance;
  }
  
  public void stop() {
    exit = true;
  }
  public float[] getUsRecord() {
    return usRecord;
  }


  public void setUsRecord(float[] usRecord) {
    this.usRecord = usRecord;
  }

}
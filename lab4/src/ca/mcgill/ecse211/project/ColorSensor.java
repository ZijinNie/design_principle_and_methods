package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.INVALID_SAMPLE_LIMIT;
import static ca.mcgill.ecse211.project.Resources.LIGHT_RADIUS;
import static ca.mcgill.ecse211.project.Resources.odometer;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.Sound;
public class ColorSensor implements Runnable {
	
	private float[] lightData = new float[Resources.colorSensor.sampleSize()];
	private static final double INTENSITY_THRESHOLD = 13;
	private int lowDensityCount;
	private double currentIntensity;
	
	  // Thread control tools
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
	public ColorSensor() {
		
	}
	
	
	public void run() {
		double currentIntensity;
		boolean preOnLine= false;
		 boolean isOnLine = false;
		double[] angles = new double[4];
		int count = 0;
		while(true) {
			currentIntensity = readIntensity();
			isOnLine = isOnLine();
			 System.out.println("Count: " + count);

//			System.out.println("Inte: "+currentIntensity);
			
			setCurrentIntensity(currentIntensity);
			
			if(!preOnLine && isOnLine) {
				 angles[count] = odometer.getXyt()[2];
				 if(count ==3) {
					 CircleTurningDriver.stopMotors();
					 break;
				 }
				 count ++;
			 }
			 preOnLine = isOnLine;
			
			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
		for (int i = 0; i < 4; i++) {
			System.out.println(angles[i]);
		}
		double deltax = LIGHT_RADIUS * Math.cos( (angles[3] - angles[1])/2 * 0.0174533);
		 
		 double deltay = LIGHT_RADIUS * Math.cos( (angles[2] - angles[0])/2 * 0.0174533);

		 double thetay = angles[3] - angles[1];
		 
		 CircleTurningDriver.turnBy(90 - thetay/2);
		 System.out.println("x" + deltax + "y" + deltay + " theta " + (90 - thetay/2));
		 CircleTurningDriver.moveStraightFor(deltay+1);
		 CircleTurningDriver.turnBy(90);
		 CircleTurningDriver.moveStraightFor(deltax);
		 CircleTurningDriver.turnBy(-93);
		 odometer.setXyt(Resources.TILE_SIZE,Resources.TILE_SIZE , 0);
	}
	
	public double readIntensity() {
		Resources.colorSensor.fetchSample(lightData, 0);
		return lightData[0];
	}
	
	public double getCurrentIntensity() {
		double result = 0;    
		    lock.lock();
		    try {
		      while (isResetting) { // If a reset operation is being executed, wait until it is over.
		        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
		      }

		      result = currentIntensity;
		    } catch (InterruptedException e) {
		      e.printStackTrace();
		    } finally {
		      lock.unlock();
		    }

		    return result;
	}	
	
	public void setCurrentIntensity(double intensity) {
		lock.lock();
	    try {
	      while (isResetting) { // If a reset operation is being executed, wait until it is over.
	        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
	      }

	      currentIntensity = intensity;
	    } catch (InterruptedException e) {
	      e.printStackTrace();
	    } finally {
	      lock.unlock();
	    }
	}
	
	public boolean isOnLine() {
		currentIntensity = getCurrentIntensity();
		if ((int)currentIntensity ==  INTENSITY_THRESHOLD && lowDensityCount < Resources.LOW_DENSITY_LIMIT) {
		      // low intensity value, increment the filter value and return the distance remembered from before
		      lowDensityCount++;
		      return false;
		} else if((int)currentIntensity ==  INTENSITY_THRESHOLD && lowDensityCount >= Resources.LOW_DENSITY_LIMIT){
			Sound.beep();
		     return true;
		}
		else lowDensityCount = 0;
		return false;
//		if(currentIntensity == INTENSITY_THRESHOLD) Sound.beep();
//		return (currentIntensity == INTENSITY_THRESHOLD);
	}
}

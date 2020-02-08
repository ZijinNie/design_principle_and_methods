package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.INVALID_SAMPLE_LIMIT;

import ca.mcgill.ecse211.project.Resources.*;
public class ColorSensor implements Runnable {
	
	private float[] lightData = new float[Resources.colorSensor.sampleSize()];
	private static final double INTENSITY_THRESHOLD = 50;
	private int lowDensityCount;
	private double currentIntensity;
	public ColorSensor() {
		
	}
	
	
	public void run() {
		
	}
	
	public double readIntensity() {
		Resources.colorSensor.fetchSample(lightData, 0);
		return lightData[0];
	}
	
	public boolean isOnLine() {
		currentIntensity = getCurrentIntensity();
		if (currentIntensity< INTENSITY_THRESHOLD && lowDensityCount < Resources.LOW_DENSITY_LIMIT) {
		      // bad value, increment the filter value and return the distance remembered from before
		      lowDensityCount++;
		      return false;
		} else if(currentIntensity< INTENSITY_THRESHOLD && lowDensityCount >= Resources.LOW_DENSITY_LIMIT){
		     return true;
		}
		else lowDensityCount = 0;
		return false;
	}
}

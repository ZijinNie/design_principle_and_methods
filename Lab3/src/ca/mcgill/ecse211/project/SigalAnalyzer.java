package ca.mcgill.ecse211.project;

import java.util.Arrays;

public class SigalAnalyzer {
  
  private int usSamplingRate;
  
  private float[] rawRecord;
  
  private float[] truncRecord;
  
  public SigalAnalyzer(int usSamplingRate, float[] record){
    this.usSamplingRate = usSamplingRate;
    this.rawRecord = record;
  }
  
  public int getInitialAngle() {
    cutRecord();
    truncRecord = smoothArray(truncRecord);
    float[] dev = derivative(truncRecord);
    int convexPlaceAngle = getConvexPlaceAngle(dev);
    return (convexPlaceAngle+ 135)%360;
  }
  
  private void cutRecord() {
    int zeroNum = 0;
    int num;
    for(num = 0; num<rawRecord.length; num++) {
      if (rawRecord[num] == 0) zeroNum ++;
      if (zeroNum >= 5) break;
    }
    float[] array = Arrays.copyOf(rawRecord, num-4);
    truncRecord = array;
  }
  
//  private void getStandardDev(float[] array) {
//    float mean = getMean(array);
//    float[] dup = 
//    
//  }
  
  public float[] smoothArray(float[] input) {
    float[] dup = Arrays.copyOf(input, input.length);
    int index = 0;
    for(index =0; index< input.length; index++) {
      if(index != 0 && index != input.length-1) {
        input[index] = (dup[index-1]+dup[index]+dup[index+1])/3;
      }
    }
    return input;
  }
  
//  public float[] derivative(float[] input) {
//    
//  }
  
  public int getConvexPlaceAngle(float[] input) {
    int minIndex;
    float minDer = Float.MAX_VALUE;
    for(int i = 0; i< input.length; i++) {
//      if(Math.abs(input[i])< minDer) {
//        minDer = Math.abs(input[i]);
//        minIndex = i;
//      }
        if(Math.abs(input[i]) < 5) {
          if(input[i-2]< input[i] && input[i+2]<input[i]) {
            return (int) (i/input.length)*360;
          }
        }
    }
    return 0;
  }
  
  
  public float getMean(float[] array) {
    float sum = 0;
    for(float i: array) {
      sum += i;
    }
    return sum/array.length;
  }
  
}

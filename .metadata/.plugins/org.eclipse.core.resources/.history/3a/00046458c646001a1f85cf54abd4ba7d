package ca.mcgill.ecse211.project;

import java.util.Arrays;

public class SignalAnalyzer {

  private int usSamplingRate;

  private float[] rawRecord;

  private float[] truncRecord;

  public SignalAnalyzer(int usSamplingRate, float[] record){
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
    System.out.println(Arrays.toString(truncRecord));
  }

//  private void getStandardDev(float[] array) {
//    float mean = getMean(array);
//    float[] dup =
//
//  }

public float[] smoothArray(float[] input) {
  System.out.println("Smoothed array: ");
  float[] dup = Arrays.copyOf(input, input.length);
  int index = 0;
  int length = input.length;
  int smoothGroupNumber = 10;
  int left = smoothGroupNumber/2;
  int right = smoothGroupNumber/2;
  int sum = 0;
  for(index =0; index< input.length; index++) {
    for(int i = 1; i <= left; i++) {
      sum += dup[(-i+index+length)%length]+ dup[(i+index+length)%length];
    }
    sum += dup[index];
    input[index] = sum/(2*left+1);
    sum = 0;
  }

  System.out.println(Arrays.toString(input));

  return input;
}

  public float[] derivative (float[] input) {
    float[] dup = Arrays.copyOf(input, input.length);
    int i = 0;
    System.out.println("Derivative: ");
    while(i<= (input.length-1)) {
      if(i != 0) {
        input[i] = (dup[i]-dup[i-1])*10;
      }
      i++;
    }
    System.out.println(Arrays.toString(input));
    return input;
  }

  public int getConvexPlaceAngle(float[] input) {
    int minIndex;
    float minDer = Float.MAX_VALUE;
    for(int i = 0; i< input.length; i++) {
//      if(Math.abs(input[i])< minDer) {
//        minDer = Math.abs(input[i]);
//        minIndex = i;
//      }
//        if(Math.abs(input[i]) < 5) {
//          if(input[i-2]< input[i] && input[i+2]<input[i]) {
//            return (int) (i/input.length)*360;
//          }
//        }
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

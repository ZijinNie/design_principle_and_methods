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
  
  public float getMean(float[] array) {
    float sum = 0;
    for(float i: array) {
      sum += i;
    }
    return sum/array.length;
  }
  
}
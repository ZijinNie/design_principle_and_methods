package ca.mcgill.ecse211.lab5;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.lab5.LightLocalizer;
import ca.mcgill.ecse211.lab5.Odometer;
import ca.mcgill.ecse211.lab5.OdometerExceptions;
import ca.mcgill.ecse211.lab5.UltrasonicLocalizer;

/* This is our main class for Lab5
 * It implements color detection and searching from pressing button
 * @author Team5
 */
public class Lab5 {
  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
  new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
  new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final Port usPort = LocalEV3.get().getPort("S2"); //port for the Ultrasonic Sensor
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.15;
  public static final double TRACK = 14.2 ;
  public static final double tileSize = 30.48;
  
  public static final int[] LL = {2,2};//start point
  public static final int[] UR = {5,5};//end point
  public static final int SC  = 0;//starting corner
  public static final String[] colors = {"red","green","blue","yellow"};
  public static int targetColor = 2;//target can color
  
  //colorDetector sensor related objects
  static EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
   
  // Ultrasonic sensor related objects
  private static SampleProvider sampleProviderUS;
  static EV3UltrasonicSensor usSensor;
  static float[] usValues;
	  
  LightLocalizer ll;
	  
  public static Odometer odometer;
  public static void main(String[] args) throws InterruptedException,OdometerExceptions {
		// TODO Auto-generated method stub
	int buttonChoice;
	do {
	  // clear the display
	  lcd.clear();

	  // ask the user use color detection or field test
	  lcd.drawString("< Left | Right >", 0, 0);
	  lcd.drawString("       |        ", 0, 1);
	  lcd.drawString("color  | field  ", 0, 2);
	  lcd.drawString("detect | search   ", 0, 3);
	  lcd.drawString("       |        ", 0, 4);

	  buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
	} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
	//choose the condition that use color detection or field search
    if (buttonChoice == Button.ID_LEFT) { //Start color detection
      int color;
      lcd.clear();//clear screen
      ColorClassification colorDetector = new ColorClassification(colorSensor);
      usSensor = new EV3UltrasonicSensor(usPort);
      sampleProviderUS = usSensor.getMode("Distance");
      usValues = new float[sampleProviderUS.sampleSize()];
      for (int i = 0 ; i < 5;i++) {//set loop for five required cans
    	  lcd.clear();
    
    	sampleProviderUS.fetchSample(usValues, 0);//make sure can close to robot
    	while(usValues[0]*100 > 10) {
    	  sampleProviderUS.fetchSample(usValues, 0);
    	}
    	  color = colorDetector.findColor();
    	  lcd.drawString("object detected", 0, 1);
    	  if(color<0) {
    		  lcd.drawString("Failure", 0, 2);
    	  }
    	  else {
    		  lcd.drawString(colors[color], 0, 2);
    	  }  
    	  if (Button.waitForAnyPress() == Button.ID_ESCAPE){
    	    System.exit(0);
    	  }
    	  else {
    		Thread.sleep(1000);  
    	  }
      }
      
    } else if (buttonChoice == Button.ID_RIGHT) { //field search start
      Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);//Completed implementation
      
      //setup ultrasonic sensor
      usSensor = new EV3UltrasonicSensor(usPort);
      sampleProviderUS = usSensor.getMode("Distance");
      usValues = new float[sampleProviderUS.sampleSize()];
  	  //construct  a UltrasonicLocalizer	
      UltrasonicLocalizer usl = new UltrasonicLocalizer(sampleProviderUS, usValues, leftMotor, rightMotor, SC);
      
      //setup threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      //Do localization with ultrasonic sensor    	
      usl.doLocalization();
      usSensor.close();
      //set up the light 
      LightLocalizer ll = new LightLocalizer(leftMotor, rightMotor,SC);
      //Do localization with light sensor
      ll.doLocalization();
      usSensor = new EV3UltrasonicSensor(usPort);
      sampleProviderUS = usSensor.getMode("Distance");
      usValues = new float[sampleProviderUS.sampleSize()];
      ColorClassification colorDetector = new ColorClassification(colorSensor);
      Navigation.odometer = odometer;
      Navigation navigation = new Navigation(leftMotor,rightMotor,targetColor,UR[0],LL[0],UR[1],LL[1],colorDetector);
      SensorPoller usPoller = new SensorPoller(usSensor,usValues,leftMotor,rightMotor,navigation);
      Thread usThread = new Thread(usPoller);//set and start thread
      usThread.start();
      Thread navigate = new Thread(navigation);//set and start thread
      navigate.start();
      while (Button.waitForAnyPress() != Button.ID_ESCAPE);
      System.exit(0);
    }
  }

}

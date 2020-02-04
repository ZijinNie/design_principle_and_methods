package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class is the main navigation class, it will make robot follow the path chosen 
 * and avoid the obstacles it detects		
 * @author Group 6										
 */
public class Navigation implements Runnable {
	
  public static EV3LargeRegulatedMotor rightMotor;
  public static EV3LargeRegulatedMotor leftMotor;
  
  public static final EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
  // creating the sensor
  private static final Port usPort = LocalEV3.get().getPort("S1");
  private static SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
  private static SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
  // this instance
  private static float[] usData = new float[usDistance.sampleSize()];
	
  // instance of odometer
  private static Odometer odo;
  private static double currentX;
  private static double currentY;
  private static double currentT;
  private static double deltatheta;
	
  //holds whether or not the robot is traveling
  private static boolean travelStatus = false;
  
  /** Speeds for the motors
   * @param FWDSPEED Forward speed
   * @param ROTATE_SPEED Turning speed
   * @param SENSOR_SPEED speed for sensor's motor, control sensor rotation speed
   */
  private static final int FWDSPEED = 200;
  private static final int ROTATE_SPEED = 80;
  private static final int SENSOR_SPEED = 40;
  /** initializing the 4 possible routes from given material
   * Using 2D array
   * @value Map1
   * @param Map2
   * @param Map3
   * @param Map4
  */
  private static final int[][] Map1 = new int[][] { { 0, 2 }, { 1, 1 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
  private static final int[][] Map2 = new int[][] { { 1, 1 }, { 0, 2 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
  private static final int[][] Map3 = new int[][] { { 1, 0 }, { 2, 1 }, { 2, 2 }, { 0, 2 }, { 1, 1 } };
  private static final int[][] Map4 = new int[][] { { 0, 1 }, { 1, 2 }, { 1, 0 }, { 2, 1 }, { 2, 2 } };
	
  /** initializing variable used in this main class
   * @param disatance
   * @param SAFE_DISTANCE  distance for checking too close
   * @param theta
   * @param AVOID_DISTANCE distance for whole avoidance process
   */
  private static int distance;
  private static final int SAFE_DISTANCE=14;
  private static double theta;
  private static final int AVOID_DISTANCE = 65;
  /**
  * Constructor, initialize motor
  * @param Tile_Size
  * @param leftMotor1
  * @param rightMotor1
  */
  public Navigation(EV3LargeRegulatedMotor leftMotor1, EV3LargeRegulatedMotor rightMotor1) {
	rightMotor = rightMotor1;
	leftMotor = leftMotor1;
	rightMotor.stop();
	leftMotor.stop();
	}
	
  /**
  * Run method create a loop to enable robot visit five points one by one
  * @return void
  */
  public void run() {
	  int[][] waypoints = Map2;
	    for (int i = 0; i < 5; i++) {

		try {
		  TravelTo(Lab3.Tile_Size * waypoints[i][0], Lab3.Tile_Size * waypoints[i][1]);
				
		  } catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		  } catch (InterruptedException e) {
		    // TODO Auto-generated catch block
			e.printStackTrace();
		 }
  }
}
  /**
  * This method make the robot go to the point (x,y)
  * @param x : target position  
  * @param y : target position
  * @param currentX 
  * @param currentY
  * @throws OdometerExceptions
  * @throws InterruptedException
  * @return void
  */
  public static void TravelTo(double x, double y) throws OdometerExceptions, InterruptedException {
    theta = 0.0;
    // get current position for next process(calculate angle)
	odo = Lab3.odometer;
	currentX = odo.getXYT()[0];
	currentY = odo.getXYT()[1];
		
	while (!withinerror(currentX, currentY, x, y)) {
			
	  travelStatus = true;
	  //updating our x and y
	  currentX = odo.getXYT()[0];
	  currentY = odo.getXYT()[1];
	  // calculating the angle that robot should facing
	  if (currentX == x) {
        if (currentY > y) {
        theta = Math.PI;
        } else if (currentY < y) {
		  theta = 0;
		  }

		} else if (currentY == y) {
		    if (currentX > x) {
			theta = -Math.PI/2;
			} else if (currentX < x) {
			  theta = Math.PI/2;
			}
			} else {
			  theta = Math.atan((currentX - x) / (currentY - y));
		   	  if (currentY > y) {
	    	  theta += Math.PI;
			  }
			}
		  // turn to the angle calculated
		  turnTo(theta * 180 / Math.PI);
	      // move forward by required distance
		  // distance is calculated using Pythagorean Theorem
	   	  leftMotor.setSpeed(FWDSPEED);
		  rightMotor.setSpeed(FWDSPEED);
		  double dist = Math.hypot((currentX - x) , (currentY - y));
		  leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, dist), true);
	      rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, dist), true);
		  // checking for obstacle 
	      while (!withinerror(currentX, currentY, x, y)) {
		  // update current X and Y
		    currentX = odo.getXYT()[0];
			currentY = odo.getXYT()[1];
			// get distance data from the sensor
			usSensor.fetchSample(usData, 0); 
			distance = (int) (usData[0] * 100.0); 
			// if obstacle detected, call the Avoid and break from this loop to go back the outter loop
			if (distance < SAFE_DISTANCE) {
			  if (predictPath() == true){
		  		  RightAvoid();
				  }
				  else if 
				  (predictPath() == false){
				  leftAvoid();
				  }
				
			  break;
			}

			try {
				Thread.sleep(50);
			} catch (Exception e) {
			}
		}
			
		}
		// arrived to its destination
		travelStatus = false;
		
	}
  /**
  * This method make the robot turn the angle theta, as required by lab document
  * @param theta
  */
  public static void turnTo(double theta) {
    currentT = odo.getXYT()[2];
    // calculate angle should be turn
	deltatheta = theta - currentT;
	//confirm the angle is between 0 and 360
    deltatheta = (deltatheta + 360) % 360;
	//make sure robot turn by the minimal angle
	  if (Math.abs(deltatheta - 360) < deltatheta) {
	    deltatheta -= 360;
	  }
	// let the robot turn by deltaTheta
	leftMotor.setSpeed(ROTATE_SPEED);
	rightMotor.setSpeed(ROTATE_SPEED);
	leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, deltatheta), true);
	rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, deltatheta), false);

	}
  /**
  * This method returns whether or not the robot is moving
  * @return	status 
  */
  public static boolean isNavigating() {
	return travelStatus;
  }
//This helper method convert the required distance for moving
  private static int convertDistance(double radius, double distance) {
	return (int) ((180.0 * distance) / (Math.PI * radius));
  }
//This helper method convert the required angle for turning
  private static int convertAngle(double radius, double width, double angle) {
	return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
	
  /**
   * This method return wether or not the robot reached to desired position
   * @param Cx :currentX
   * @param Cy : currentY
   * @param x :target x
   * @param y : traget y
   * @return boolean 
   */
  private static boolean withinerror(double Cx, double Cy, double x, double y) {
    double error = Math.sqrt(Math.pow((Cx - x), 2) + Math.pow((Cy - y), 2));

	return error < 2.0;
	}
  
  /**
  * This method make the robot avoid the obstacle detected from right side of obstacle
  * @return void
  */
  public static void RightAvoid() {
	//make the robot turn 90 degrees to the right
	leftMotor.setSpeed(ROTATE_SPEED);
	rightMotor.setSpeed(ROTATE_SPEED);
	leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), true);
	rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), false);
	// turn the sensor to face the obstacle
	sensorMotor.setSpeed(SENSOR_SPEED);
	sensorMotor.rotate(80, false);
	// Avoidance begin, using wall following principle(bang-bang)
		
	//recover the sensor to original position after turning
    double xAvoid=0, yAvoid=0, prevX, prevY;		
      prevX = odo.getXYT()[0];
  	  prevY = odo.getXYT()[1];
  	  while(true){			
	  currentX = odo.getXYT()[0];
	  currentY = odo.getXYT()[1];
	  xAvoid += Math.abs(currentX - prevX);
	  yAvoid += Math.abs(currentY - prevY);
	  prevX = currentX;
	  prevY = currentY;
	    if (Math.hypot(xAvoid, yAvoid) > AVOID_DISTANCE){
		sensorMotor.setSpeed(SENSOR_SPEED);
		sensorMotor.rotate(-80, false);  
		return;
		}		 
	//Wallfollower implemented, aim to avoid obstacle perfectly	
    int currentDistance = 0;
	  while (currentDistance < SAFE_DISTANCE) {
	  	usSensor.fetchSample(usData, 0); 
    	currentDistance = (int) (usData[0] * 100.0);
    	int error = currentDistance - SAFE_DISTANCE;
	   	if(Math.abs(error) <= 3) { //band-width
	      leftMotor.setSpeed(ROTATE_SPEED);
	 	  rightMotor.setSpeed(ROTATE_SPEED);
	  	  leftMotor.forward();
	      rightMotor.forward();
	    }
	    else if(error < 0){	//too close control
	  	  leftMotor.setSpeed(ROTATE_SPEED);	
	  	  rightMotor.setSpeed(ROTATE_SPEED);		
	      leftMotor.forward();
	      rightMotor.backward();                 
	    	}
	    else{ //turning
	      leftMotor.setSpeed(ROTATE_SPEED - 40);
	      rightMotor.setSpeed(ROTATE_SPEED );
	      leftMotor.forward();
	      rightMotor.forward();
	    	}
    }
  }
}

  public static void leftAvoid() {
    //make the robot turn 90 degrees to the left
    leftMotor.setSpeed(ROTATE_SPEED);
	rightMotor.setSpeed(ROTATE_SPEED);
	leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), true);
	rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), false);
	
	// turn the sensor to face the obstacle
	sensorMotor.setSpeed(SENSOR_SPEED);
	sensorMotor.rotate(-80, false);  	
	//Avoidance begin, using wall following principle(bang-bang)
	
	//recover the sensor to original position after turning
	double xAvoid=0, yAvoid=0, prevX, prevY;	 
	prevX = odo.getXYT()[0];
	prevY = odo.getXYT()[1];
	while(true){	
	  currentX = odo.getXYT()[0];
      currentY = odo.getXYT()[1];
	  xAvoid += Math.abs(currentX - prevX);
	  yAvoid += Math.abs(currentY - prevY);
	  prevX = currentX;
      prevY = currentY;
	  if(Math.hypot(xAvoid, yAvoid) > AVOID_DISTANCE){
		sensorMotor.setSpeed(SENSOR_SPEED);
		sensorMotor.rotate(80, false);  
	    return;
	  }	
	  //Wallfollower
	  int currentDistance = 0;

	  while (currentDistance < SAFE_DISTANCE) {
		usSensor.fetchSample(usData, 0); // fetch data
		currentDistance = (int) (usData[0] * 100.0);
		int error = currentDistance - SAFE_DISTANCE;
		if(Math.abs(error) <= 3 ){ //band-width
		  leftMotor.setSpeed(ROTATE_SPEED);
		  rightMotor.setSpeed(ROTATE_SPEED);
		  leftMotor.forward();
		  rightMotor.forward();
		}
		else if(error < 0){	//too close control
   		  leftMotor.setSpeed(ROTATE_SPEED);	
		  rightMotor.setSpeed(ROTATE_SPEED);		
		  leftMotor.backward();
		  rightMotor.forward();               
		}
		else{ //turning
		  leftMotor.setSpeed(ROTATE_SPEED); 
		  rightMotor.setSpeed(ROTATE_SPEED -40);
		  leftMotor.forward();
		  rightMotor.forward();
		}
    }
  }
}

  /**
   * This method determine whether the robot should turn
   * left or right to avoid the obstacle
   * due to possible fall out of panel
   * @param currentX
   * @param currentY
   * @param currentT
   */
	public static boolean predictPath() {

	currentX = odo.getXYT()[0];
	currentY = odo.getXYT()[1];
	currentT = odo.getXYT()[2];
		
	if (currentT > 350 || currentT <= 10) {//positive y direction
	  if (currentX < Lab3.Tile_Size -1) {
	    return true;           // true represents right avoid and false represents left avoid
	  } 
	  else {
		return false;
			}
		} 
	  else if(currentT >= 80 && currentT < 100){//positive x direction
		if (currentY < Lab3.Tile_Size) {
		  return false;
		} 
		else {
		  return true;
		}
		}
		else if(currentT > 170 && currentT < 190){//negative y direction
		  if (currentX < Lab3.Tile_Size) {
   		  return false;
		} 
		else {
   		  return true;
		}
		}
		else if(currentT > 260 && currentT < 280){//negative y direction
		  if (currentY <= Lab3.Tile_Size) {
		  return true;
		  } 
	      else {
		  return false;
	      }
		  }
		else { 
		  return true;
			}
	}
}

package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.Odometer;
import ca.mcgill.ecse211.lab5.OdometerExceptions;
import lejos.hardware.Sound;

import lejos.hardware.motor.EV3LargeRegulatedMotor;


/**
 * This class is our navigation class used to determine our path for searching cans
 * And implement avoidance during searching
 * This navigation thread is called in main class and start when localization is done
 * @param colors //array to store four colors
 * @param targetColour //indicate our searching target can color
 * @param planningType //indicate the route method calling
 * @param LLx LLy URx URy // position paramters for search area
 * @param obstacle //boolean value for controlling and indicating status of avoid
 * @param tileSize ROTATE_SPEED LeftRadius rightRadius track FORWARD_SPEED// robot-related parameters
 * @param safeDistance //the lower threshold for us sensor to react and make robot stop
 * @param travelling //boolean indicating travelling to point
 * @param searching //boolean indicating searching status
 * @param found //boolean indicating can detected
 * @param search //boolean indicating can detected when doing whole line detection
 * @param avoid //boolean indicating avoid process, true means avoid strat
 * @param returnline //boolean indicating go back to middle point of tile border status
 * @author Team 5
 * 
 */
public class Navigation extends Thread{
	
  private static final int FORWARD_SPEED = 200;
  public static final String[] colors = {"red","green","blue","yellow"};
  private int targetColour;
  private char planningType;
  private int LLx;
  private boolean firstSide;
  private int URx;
  private int LLy;
  private int URy;
  public boolean obstacle;
  public static final double tileSize = 30.48;
  private static final int ROTATE_SPEED = 160;
  public static Odometer odometer;
  private static EV3LargeRegulatedMotor leftMotor ;
  private static EV3LargeRegulatedMotor rightMotor;
  private ColorClassification colorDetector;
  static final double leftRadius = 2.15;
  static final double rightRadius = 2.15;
  static final double track = 14.2;
  private static final int safeDistance= 5;
  private boolean travelling;
  private boolean searching;
  private boolean found;
  static double x;
  static double y;
  public boolean search;
  private static boolean avoid=false;
  private static boolean returnline = true;
  
  
  /**
   * This is the navigation class constructor. parameter already explained
   * @param LLX
   * @param LLy
   * @param URx
   * @param URy
   * @param searching
   * @param targetColour
   * @param obstacle
   * @param found
   * @param colorDetector
   * @param search
   * @param travelling
   * @throws OdometerExceptions
   */
  public Navigation(EV3LargeRegulatedMotor leftmotor,EV3LargeRegulatedMotor rightmotor,int targetColour,int URx,int LLx,int URy,int LLy,ColorClassification colorDetector) throws OdometerExceptions {
	this.LLx = LLx;
	this.LLy = LLy;
	this.URx = URx;
	this.URy = URy;
	this.searching = false;
	this.targetColour = targetColour;
	this.obstacle = false;
	this.found = false;
	this.colorDetector = colorDetector;
	this.search = false;
	this.travelling = false;
	leftMotor = leftmotor;
    rightMotor = rightmotor;
    x = 0;
    y = 0;
    
  }

  /**
   * This method is meant to drive the robot to desired target point
   * 
   * @param leftMotor
   * @param rightMotor
   * @param leftRadius
   * @param rightRadius
   * @param width
   */
  public void travelTo(double x,double y ) throws OdometerExceptions{
	  if (!searching) {
	    	double distance1 = Math.hypot(odometer.getXYT()[0]-x, odometer.getXYT()[1]-y);
	    	if(distance1 < 3) {
	    		return;
	    	}
	    } else {
	    		if (search && !returnline) {
	    				if (closeSearchEnd()) {
	    					returnline = true;
	    					return;
	    			}
	    		}
	    }
	double theta, dX, dY, distance;
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.stop();
      motor.setAcceleration(3000);
    }

    // Sleep for 2 seconds
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }

    //Calculation from current position and next position
    dX = x - odometer.getXYT()[0];
    dY = y - odometer.getXYT()[1];
    distance = Math.sqrt(dX*dX + dY*dY);
    theta = Math.toDegrees(Math.atan(dX/dY));
    
    if(dX < 0 && dY < 0) {
    	theta += 180;
    }
    if(dX > 0 && dY < 0) {
    	theta += 180;
    }
    if(dX < 0 && dY > 0) {
    	theta += 360;
    }
    turnTo(theta);
    this.travelling = true;
   if(search) {//if can exists in this whole line, slow down forward speed to reduce risk
    leftMotor.setSpeed((int)(FORWARD_SPEED*0.6));
    rightMotor.setSpeed((int)(FORWARD_SPEED*0.6));
   }else {
	   leftMotor.setSpeed(FORWARD_SPEED);
	    rightMotor.setSpeed(FORWARD_SPEED); 
   }
    leftMotor.rotate(convertDistance(leftRadius, distance), true);
    rightMotor.rotate(convertDistance(rightRadius, distance), true);
    while(true) {
    	if(obstacle) {//call avoid
    		avoid(SensorPoller.realDistance);
    		if(searching) {
    		if(found) {
    			break;
    		}
    		}
    		if (!searching || search) {//no can, travel to next point
    			travelTo(x,y);   
    		}
    		break;
    	}
    if (!searching) {//checking error
    	double distance1 = Math.hypot(odometer.getXYT()[0]-x, odometer.getXYT()[1]-y);
    	if(distance1 < 3) {
    		break;
    	}
    } else {
    				if ( search && !returnline && closeSearchEnd()) {
    					break;
    				
    			
    		}else {
    			double distance1 = Math.hypot(odometer.getXYT()[0]-x, odometer.getXYT()[1]-y);
    	    	if(distance1 > 3) {
    	    		distance1 = Math.hypot(odometer.getXYT()[0]-x, odometer.getXYT()[1]-y);
    	    	}else {
    	    		break;
    	    	}
    		}
    }
    }
    travelling = false;
   
  }
  /*This method implement start color detection and decide the direction of avoid 
   * after color detection using predict path method that implemented before in lab3
   * @param color //indicate which color is found
   */
  public  void avoid(double distance) throws OdometerExceptions{
	int color = 4;
    leftMotor.stop(true);
	rightMotor.stop();
	if(searching) {
		carStop();
	      leftMotor.setSpeed(FORWARD_SPEED/2);
	      rightMotor.setSpeed(FORWARD_SPEED/2);
		color = colorDetector.findColor();
	  if (color == targetColour) {//target can found
	    Sound.beep();
	    this.found = true;
	    return;
	  }else {
	    Sound.twoBeeps();//not target can
	  }
	  leftMotor.setSpeed(FORWARD_SPEED/2);
      rightMotor.setSpeed(FORWARD_SPEED/2);
	}
	travelling = false;
	if (distance < safeDistance) {
	  if (predictPath() == 1){
	    RightAvoid(color);
	  }
	  else if (predictPath() == 0){
	    leftAvoid(color);
	  }
	  if (found) {
			return;
		}
	}
	this.obstacle = false;
	   
  }
  /*This method implement travelling status used in SensorPoller
   *@param travelling
   */
   boolean  isNavigating(){
	  return this.travelling;
  }
   /*This method decides whether robot should avoid in left or right after detection
    * 
    */
  public int predictPath() {
	

		double currx = odometer.getXYT()[0];
		double curry = odometer.getXYT()[1];
		double currTheta = odometer.getXYT()[2];
		
		if (currTheta > 340 || currTheta <= 20) {//going up
			if (currx < (LLx+0.5)*tileSize) {
				return 1;
			           // 1 represents right dodge and 0 represents left dodge
			} 
			else if (currx > (URx-0.5)*tileSize){
				return 0;
				
			}
		} 
		else if(currTheta >= 70 && currTheta < 110){//going right
			if (curry < (LLy+0.5)*tileSize) {
				return 0;
				
			} 
			else if (curry > (URy-0.5)*tileSize) {
				return 1;
				
			}
		}
		else if(currTheta > 160 && currTheta < 200){//going down
			if (currx < (LLx+0.5)*tileSize) {
				return 0;
			} 
			else if (currx > (URx-0.5)*tileSize) {
				return 1;
			
			}
		}
		else if(currTheta > 250 && currTheta < 290){//going left
			if (curry <= (LLy+0.5)*tileSize ) {
				return 1;
			} 
			else if (curry > (URy-0.5)*tileSize) {
				return 0;
			}
		}
			if (this.planningType == 'P') {
				if (this.firstSide) {
					
					return 0;
				}else {
					System.out.println("10");
					return 1;
				}
			}else {
				return 0;
			}
	}
  
  /*This method implement the required rightavoid in avoid method
   * avoid the can in fixed route, parameters decide from tile size
   */
  public void RightAvoid(int color) {
	  avoid = true;
	  returnline =false;
	  carStop();
	  leftMotor.setSpeed(ROTATE_SPEED/2);
	  rightMotor.setSpeed(ROTATE_SPEED/2);
	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
      rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
      leftMotor.setSpeed(FORWARD_SPEED/2);
      rightMotor.setSpeed(FORWARD_SPEED/2);
      leftMotor.rotate(convertDistance(leftRadius, 15), true);
      rightMotor.rotate(convertDistance(rightRadius, 15), false);
      leftMotor.setSpeed(ROTATE_SPEED/2);
	  rightMotor.setSpeed(ROTATE_SPEED/2);
	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
      rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
      leftMotor.setSpeed(FORWARD_SPEED/2);
      rightMotor.setSpeed(FORWARD_SPEED/2);
      leftMotor.rotate(convertDistance(leftRadius, 15), true);
      rightMotor.rotate(convertDistance(rightRadius, 15), false);
      carStop();
      if (color == -1) {//if first time color detection failed, redo it in another position facing the can
    	  leftMotor.setSpeed(ROTATE_SPEED/2);
    	  rightMotor.setSpeed(ROTATE_SPEED/2);
    	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
          rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
          carStop();
          leftMotor.setSpeed(FORWARD_SPEED/2);
	      rightMotor.setSpeed(FORWARD_SPEED/2);
	      //move forward to get close to can for color detection
	      leftMotor.rotate(convertDistance(leftRadius, 3), true);
	      rightMotor.rotate(convertDistance(rightRadius, 3), false);
          color = colorDetector.findColor();
    	  if (color == targetColour) {
    	    Sound.beep();
    	    this.found = true;
    	    return;
    	  }else {
    	    Sound.twoBeeps();
    	  }
    	    leftMotor.setSpeed(FORWARD_SPEED/2);
		      rightMotor.setSpeed(FORWARD_SPEED/2);
		      //move backward to ensure not touch the can
		      leftMotor.rotate(convertDistance(leftRadius, -10), true);
		      rightMotor.rotate(convertDistance(rightRadius, -10), false);
    	  
          if (search == true && ((Math.abs(odometer.getXYT()[0]-(x)) > 3)&&(Math.abs(odometer.getXYT()[1]-(y)) > 3))) {
        	  leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
              carStop();
              leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);
              carStop();
              leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
              carStop();
              leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);
              carStop();
              leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
              carStop();
              returnline = true;
          }
      }else {//color detection success, travel to next middle point
    	  if (search == true && ((Math.abs(odometer.getXYT()[0]-x) > 3)&&(Math.abs(odometer.getXYT()[1]-y) > 3))) {
              leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);
              carStop();
              leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
              carStop();
              leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);
              carStop();
              leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
              carStop();
              returnline = true;
    	  }
      }
      avoid = false;
      
	  
	}
	
  /*This method implement the required rightavoid in avoid method
   * avoid the can in fixed route, parameters decide from tile size
   */
  public void leftAvoid(int color) {
	returnline  = false;
	avoid = true;
	carStop();
    leftMotor.setSpeed(ROTATE_SPEED/2);
    rightMotor.setSpeed(ROTATE_SPEED/2);
	leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	leftMotor.setSpeed(FORWARD_SPEED/2);
	rightMotor.setSpeed(FORWARD_SPEED/2);
	leftMotor.rotate(convertDistance(leftRadius, 15), true);
	rightMotor.rotate(convertDistance(rightRadius, 15), false);
	leftMotor.setSpeed(ROTATE_SPEED/2);
    rightMotor.setSpeed(ROTATE_SPEED/2);
    leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	leftMotor.setSpeed(FORWARD_SPEED/2);
	rightMotor.setSpeed(FORWARD_SPEED/2);
	leftMotor.rotate(convertDistance(leftRadius, 15), true);
	rightMotor.rotate(convertDistance(rightRadius, 15), false);
	carStop();
	  if (color == -1) {//if first time color detection failed, redo it in another position facing the can
    	leftMotor.setSpeed(ROTATE_SPEED/2);
	    leftMotor.setSpeed(ROTATE_SPEED/2);
	    rightMotor.setSpeed(ROTATE_SPEED/2);
	 	leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
        rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);	          
	    leftMotor.setSpeed(FORWARD_SPEED/2);
        rightMotor.setSpeed(FORWARD_SPEED/2);
        //move forward to get close to can for color detection
        leftMotor.rotate(convertDistance(leftRadius, 3), true);
        rightMotor.rotate(convertDistance(rightRadius, 3), false);
        carStop();
        color = colorDetector.findColor();
	      if (color == targetColour) {
	        Sound.beep();
	    	this.found = true;
	    	return;
	      }else {
	    	 Sound.twoBeeps();
	    	  }
	      leftMotor.setSpeed(FORWARD_SPEED/2);
		  rightMotor.setSpeed(FORWARD_SPEED/2);
		  //move backward to ensure not touch the can
		  leftMotor.rotate(convertDistance(leftRadius, -10), true);
		  rightMotor.rotate(convertDistance(rightRadius, -10), false);
		      
	      if (search == true && ((Math.abs(odometer.getXYT()[0]-(x)) > 3)&&(Math.abs(odometer.getXYT()[1]-(y)) > 3))) {
	        leftMotor.setSpeed(ROTATE_SPEED/2);
	    	rightMotor.setSpeed(ROTATE_SPEED/2);
	    	leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);      
	    	rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	        carStop();
	        leftMotor.setSpeed(FORWARD_SPEED/2);
	        rightMotor.setSpeed(FORWARD_SPEED/2);
	        leftMotor.rotate(convertDistance(leftRadius, 15), true);
	        rightMotor.rotate(convertDistance(rightRadius, 15), false);
	        leftMotor.setSpeed(ROTATE_SPEED/2);
	        rightMotor.setSpeed(ROTATE_SPEED/2);
	     	leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	        rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	        leftMotor.setSpeed(FORWARD_SPEED/2);
	        rightMotor.setSpeed(FORWARD_SPEED/2);
	        leftMotor.rotate(convertDistance(leftRadius, 15), true);
	        rightMotor.rotate(convertDistance(rightRadius, 15), false);
	        leftMotor.setSpeed(ROTATE_SPEED/2);
	        rightMotor.setSpeed(ROTATE_SPEED/2);
	     	leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	        rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	        carStop();
	        returnline = true;
	          }
	        }else {//color detection success, travel to next middle point
	    	  if (search == true && ((Math.abs(odometer.getXYT()[0]-(x)) > 3)&&(Math.abs(odometer.getXYT()[1]-(y)) > 3))) {
	            leftMotor.setSpeed(FORWARD_SPEED/2);
	            rightMotor.setSpeed(FORWARD_SPEED/2);
	            leftMotor.rotate(convertDistance(leftRadius, 15), true);
	            rightMotor.rotate(convertDistance(rightRadius, 15), false);
	            leftMotor.setSpeed(ROTATE_SPEED/2);
	         	rightMotor.setSpeed(ROTATE_SPEED/2);
	            leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	            rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	            odometer.setTheta(180);
	            leftMotor.setSpeed(FORWARD_SPEED/2);
	            rightMotor.setSpeed(FORWARD_SPEED/2);
	            leftMotor.rotate(convertDistance(leftRadius, 15), true);
	            rightMotor.rotate(convertDistance(rightRadius, 15), false);
	            leftMotor.setSpeed(ROTATE_SPEED/2);
	        	rightMotor.setSpeed(ROTATE_SPEED/2);
	        	leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	            rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	            carStop();
	              returnline = true;
	    	  }
		avoid = false;
	}
	}


  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  
  /* This method implement main searching route
   * On leftmost vertical line, turn right to check if can exist in one horizontal line
   * Otherwise turn left to check
   */
  public void searchingcan(int Rx,int Ry) throws OdometerExceptions {
	this.planningType = 'P';
	this.firstSide = true;
	this.searching = true;
	for(int i = 0;i <= Ry;i++) {
		  
		if(this.firstSide) {
		    turnTo(90);
		  }else {
			turnTo(270);
		  }
		if ((SensorPoller.realDistance)<(Rx*tileSize)) {//can exist
			returnline = true;
			this.search = true;
		    if(this.firstSide) {//travel to next line after detect
				  travelTo(this.URx*tileSize,(this.LLy+i)*tileSize);
			  }else {
				  travelTo(this.LLx*tileSize,(this.LLy+i)*tileSize);
			  }
			 this.firstSide = !this.firstSide;
		  }
	    if (!returnline) {
			if (this.firstSide) {//travel to next middle point after first can color detected
			   leftMotor.setSpeed(ROTATE_SPEED/2);
	           rightMotor.setSpeed(ROTATE_SPEED/2);
	           leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	           rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	           leftMotor.setSpeed(FORWARD_SPEED/2);
	           rightMotor.setSpeed(FORWARD_SPEED/2);
	           leftMotor.rotate(convertDistance(leftRadius, 15), true);
	           rightMotor.rotate(convertDistance(rightRadius, 15), false);
	           leftMotor.setSpeed(ROTATE_SPEED/2);
	           rightMotor.setSpeed(ROTATE_SPEED/2);
	           leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	           rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	           leftMotor.setSpeed(FORWARD_SPEED/2);
	           rightMotor.setSpeed(FORWARD_SPEED/2);
	           leftMotor.rotate(convertDistance(leftRadius, 15), true);
	           rightMotor.rotate(convertDistance(rightRadius, 15), false);
	           carStop();
			   }else {
			   leftMotor.setSpeed(ROTATE_SPEED/2);
	           rightMotor.setSpeed(ROTATE_SPEED/2);
	           leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	           rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	           leftMotor.setSpeed(FORWARD_SPEED/2);
	           rightMotor.setSpeed(FORWARD_SPEED/2);
	           leftMotor.rotate(convertDistance(leftRadius, 15), true);
	           rightMotor.rotate(convertDistance(rightRadius, 15), false);	         
	           leftMotor.setSpeed(ROTATE_SPEED/2);
	           rightMotor.setSpeed(ROTATE_SPEED/2);
	           leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	           rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	           leftMotor.setSpeed(FORWARD_SPEED/2);
	           rightMotor.setSpeed(FORWARD_SPEED/2);
	           leftMotor.rotate(convertDistance(leftRadius, 15), true);
	           rightMotor.rotate(convertDistance(rightRadius, 15), false);
			  }
		  }
		 this.search = false; 
		  if (this.found) {
			  break;
		  }
		  if(i < Ry) {//travel to next horizontal line
			    if(this.firstSide) {
			      travelTo(this.LLx*tileSize,(this.LLy+i+1)*tileSize);
			    }else {
			    	travelTo(this.URx*tileSize,(this.LLy+i+1)*tileSize);
			    }
			  }
	  }
	  searching = false;
	  //move backward for a bit to not move the can
	  leftMotor.rotate(convertDistance(leftRadius, -7), true);
      rightMotor.rotate(convertDistance(rightRadius, -7), false);
	  travelTo(URx*tileSize,URy*tileSize);
  }
  /* This method calculate current position and if not in middle point
   * Do a return to middle point operation
   */
  public boolean closeSearchEnd() {
	 return (Math.abs(odometer.getXYT()[0]-x) <= 3)&&(Math.abs(odometer.getXYT()[1]-y) <= 15) || (Math.abs(odometer.getXYT()[0]-x) <= 18)&&(Math.abs(odometer.getXYT()[1]-y) <= 3);
  }
  /*travel to start point of search area
   *Then start searching
   * @see java.lang.Thread#run()
   */
  public void run() {
	  try {
		travelTo(LLx * tileSize,LLy*tileSize);
	} catch (OdometerExceptions e1) {
		// TODO Auto-generated catch block
		e1.printStackTrace();
	}
	  Sound.beep();
	  int Rx = this.URx - this.LLx;//calculate size of searching area
	  int Ry = this.URy - this.LLy;//calculate size of searching area
		try {
		searchingcan(Rx,Ry);
		}catch (OdometerExceptions e) {
				
		  e.printStackTrace();
			}
	  }
  
  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param radius
   * @param distance
   * @return
   */
  public void turnTo (double theta) {
	  double angle,smallestAngle;
	  angle = odometer.getXYT()[2];
	  
	    if((theta - angle) > 180) {
	    	smallestAngle = theta - angle - 360;
	    }
	    else if((theta - angle) < -180) {
	    	smallestAngle = theta - angle + 360;
	    }
	    else {
	    	smallestAngle = theta - angle;
	    }
	       
	        leftMotor.setSpeed(ROTATE_SPEED);
	        rightMotor.setSpeed(ROTATE_SPEED);

	        leftMotor.rotate(convertAngle(leftRadius, track, smallestAngle), true);
	        rightMotor.rotate(-convertAngle(rightRadius, track, smallestAngle), false);
  }
  /*This is a helper method for stopping robot immediately
   * Reduce error
   */
  private void carStop() {
	    leftMotor.stop(true);
	    rightMotor.stop();
	  }
}

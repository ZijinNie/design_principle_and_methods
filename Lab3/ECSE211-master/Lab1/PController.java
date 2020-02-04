package ca.mcgill.ecse211.Lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 200;
	private static final int FILTER_OUT = 20;
	private static final int maxCorrection = 160;

	private static final double PROPORTION_CONST = 5;

	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int filterControl;

	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;

		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
		int diff, leftSpeed, rightSpeed;
		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
	    // could have).
	    //

		
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		int distError = bandCenter - this.distance;

		// Moving FORWARD if error is smaller than bandwidth
		if (Math.abs(distError) <= bandWidth) {
			leftSpeed = MOTOR_SPEED;
			rightSpeed = MOTOR_SPEED;
			WallFollowingLab.leftMotor.setSpeed(leftSpeed);
			WallFollowingLab.rightMotor.setSpeed(rightSpeed);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();

			//Turning RIGHT as robot is approaching the wall
		} else if (distError < 45 && distError > 0){

			//Pivot on itself to the RIGHT if too close to wall
			if (this.distance < 27) {
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED / 2 );
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED / 2 );
				WallFollowingLab.leftMotor.forward(); 
				WallFollowingLab.rightMotor.backward();
			}
			//Turn RIGHT as robot is approaching the wall but not too close
			//Proportionally increasing left motor speed
			else{
				diff = Correction(distError);
				leftSpeed = MOTOR_SPEED + diff;
				rightSpeed = MOTOR_SPEED - diff/3; //not turn too sharp
				WallFollowingLab.leftMotor.setSpeed(leftSpeed); 
				WallFollowingLab.rightMotor.setSpeed(rightSpeed);
				WallFollowingLab.leftMotor.forward();
				WallFollowingLab.rightMotor.forward();
			}
			//Turning LEFT as the robot is leaving the wall
			//Proportionally increasing right motor speed
		} else if (distError < 0) {
			diff = Correction(distError);
			leftSpeed = MOTOR_SPEED - diff/4;
			rightSpeed = MOTOR_SPEED + diff;
			WallFollowingLab.leftMotor.setSpeed(leftSpeed); // 
			WallFollowingLab.rightMotor.setSpeed(rightSpeed);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		//turn left but in constant speed for large convex
		//may use max speed for turning because of large reading, so use constant
		else if (distError > 45) {
			WallFollowingLab.leftMotor.setSpeed(120);
			WallFollowingLab.rightMotor.setSpeed(180);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		}
	

	private int Correction(int error) {
		int speedModify;
		if (error < 0)
			error = Math.abs(error);

		speedModify = (int) (PROPORTION_CONST * error);

		if (speedModify > maxCorrection)
			speedModify = 160;

		return speedModify;

	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
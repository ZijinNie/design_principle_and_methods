package ca.mcgill.ecse211.Lab1;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

	private final int bandCenter; //33
	private final int bandWidth; //3
	private final int motorLow; //90
	private final int motorHigh; //275
	private int distance;

	private static final int FILTER_OUT = 20;
	private int filterControl;

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;

		this.filterControl = 0;
		

		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
       // rudimentary filter - toss out invalid samples corresponding to null signal 
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value: do not set the distance var, however 
			//do increment the filter value
			this.filterControl++;
		} 
		else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		}
		
		else {
			// distance went below 255: reset filter and leave
			// distance alone.
			this.filterControl = 0;
			this.distance = distance;
		}

		//Turn LEFT if distance over 33 + 3
		if (this.distance > bandCenter + bandWidth) {
			// turn LEFT by setspeed of rightmotor greater than leftmotor)
			//turn not too sharp so constant speed not motorhigh and motorlow
			WallFollowingLab.leftMotor.setSpeed(125);
			WallFollowingLab.rightMotor.setSpeed(200);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}

		//Turn RIGHT if 10 < distance < 27
		else if (this.distance < bandCenter - bandWidth && this.distance > 17) {
			//turn RIGHT by setspeed of leftmotor greater than rightmotor)
			WallFollowingLab.leftMotor.setSpeed(motorHigh);
			WallFollowingLab.rightMotor.setSpeed(motorLow);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();

			
		} 
		
		else if (this.distance <= 17) {
			//In this condition, car is too close to wall, so pivot on itself to the RIGHT 
			//set right motor with same highspeed but backward 
				WallFollowingLab.leftMotor.setSpeed(motorHigh/2);
				WallFollowingLab.rightMotor.setSpeed(motorHigh/2);
				WallFollowingLab.leftMotor.forward(); 
				WallFollowingLab.rightMotor.backward();
			}
		//} 
		//Moving FORWARD (distance is correct)
		else 
	{
			WallFollowingLab.leftMotor.setSpeed(220);
			WallFollowingLab.rightMotor.setSpeed(220);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();

		}

	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
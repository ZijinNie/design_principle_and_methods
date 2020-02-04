package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.localization.*;



public class Lab4 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static boolean Risingorfalling = true;

	//Robot related parameters
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 14.7;

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd); 

		@SuppressWarnings("resource") // Because we don't bother to close this resource
		// Instance  ultrasonicsensor 
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
		// usDistance fetch samples from this instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");

		do {
			// clear display
			lcd.clear();

			// ask the user whether the motors should use rising or falling edge
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("Rising |Falling ", 0, 2);
			lcd.drawString(" Edge  |  Edge  ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		// Select which edge to use.
		if (buttonChoice == Button.ID_LEFT) {
			Risingorfalling = true;
		} else {
			Risingorfalling = false;
		}

		// Start odometer and display threads
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		// Create ultrasonicsensor and light localizer objects
		USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, Risingorfalling, usDistance);
		LightLocalizer lightLocatizer = new LightLocalizer(odometer, leftMotor, rightMotor);

		// start the ultrasonic localization
		USLocalizer.localize();
		if(Button.waitForAnyPress() == Button.ID_ESCAPE) {
			System.exit(0);
		}
		// wait for right button to be pressed to start light localization
		while (Button.waitForAnyPress() != Button.ID_RIGHT)
			;

		// implement the light sensor localization
		lightLocatizer.localize();
        //End process
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}
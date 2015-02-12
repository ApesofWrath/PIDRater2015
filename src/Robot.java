package org.usfirst.frc.team668.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

/**
 * I deleted this comment because Ariel had too much stuff about communism here.
 * NOTE: AUTONOMOUS METHODS CALL THEIR RESPECTIVE TELEOPERATIONAL COUNTERPARTS.
 * NOTE2: This class can also be used to change PID because all algorithm-based
 * items are in the ratePID() method
 */
public class Robot extends IterativeRobot {

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public static CANTalon hammerTalon;
	public static PowerDistributionPanel pdp;
	public static Encoder hammerEncoder;
	public static PIDController pid;
	public static Joystick joystickOp;
	public static Timer time, ti;
	public static double pVal = -0.018; // These are the current tuning values
	public static double iVal = -0.001; // They can still be changed in the code
	public static double dVal = -0.030;
	public static boolean go = true;
	public static double scale = 0.001;
	public static int setpoint = 65;
	public static double current = 0.0;
	public static boolean button3 = false, button4 = false, button5 = false,
			button6 = false, button7 = false, button8 = false, button9 = false,
			button10 = false, button11 = false, button12 = false;
	public static double[][] pidTesting, pidValues;
	public static int i;

	public void robotInit() {
		hammerTalon = new CANTalon(2);
		hammerEncoder = new Encoder(9, 8);
		hammerEncoder.setPIDSourceParameter(PIDSourceParameter.kDistance);
		hammerTalon.reverseOutput(false);
		pid = new PIDController(0.0, 0.0, 0.0, hammerEncoder, hammerTalon);
		joystickOp = new Joystick(1);
		hammerEncoder.reset();
		pdp = new PowerDistributionPanel();
		time = new Timer();
	}

	public void autonomousInit() {
		teleopInit();
	}

	public void disabledInit() {
		super.disabledInit(); // Calls the Iterative Robot disabledInit()
								// because there is no point in us writing one
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		teleopPeriodic();
	}

	public void teleopInit() {
		hammerEncoder.reset();
		pid.setPID(pVal, iVal, dVal);
		pid.setSetpoint(setpoint); // Predetermined setpoint
		go = true;
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		if (!pid.isEnable()) { // disable() will automatically enable PID after
								// 8 seconds
			disable();
		} else if (go == false) {
			Timer.delay(Long.MAX_VALUE); // Purposeful thread hanging
		} else {
			checkFlip();
			printDashboard();

			pid.setPID(pVal, iVal, dVal);
			pid.setSetpoint(setpoint);
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testInit() {
		pidTesting = new double[50][7]; // 50 is a random value
		// Array > ArrayList because it is so much simpler

		/*
		 * pidTesting values: 0 1 2 3 4 5 6 P I D AccuracyScore
		 * TimeOscillationScore CurrentScore OverallScore
		 */
		pidValues = new double[50][3];
		for (i = 0; i < 50; i++) {
			pidValues[i][0] = -Math.random();
			pidValues[i][1] = -Math.random();
			pidValues[i][2] = -Math.random();
		}

		i = 0;
	}

	public void testPeriodic() {
		if (!PIDRater.getValueDone) {
			pVal = pidValues[i][0];
			iVal = pidValues[i][1];
			dVal = pidValues[i][2];
			teleopPeriodic();
			ratePID();
		} else {
			pidTesting[i][0] = pid.getP();
			pidTesting[i][1] = pid.getI();
			pidTesting[i][2] = pid.getD();
			pidTesting[i][3] = PIDRater.accuracyScore;
			pidTesting[i][4] = PIDRater.timeOscillationScore;
			pidTesting[i][5] = PIDRater.currentScore;
			pidTesting[i][6] = PIDRater.overallScore;

			pid.setPID(pVal, iVal, dVal);
			teleopInit();
			PIDRater.getValueDone = false;
			i++;
		}
	}

	public void checkFlip() {
		if (hammerEncoder.getDistance() > 150.0) {
			pid.disable();
			disable();
			go = false;
		}
	}

	public void pidPeriodic() {
		pid.setSetpoint(setpoint);

		checkFlip(); // Redundancy in safety is good
		printDashboard();
		doButtons();
		checkFlip(); // Redundancy in safety is good

		pid.setPID(pVal, iVal, dVal);
		if (joystickOp.getRawButton(2)) {
			pid.disable(); // THIS DISABLE IS FOR THE PID, NOT THE ROBOT
			SmartDashboard.putBoolean("Disabled", true);
		} else if (joystickOp.getRawButton(1)) { // We want disable to be
													// priority over enable and
													// only have one work
			pid.enable();
			SmartDashboard.putBoolean("Disabled", false);
		}
	}

	public static void doButtons() {
		if (joystickOp.getRawButton(5) && button5 == false) { // Scale
			scale *= 10;
			button5 = true;
		} else if (!joystickOp.getRawButton(5)) {
			button5 = false;
		}
		if (joystickOp.getRawButton(3) && button3 == false) {
			scale /= 10;
			button3 = true;
		} else if (!joystickOp.getRawButton(3)) {
			button3 = false;
		}

		if (joystickOp.getRawButton(6) && button6 == false) { // Setpoints are
																// integers (not
																// doubles)
			setpoint += scale * 1000;
			button6 = true;
		} else if (!joystickOp.getRawButton(6)) {
			button6 = false;
		}
		if (joystickOp.getRawButton(4) && button4 == false) {
			setpoint -= scale * 1000;
			button4 = true;
		} else if (!joystickOp.getRawButton(4)) {
			button4 = false;
		}

		if (joystickOp.getRawButton(7) && button7 == false) {
			pVal += scale;
			button7 = true;
		} else if (!joystickOp.getRawButton(7)) {
			button7 = false;
		}
		if (joystickOp.getRawButton(8) && button8 == false) {
			pVal -= scale;
			button8 = true;
		} else if (!joystickOp.getRawButton(8)) {
			button8 = false;
		}
		if (joystickOp.getRawButton(9) && button9 == false) {
			iVal += scale;
			button9 = true;
		} else if (!joystickOp.getRawButton(9)) {
			button9 = false;
		}
		if (joystickOp.getRawButton(10) && button10 == false) {
			iVal -= scale;
			button10 = true;
		} else if (!joystickOp.getRawButton(10)) {
			button10 = false;
		}
		if (joystickOp.getRawButton(11) && button11 == false) {
			dVal += scale;
			button11 = true;
		} else if (!joystickOp.getRawButton(11)) {
			button11 = false;
		}
		if (joystickOp.getRawButton(12) && button12 == false) {
			dVal -= scale;
			button12 = true;
		} else if (!joystickOp.getRawButton(12)) {
			button12 = false;
		}
	}

	public static void printDashboard() {
		SmartDashboard.putNumber("P", pid.getP());
		SmartDashboard.putNumber("I", pid.getI());
		SmartDashboard.putNumber("D", pid.getD());
		SmartDashboard.putNumber("PID Error", pid.getError());
		SmartDashboard.putNumber("Scale", scale);
		SmartDashboard.putNumber("Setpoint", setpoint);
		SmartDashboard.putBoolean("Disabled", pid.isEnabled());

		SmartDashboard.putNumber("Hammer Encoder Distance",
				hammerEncoder.getDistance());
		SmartDashboard.putNumber("Current", hammerTalon.getOutputCurrent());
		SmartDashboard.putNumber("Motor Speed", hammerTalon.getSpeed());
	}

	public static void enable() {
		pid.enable();
		SmartDashboard.putBoolean("Disabled", false);
	}

	public static void disable() {
		pid.disable();
		SmartDashboard.putBoolean("Disabled", true);
		ti = new Timer();
		ti.reset();
		ti.start();
		if (ti.get() >= 8) {
			enable();
		} else {
			disable();
		}
	}

	public static void noGo() {
		go = false;
		Timer.delay(Double.MAX_VALUE); // This will purposefully hang the thread
	}

	public static void ratePID() { // All PID algorithms called here
		time.start();
		PIDRater.checkStop();
		PIDRater.getValues();
		PIDRater.processValues();
		PIDRater.ratePID();
	}
}

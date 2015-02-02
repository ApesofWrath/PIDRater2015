package org.usfirst.frc.team668.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 *Rater for any PID code that will return a score out of 100% based 
 *on encoder readouts for PID error and motor current draw graph.
 */
public class PIDRater {

    public static double accuracyScore, timeOscillationScore, currentScore, overallScore;
    private static double currentTime;
    private static double currentAccuracy, lastAccuracy, secondToLastAccuracy;

    private static double timeOscillation, current, accuracy;
    public static boolean getValueDone;

    /*
     The three criteria we will use will be as follows:
     -Time Oscillation (Integral of Time-Oscillation graph)
     -Current (Integral of Amps-Secs graph)
     -Accuracy (At finish, absolute value of distance from percent error 0)
    
     Stops:
     -10 seconds (Safety)
     -100+ units of absolute value of PID Error
     -Over 100 amps of current at any given time
     -Over 20 amps of current after 5 seconds
     */
    public static void checkStop() {
	if (Robot.time.get() >= PIDMap.MAXIMUM_TIME) {
	    stop("Time");
	    Robot.disable();
	}
	if (Math.abs(Robot.pid.getError()) >= PIDMap.MAXIMUM_PID_ERROR) {
	    stop("PID Error");
	    Robot.disable();
	}
	if (Robot.hammerTalon.getOutputCurrent() >= PIDMap.MAXIMUM_INIT_CURRENT) {
	    stop("Current Init");
	    Robot.disable();
	}
	if (Robot.hammerTalon.getOutputCurrent() >= PIDMap.MAXIMUM_SUSTAIN_CURRENT && Robot.time.get() >= PIDMap.CURRENT_CHECK_TIME) {
	    stop("Current Sustained");
	    Robot.disable();
	}
    }

    public static void stop(String errorMessage) {
	Robot.hammerTalon.set(0.0);
	Robot.pid.disable();
	SmartDashboard.putString("STOP", errorMessage);
	Robot.noGo(); // Must turn code off then onn
    }

    public static void getValues() {
	if (!getValueDone) {
	    timeOscillation += Robot.time.get() * Math.abs(Robot.pid.getError());
	    current += Robot.time.get() * Robot.hammerTalon.getOutputCurrent();

	    secondToLastAccuracy = lastAccuracy;
	    lastAccuracy = currentAccuracy;
	    currentAccuracy = Robot.pid.getError();

	    // These boolean statements simplify the if statements
	    boolean a = currentAccuracy < lastAccuracy + PIDMap.PID_ERROR_THRESHOLD;
	    boolean b = currentAccuracy > lastAccuracy - PIDMap.PID_ERROR_THRESHOLD;
	    boolean c = currentAccuracy < secondToLastAccuracy + PIDMap.PID_ERROR_THRESHOLD;
	    boolean d = currentAccuracy > secondToLastAccuracy - PIDMap.PID_ERROR_THRESHOLD;

	    if (a && b) {
		if (c && d) { // Inside both deadzones -> three consecutive values within 1
		    getValueDone = true;
		    accuracy = Math.abs((currentAccuracy + lastAccuracy + secondToLastAccuracy) / 3.0);
		    // Average of errors shows accuracy. Absolute value is because accuracy isn't negative.
		}
	    }
	} else { // Of it is done, clear values
	    currentAccuracy = 0;
	    lastAccuracy = 0;
	    secondToLastAccuracy = 0;
	    timeOscillation = 0;
	    current = 0;
	    getValueDone = false;
	    Robot.time.stop();
	    Robot.time.reset();
	}
    }

    public static void processValues() { // Called constantly; Need error handling?
	accuracyScore = 100 - (10 * accuracy);
	currentScore = 100 - ((current / PIDMap.CURRENT_MAXIMUM) * 100);
	timeOscillationScore = 100 - ((timeOscillation / PIDMap.TIME_OSCILLATION_MAXIMUM) * 100);

	if (accuracyScore < 0) {
	    accuracyScore = 0;
	    SmartDashboard.putBoolean("Accuracy Score", false);
	} else {
	    SmartDashboard.putBoolean("Accuracy Score", true);
	}
	if (currentScore < 0) {
	    currentScore = 0;
	    SmartDashboard.putBoolean("Current Score", false);
	} else {
	    SmartDashboard.putBoolean("Current Score", true);
	}
	if (timeOscillationScore < 0) {
	    timeOscillationScore = 0;
	    SmartDashboard.putBoolean("Time Oscillation Score", false);
	} else {
	    SmartDashboard.putBoolean("Time Oscillation Score", true);
	}

	// Accuracy is rated from 0-100
	// Current is scaled into a percentage relative to the maximum and them rated from 0-100
	// Time Oscillation is also scaled to max and rated 0-100
    }

    public static double ratePID() {
	overallScore = (accuracyScore + currentScore + timeOscillationScore) / 3.0; // Average of three scores -> percentage scale
	SmartDashboard.putNumber("Accuracy Score", accuracyScore);
	SmartDashboard.putNumber("Current Score", currentScore);
	SmartDashboard.putNumber("Time Oscillation Score", timeOscillationScore);
	SmartDashboard.putNumber("Overall Score", overallScore);
	return overallScore; // Overall score is out of 100
    }
}

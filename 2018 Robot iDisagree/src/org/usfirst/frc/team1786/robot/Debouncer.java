package org.usfirst.frc.team1786.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

/**
 * Debouncer allows for simple handling of time-based joystick button debouncing
 *
 * <p>Features setDebouncePeriod and get methods
 */
public class Debouncer {

	Timer timer;
	Joystick joystick;
	int buttonNum;
	double period;
	double latest = 0;
	double now;
	
	/**
	 * Constructor
	 * @param joy - wpilib.Joystick object
	 * @param button - wplib.Joystick button channel
	 * @param waitPeriod - wait period in seconds between state change
	 */
	public Debouncer(Joystick joy, int button, double waitPeriod) {
		joystick = joy;
		buttonNum = button;
		period = waitPeriod;
	}
	
	/**
	 * Change the debounce wait period after construction
	 * @param waitPeriod - wait period in seconds between state change
	 */
	public void setDebouncePeriod(double waitPeriod) {
		period = waitPeriod;
	}
	
	/**
	 * Get state of button with debounce filtering
	 * @return boolean - state of button
	 */
	public boolean get() {
		now = Timer.getFPGATimestamp();
		if(joystick.getRawButton(buttonNum)) {
			if ((now - latest) > period) {
				latest = now;
				return true;
			}
		};
		return false;
	}
		

};

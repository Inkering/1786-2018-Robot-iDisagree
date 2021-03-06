/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1786.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.buttons.*;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
// welcome to my branch boys
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	WPI_TalonSRX talonL1 = new WPI_TalonSRX(1);
	WPI_TalonSRX talonL2 = new WPI_TalonSRX(2);
	WPI_TalonSRX talonL3 = new WPI_TalonSRX(3);
	WPI_TalonSRX talonR4 = new WPI_TalonSRX(4);
	WPI_TalonSRX talonR5 = new WPI_TalonSRX(5);
	WPI_TalonSRX talonR6 = new WPI_TalonSRX(6);
	
	Joystick joystickLeft = new Joystick(0);
	Joystick joystickRight = new Joystick(1);
	
	JoystickButton shiftUp = new JoystickButton(joystickLeft, 3);
	JoystickButton shiftDown = new JoystickButton(joystickLeft, 4);
	
	Compressor robotCompressor = new Compressor();
	
	
	DifferentialDrive myRobot = new DifferentialDrive(talonL1, talonR4);
	
	private int maxPeakAmp = 60; //defines the max amp that can be given to a moter during its peak
	private int maxCountAmp = 40; //defines the max amp that can be given to a moter after its peak
	private int peakTimeDuration = 10000; //defines how long the peak will last in milliseconds
	
	
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		
		talonL2.follow(talonL1); //tells the following talons to follow their leading talons
		talonL3.follow(talonL1);
		talonR5.follow(talonR4);
		talonR6.follow(talonR4);
		
		Double deadband = .05; //defines the deadzone
		
		myRobot.setDeadband(deadband); //sets the deadzone
		
		
		
		talonL1.configPeakCurrentDuration(peakTimeDuration, 0); //sets the duration of the peak
		talonL1.configPeakCurrentLimit(maxPeakAmp, 0); //sets the max current of the peak
		talonL1.configContinuousCurrentLimit(maxCountAmp, 0); //sets the max current for the time after the peak
		
		talonR4.configPeakCurrentDuration(peakTimeDuration, 0); //same as the other one
		talonR4.configPeakCurrentLimit(maxPeakAmp, 0);
		talonR4.configContinuousCurrentLimit(maxCountAmp, 0);
		
		
		
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		
		Double dispYValueLeft = joystickLeft.getY(); //puts the left joysticks Y value into a variable
		Double xValueLeft = joystickLeft.getX();//puts the left joysticks X value into a variable
		Double zValueLeft = joystickLeft.getZ();//puts the left joysticks Z value into a variable
		SmartDashboard.putNumber("Y value", dispYValueLeft); //displays the y value on computer
		SmartDashboard.putNumber("X value", xValueLeft); //displays the x value on computer
		SmartDashboard.putNumber("Z value", zValueLeft); //displays the z value on computer
		
		Double driveYValue = -(joystickLeft.getY()); //inverts the y value so that foward is foward
		
		double talon1 = talonL1.getOutputCurrent(); //defines the talons AMP values
		double talon2 = talonL2.getOutputCurrent();
		double talon3 = talonL3.getOutputCurrent();
		double talon4 = talonR4.getOutputCurrent();
		double talon5 = talonR5.getOutputCurrent();
		double talon6 = talonR6.getOutputCurrent();
		SmartDashboard.putNumber("Talon1", talon1); //displays all the talon AMP values
		SmartDashboard.putNumber("Talon2", talon2);
		SmartDashboard.putNumber("Talon3", talon3);
		SmartDashboard.putNumber("Talon4", talon4);
		SmartDashboard.putNumber("Talon5", talon5);
		SmartDashboard.putNumber("Talon6", talon6);
		
		
		
		
		//myRobot.arcadeDrive(driveYValue, zValueLeft, true); //alows the robot to drive with scaling using the y and z values from the left joystick
		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		
		while(robotCompressor.getPressureSwitchValue())
		{
			robotCompressor.setClosedLoopControl(true);
		}
			robotCompressor.setClosedLoopControl(false);
		}
			
	
}

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
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.ErrorCode;;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();


	
	//Declare Joysticks
	Joystick joystickLeft = new Joystick(0);
	Joystick joystickRight = new Joystick(1);
	
	//Declaring Talons
	WPI_TalonSRX talonL1 = new WPI_TalonSRX(1);
	WPI_TalonSRX talonL2 = new WPI_TalonSRX(2);
	WPI_TalonSRX talonL3 = new WPI_TalonSRX(3);
	WPI_TalonSRX talonR4 = new WPI_TalonSRX(4);
	WPI_TalonSRX talonR5 = new WPI_TalonSRX(5);
	WPI_TalonSRX talonR6 = new WPI_TalonSRX(6);
	
	DifferentialDrive myRobot = new DifferentialDrive(talonL1, talonR4);
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		//Talon follows
		talonL2.follow(talonL1);
		talonL3.follow(talonL1);
		talonR5.follow(talonR4);
		talonR6.follow(talonR4);
		
		//Set and apply deadband
		double deadband = 0.05;
		myRobot.setDeadband(deadband);
		
		//Current Limiter
		
		int contCurrent = 40;
		int peakDuration = 10000;
		int peakCurrent = 60;
		
		talonL1.configContinuousCurrentLimit(contCurrent, 0);
		talonR4.configContinuousCurrentLimit(contCurrent, 0);
		talonL2.configContinuousCurrentLimit(contCurrent, 0);
		talonL3.configContinuousCurrentLimit(contCurrent, 0);
		talonR5.configContinuousCurrentLimit(contCurrent, 0);
		talonR6.configContinuousCurrentLimit(contCurrent, 0);
		
		talonL1.configPeakCurrentLimit(peakCurrent, 0);
		talonR4.configPeakCurrentLimit(peakCurrent, 0);
		talonL2.configPeakCurrentLimit(peakCurrent, 0);
		talonL3.configPeakCurrentLimit(peakCurrent, 0);
		talonR5.configPeakCurrentLimit(peakCurrent, 0);
		talonR5.configPeakCurrentLimit(peakCurrent, 0);
		
		talonL1.configPeakCurrentDuration(peakDuration, 0);
		talonR4.configPeakCurrentDuration(peakDuration, 0);
		talonL2.configPeakCurrentDuration(peakDuration, 0);
		talonL3.configPeakCurrentDuration(peakDuration, 0);
		talonR5.configPeakCurrentDuration(peakDuration, 0);
		talonR5.configPeakCurrentDuration(peakDuration, 0);
		
		talonL1.enableCurrentLimit(true);
		talonR4.enableCurrentLimit(true);
		talonL2.enableCurrentLimit(true);
		talonL3.enableCurrentLimit(true);
		talonR5.enableCurrentLimit(true);
		talonR5.enableCurrentLimit(true);
		
		
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
		
		
		
		myRobot.arcadeDrive(joystickLeft.getY(), joystickLeft.getZ(), true);
		
		
		
		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}

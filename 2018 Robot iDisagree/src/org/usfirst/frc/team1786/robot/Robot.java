/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Will
package org.usfirst.frc.team1786.robot;
 

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import java.lang.Math;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	
	Joystick stick1 = new Joystick(1);
	Joystick stick2 = new Joystick(2);
	
	//Current limit 120 amps
	WPI_TalonSRX robotLeft = new WPI_TalonSRX(1);
	WPI_TalonSRX robotLeftSlave1 = new WPI_TalonSRX(2);
	WPI_TalonSRX robotLeftSlave2 = new WPI_TalonSRX(3);
	
	WPI_TalonSRX robotRight = new WPI_TalonSRX(4);
	WPI_TalonSRX robotRightSlave1 = new WPI_TalonSRX(5);
	WPI_TalonSRX robotRightSlave2 = new WPI_TalonSRX(6);
	
	//current limiting
	
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		robotLeftSlave1.follow(robotLeft);
		robotLeftSlave2.follow(robotLeft);
		
		robotRightSlave1.follow(robotRight);
		robotRightSlave2.follow(robotRight);
		
		//current limiting
		robotLeft.configContinuousCurrentLimit(120, 1);
		robotRight.configContinuousCurrentLimit(120, 1);
		
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
		double x = stick1.getX();
		double y = stick1.getY();
		double z = stick1.getZ();
		
		//get current & display current;
		
		SmartDashboard.putNumber("Left Voltage: ", robotLeft.getOutputCurrent());
		SmartDashboard.putNumber("Right Voltage: ", robotRight.getOutputCurrent());
		SmartDashboard.putNumber("Left Temperature: ", robotLeft.getTemperature());
		SmartDashboard.putNumber("Right Temperature: ", robotRight.getTemperature());
		
		
		if(z < -0.1)
		{
			//twist right
			
			robotLeft.set(z);
			robotRight.set(-z);
		}
		else if(z > 0.1)
		{
			//twist left
			
			robotLeft.set(-z);
			robotRight.set(z);
		}
		else
		{
			double power = Math.sqrt((x*x)+(y*y));
			if (power > 1)
				power=1;
			if (y > 0)
				power=-power;
			double scale = 1-Math.abs(x);
			if(x>0)
			{
				//left
				
				robotLeft.set(power*scale);
				robotRight.set(power);
			}
			else if(x<0)
			{
				//right
				
				robotLeft.set(power);
				robotRight.set(power*scale);
			}
			//nothing 
		}
		
		
		
		
		
		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}

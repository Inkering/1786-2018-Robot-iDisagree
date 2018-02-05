/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1786.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.AHRS;


public class Robot extends IterativeRobot {

	Joystick gamepad = new Joystick(0);
	
	/* CONSTANT VALUES */
	
	final int SHIFTER = 3; // rightmost pad button
	final int QTURN = 6; // right trigger
	final int REVDRIVE = 5; // left trigger
	
	WPI_TalonSRX talonL1 = new WPI_TalonSRX(1); // left Side talons
	WPI_TalonSRX talonL2 = new WPI_TalonSRX(2);
	WPI_TalonSRX talonL3 = new WPI_TalonSRX(3);
	WPI_TalonSRX talonR4 = new WPI_TalonSRX(4); // right Side Talons
	WPI_TalonSRX talonR5 = new WPI_TalonSRX(5);
	WPI_TalonSRX talonR6 = new WPI_TalonSRX(6);
	DifferentialDrive drivetrain = new DifferentialDrive(talonL1, talonR4);
	
//	Spark armLController = new Spark(1);
//	Spark armRController = new Spark(1);
	
//	WPI_TalonSRX liftTalon = new WPI_TalonSRX(7);
	
	Compressor compressor = new Compressor(); // only one compressor in system
	Solenoid shifter = new Solenoid(0);
	
	private int maxPeakAmp = 60; // defines the max amp that can be given to a CIM motor during its peak
	private int maxCountAmp = 40; // defines the max amp that can be given to a CIM motor after its peak
	private int peakTimeDuration = 10000; // defines how long the peak will last in milliseconds
	
	private Double driveDeadband = .05; // defines the deadzone of the drivetrain object
	private Double shiftLimit = 0.2; // defines upper maximum rate of movement that shifting is allowed in
	
	boolean reversedDrive;
	boolean quickTurn;
	boolean shifted;
	
	private void dashboardUpdate() {
		
		// put amp info on dashboard
		double talon1Current = talonL1.getOutputCurrent();
		double talon2Current = talonL2.getOutputCurrent();
		double talon3Current = talonL3.getOutputCurrent();
		double talon4Current = talonR4.getOutputCurrent();
		double talon5Current = talonR5.getOutputCurrent();
		double talon6Current = talonR6.getOutputCurrent();
		double compressorCurrent = compressor.getCompressorCurrent(); 
		SmartDashboard.putNumber("Talon1 Amps", talon1Current); // displays all the talon amp values
		SmartDashboard.putNumber("Talon2 Amps", talon2Current);
		SmartDashboard.putNumber("Talon3 Amps", talon3Current);
		SmartDashboard.putNumber("Talon4 Amps", talon4Current);
		SmartDashboard.putNumber("Talon5 Amps", talon5Current);
		SmartDashboard.putNumber("Talon6 Amps", talon6Current);
		SmartDashboard.putNumber("compressor amps", compressorCurrent); //display compressor amp usage
		
		// put left joystick info on dashboard
		SmartDashboard.putNumber("drive Y value", gamepad.getY());
		SmartDashboard.putNumber("drive X value", gamepad.getX());
		SmartDashboard.putNumber("drive Z value", gamepad.getZ());
		SmartDashboard.putNumber("drive twist value", gamepad.getTwist());
		SmartDashboard.putBoolean("is shift button pressed", gamepad.getRawButton(SHIFTER));
		SmartDashboard.putBoolean("is acc drive button pressed", gamepad.getRawButton(QTURN));
		
		// compressor and solenoid information (bools)
		SmartDashboard.putBoolean("solenoid drive shifter ID 0 state", shifter.get());
		SmartDashboard.putBoolean("low pressure switch state", compressor.getPressureSwitchValue());
	}
	
	@Override
	public void robotInit() {
		
		// configure talon followers.
		talonL2.follow(talonL1); 
		talonL3.follow(talonL1);
		talonR5.follow(talonR4);
		talonR6.follow(talonR4);

		// configure motor safety for arm controllers
//		armLController.setSafetyEnabled(true);
//		armRController.setSafetyEnabled(true);
//		armLController.setExpiration(100);
//		armRController.setExpiration(100);
		
		// we can just set motor safety for the differentialDrive object
		drivetrain.setDeadband(driveDeadband);
		drivetrain.setSafetyEnabled(true);
		drivetrain.setExpiration(100);
		
		// Configure talon amp limits
		talonL1.configPeakCurrentDuration(peakTimeDuration, 0); // sets the duration of the peak
		talonL1.configPeakCurrentLimit(maxPeakAmp, 0); // "Configure the peak current limit to the threshold necessary to exceed to activate current limiting"
		talonL1.configContinuousCurrentLimit(maxCountAmp, 0); // sets the max current for the time after the peak
		talonL1.enableCurrentLimit(true);
		talonR4.configPeakCurrentDuration(peakTimeDuration, 0); // same as the other one
		talonR4.configPeakCurrentLimit(maxPeakAmp, 0);
		talonR4.configContinuousCurrentLimit(maxCountAmp, 0);
		talonR4.enableCurrentLimit(true);	
	}
	
	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		reversedDrive = false;
		quickTurn = false;
		shifted = false;
	}

	// run the drivetrain in "curvature" style
	private void teleopCurvatureDrive(double y, double z) {
		
		//toggle logic for quick turning
		if(gamepad.getRawButton(QTURN) && quickTurn) {
			quickTurn = false;
		} else if (gamepad.getRawButton(QTURN) && !quickTurn) {
			quickTurn = true;
		}
		
		if(gamepad.getRawButton(REVDRIVE) && reversedDrive) {
			reversedDrive = false;
		} else if (gamepad.getRawButton(REVDRIVE) && !reversedDrive) {
			reversedDrive = true;
		}
		
		/* checks for both reversal and accuracy modes
		 * Truth Table
		 * ---------
		 * | 1 | 0 |
		 * | 0 | 1 |
		 * | 1 | 1 |
		 * | 0 | 0 |
		 * ---------
		*/
		if(quickTurn && !reversedDrive) {
			drivetrain.curvatureDrive(y, z, true);
		} else if(!quickTurn && reversedDrive) {
			drivetrain.curvatureDrive(-y, z, false);
		} else if(quickTurn && reversedDrive) {
			drivetrain.curvatureDrive(-y, z, true);
		}  else if(!quickTurn && !reversedDrive) {
			drivetrain.curvatureDrive(y, z, false);
		}
	}
	
	// run the drivetrain in "arcade" style
	private void teleopArcadeDrive(double y, double z) {
		
		if(gamepad.getRawButton(REVDRIVE) && reversedDrive) {
			reversedDrive = false;
		} else if (gamepad.getRawButton(REVDRIVE) && !reversedDrive) {
			reversedDrive = false;
		}
		
		// checks for both reversal mode
		if (reversedDrive) {
			drivetrain.arcadeDrive(-y, z,true);
		} else {
			drivetrain.arcadeDrive(y, z, true);
		}
		
	}
	
	// run the arms forwards or backwards
//	private void teleopArms(int pov) {
//		if (pov == 0) {
//			armLController.set(1);
//			armRController.set(1);
//		} else if (pov == 180) {
//			armLController.set(-1);
//			armRController.set(-1);
//		}
//		
//	}
	
	// use a talon srx breakout board for hardwired limit switches
//	private void teleopLift(double y) {
//		liftTalon.set(y);
//	}
	
	@Override
	public void teleopPeriodic() {
		
		//get joystick axis values for use later.
		Double driveZ = gamepad.getZ(); // left/right on right stick
		Double driveY = gamepad.getY(); // forward/backward on left stick
		int top = gamepad.getPOV(); //0 is up, 180 is down, 90 is right, 270 is left
		
		teleopCurvatureDrive(driveY, driveZ);
		
		/* BEGIN PNEUMATICS CODE */
		//run the compressor if the pressure is low
		// TODO add compressor current limiting and calibrate
		// TODO calibrate wiring to be correct for switch variable
		// e.g. true = low pressure
		// 		false = acceptable pressure
//		if(!compressor.getPressureSwitchValue()) {
//			System.out.println("running compressor in teleop!");
//			compressor.setClosedLoopControl(true);
//		} else {
//			compressor.setClosedLoopControl(false);
//		}
		
		//run the shifter pnuematic cylinder
		//shifts when the shifting button is pressed and pressure is sufficient for shifting
		// TODO add timer to wait for movement between shifts 
		// TODO calibrate wiring to set reasonable defaul solenoid movements
		if(gamepad.getRawButton(SHIFTER) && shifted) {
			shifted = false;
		} else if
		(gamepad.getRawButton(SHIFTER) && !shifted) {
			shifted = true;
		}
		
		//prevent shifting if moving at high speeds
		// TODO calibrate shiftLimit Value
		if (shifted) {
			System.out.println("shifted true!");
			shifter.set(true); //push out
		} else {
			shifter.set(false); //pull in
		}
		/* END PNUEMATICS CODE */
		
		//put data on dashboard
		dashboardUpdate();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		
		// Fill the compressor during autonomous
		// TODO calibrate wiring to be correct for switch variable
		// e.g. true = low pressure
		// 		false = acceptable pressure
		while(compressor.getPressureSwitchValue() == false)
		{
			compressor.setClosedLoopControl(true);
		}
		compressor.setClosedLoopControl(false);
			
	}
}	

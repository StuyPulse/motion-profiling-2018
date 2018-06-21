/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team694.robot;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

import org.usfirst.frc.team694.robot.commands.auton.DifferentSideLeftScaleAutonCommand;
import org.usfirst.frc.team694.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	public static Drivetrain drivetrain;
	public static OI m_oi;
	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public static double startTime;
	
	boolean logOpen; 
	public String logOutputDir = "/U/MotionProfileData";
	public String logName; 
	public BufferedWriter logFile; 
	@Override
	public void robotInit() {
		drivetrain = new Drivetrain();
		m_oi = new OI();
		m_chooser.addDefault("Default Auto", new CommandGroup());
		m_chooser.addObject("Different Side Left Scale", new DifferentSideLeftScaleAutonCommand());
		SmartDashboard.putData("Auto mode", m_chooser);
		SmartDashboard.putNumber("Motion Profile P", 0);
		SmartDashboard.putNumber("Motion Profile I", 0);
		SmartDashboard.putNumber("Motion Profile D", 0);
		SmartDashboard.putNumber("Accel Gain", 0);
		putSmartDashboardData(); 
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
		Robot.drivetrain.resetEncoders(); 
    	Robot.drivetrain.resetGyro();
    	startTime = Timer.getFPGATimestamp();
    	initLog("LeftDistance RightDistance LeftVelocity RightVelocity LeftAcceleration RightAcceleration", 
    			"ft ft ft/sec ft/sec ft/sec/sec ft/sec/sec"); 
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		putSmartDashboardData();
		logData();
		if(m_autonomousCommand.isCompleted()) {
			closeFile(); 
		}
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		startTime = Timer.getFPGATimestamp();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		putSmartDashboardData();
		//drivetrain.tankDrive(0.75, 0.75);
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	public void putSmartDashboardData() {
		SmartDashboard.putNumber("Left Distance", inToFt(Robot.drivetrain.getLeftDistance()));
		SmartDashboard.putNumber("Right Distance", inToFt(Robot.drivetrain.getRightDistance()));
		SmartDashboard.putNumber("Left Velocity", inToFt(Robot.drivetrain.getLeftVelocity()));
		SmartDashboard.putNumber("Right Velocity", inToFt(Robot.drivetrain.getRightVelocity()));
		SmartDashboard.putNumber("Left Acceleration", inToFt(Robot.drivetrain.getLeftAcceleration()));
		SmartDashboard.putNumber("Right Acceleration", inToFt(Robot.drivetrain.getRightAcceleration()));
	}
	
	public void logData() {
		writeToFile("" + SmartDashboard.getNumber("Left Distance", 0)
					+ SmartDashboard.getNumber("Right Distance", 0)
					+ SmartDashboard.getNumber("Left Velocity", 0)
					+ SmartDashboard.getNumber("Right Velocity", 0)
					+ SmartDashboard.getNumber("Left Acceleration", 0)
					+ SmartDashboard.getNumber("Right Acceleration", 0));
		sendData(); 
	}
	
	public double inToFt(double value) {
		return value / 12; 
	}
	
	public void initLog(String data, String units) {
		if(logOpen) {
			System.out.println("Log file is already open");
		}else {
			try {
				logOpen = false; 
				logName = logOutputDir + "log_" + getDateTimeString() + ".csv";
				FileWriter fStream = new FileWriter(logName, true);
				logFile = new BufferedWriter(fStream);
				logFile.write(data + "\n");
				logFile.write(units + "\n");
			}catch(Exception e) {
				System.out.println("Error setting up" + e.getMessage());
			}
		}
	}
	
	public void writeToFile(String text) {
		if(!logOpen) {
			System.out.println("Log file closed cannot write");
		}else {
			try {
				logFile.write(text + "\n");
			}catch(Exception e) {
				System.out.println("Error writing to file" + e.getMessage());
			}
		}
	}
	
	public void sendData() {
		if(!logOpen) {
			System.out.println("Log not open, cannot send");
		}else {
			try {
				logFile.flush();
			}catch(Exception e) {
				System.out.println("Cannot send data" + e.getMessage());
			}
		}
	}
	
	public void closeFile() {
		if(!logOpen) {
			System.out.println("File already closed");
		}else {
			sendData();
			try {
				logFile.close();
				logOpen = false; 
				System.out.println("File close");
			}catch(Exception e) {
				System.out.println("Error closing file" + e.getMessage());
			}
		}
	}
	
	public String getDateTimeString() {
		DateFormat dt = new SimpleDateFormat("MM-dd-yyyy_hh:mm:ss");
		dt.setTimeZone(TimeZone.getDefault());
		return dt.format(new Date());
	}
}

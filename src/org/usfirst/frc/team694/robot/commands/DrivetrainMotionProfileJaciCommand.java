package org.usfirst.frc.team694.robot.commands;

import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;

import org.usfirst.frc.team694.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

/**
 *
 */
public class DrivetrainMotionProfileJaciCommand extends Command {
	EncoderFollower leftFollower; 
	EncoderFollower rightFollower; 
	
	File leftCSV; 
	File rightCSV; 
	
	Trajectory leftTraj; 
	Trajectory rightTraj;
	
	double maxVelocity; 
	
	double leftPower; 
	double rightPower; 
    public DrivetrainMotionProfileJaciCommand(String nameOfPath, double maxVelocity) {
    	requires(Robot.drivetrain);
    	leftCSV = new File("src/org/usfirst/frc/team694/robot" + nameOfPath + "_left_Jaci.csv");
    	rightCSV = new File("src/org/usfirst/frc/team694/robot" + nameOfPath + "_right_Jaci.csv");
    	try {
    		URL leftURL = new URL(getClass().getResource(nameOfPath) + "_left_Jaci.csv");
    		URL rightURL = new URL(getClass().getResource(nameOfPath) + "_right_Jaci.csv");
    		Files.copy(leftURL.openStream(), leftCSV.toPath(), StandardCopyOption.REPLACE_EXISTING);
    		Files.copy(rightURL.openStream(), rightCSV.toPath(), StandardCopyOption.REPLACE_EXISTING);
    	}catch(IOException i) {
    		System.out.println("Invalid Trajectory, Aborting");
    		cancel(); 
    	}
    	this.maxVelocity = maxVelocity; 
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	leftTraj = Pathfinder.readFromCSV(leftCSV);
    	rightTraj = Pathfinder.readFromCSV(rightCSV);
    	leftFollower = new EncoderFollower(leftTraj);
    	rightFollower = new EncoderFollower(rightTraj);
    	//Wheel diameter in feet
    	//leftFollower.configureEncoder(Robot.drivetrain.leftBottomMotor.getSensorCollection().getQuadraturePosition(), Robot.drivetrain.DRIVETRAIN_ENCODER_PULSES_PER_REVOLUTION, Robot.drivetrain.DRIVETRAIN_WHEEL_DIAMETER / 12);
    	//rightFollower.configureEncoder(Robot.drivetrain.rightBottomMotor.getSensorCollection().getQuadraturePosition(), Robot.drivetrain.DRIVETRAIN_ENCODER_PULSES_PER_REVOLUTION, Robot.drivetrain.DRIVETRAIN_WHEEL_DIAMETER / 12);
    	leftFollower.configureEncoder(Robot.drivetrain.leftBottomMotor.getSelectedSensorPosition(0), Robot.drivetrain.DRIVETRAIN_ENCODER_PULSES_PER_REVOLUTION, Robot.drivetrain.DRIVETRAIN_WHEEL_DIAMETER / 12);
    	rightFollower.configureEncoder(Robot.drivetrain.rightBottomMotor.getSelectedSensorPosition(0), Robot.drivetrain.DRIVETRAIN_ENCODER_PULSES_PER_REVOLUTION, Robot.drivetrain.DRIVETRAIN_WHEEL_DIAMETER / 12);
    	leftFollower.configurePIDVA(SmartDashboard.getNumber("Motion Profile P", 0.006), SmartDashboard.getNumber("Motion Profile I", 0), SmartDashboard.getNumber("Motion Profile D", 0.03), 1 / maxVelocity, 0);
    	rightFollower.configurePIDVA(SmartDashboard.getNumber("Motion Profile P", 0.006), SmartDashboard.getNumber("Motion Profile I", 0), SmartDashboard.getNumber("Motion Profile D", 0.03), 1 / maxVelocity, 0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//double leftOutput = leftFollower.calculate(Robot.drivetrain.leftBottomMotor.getSensorCollection().getQuadraturePosition());
    	//double rightOutput = rightFollower.calculate(Robot.drivetrain.rightBottomMotor.getSensorCollection().getQuadraturePosition());
    	double leftOutput = leftFollower.calculate(Robot.drivetrain.leftBottomMotor.getSelectedSensorPosition(0));
    	double rightOutput = rightFollower.calculate(Robot.drivetrain.rightBottomMotor.getSelectedSensorPosition(0));
    	double gyroHeading = Robot.drivetrain.getGyroAngle();
    	double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
    	double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
    	double turn = 0.8 * (-1.0 / 80.0) * angleDifference;
    	leftPower = leftOutput + turn; 
    	System.out.println("Left Power: " + leftPower);
    	rightPower = rightOutput - turn; 
    	System.out.println("Right Power: " + rightPower);
    	Robot.drivetrain.leftBottomMotor.set(leftPower);
    	Robot.drivetrain.rightBottomMotor.set(rightPower);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if((leftPower > -0.01 && leftPower < 0.01) && (rightPower > -0.01 && rightPower < 0.01)) {
        	System.out.println("Ending Motion Profile");
        	return true; 
        }else {
        	return false; 
        }
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}

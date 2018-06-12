package org.usfirst.frc.team694.robot.commands;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;

import org.usfirst.frc.team694.robot.Robot;
import org.usfirst.frc.team694.robot.RobotMap;
import org.usfirst.frc.team694.util.PathGenerator;

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
    public DrivetrainMotionProfileJaciCommand(String nameOfPath, double maxVelocity) {
    	requires(Robot.drivetrain);
    	leftCSV = new File("/home/lvuser/" + nameOfPath + "_left_Jaci.csv");
    	rightCSV = new File("/home/lvuser/" + nameOfPath + "_right_Jaci.csv");
    	try {
    		Files.copy(getClass().getResource(nameOfPath + "_left_Jaci.csv").openStream(),  leftCSV.toPath(), StandardCopyOption.REPLACE_EXISTING);
    		Files.copy(getClass().getResource(nameOfPath + "_right_Jaci.csv").openStream(), rightCSV.toPath(), StandardCopyOption.REPLACE_EXISTING);
    		leftTraj = Pathfinder.readFromCSV(leftCSV);
        	rightTraj = Pathfinder.readFromCSV(rightCSV);
        	System.out.println("CSV has been locked and loaded");
    	}catch(IOException i) {
    		System.out.println("Invalid Trajectory, Aborting");
    		cancel(); 
    	}catch(Exception e) {
    		System.err.println(e);
    	}
    	this.maxVelocity = maxVelocity; 
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    public DrivetrainMotionProfileJaciCommand(PathGenerator path) {
    	leftTraj = path.modifier.getLeftTrajectory(); 
    	rightTraj = path.modifier.getRightTrajectory();
    	maxVelocity = path.maxVelocity;
    }
    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrain.resetEncoders(); 
    	Robot.drivetrain.resetGyro();
    	leftFollower.reset();
    	rightFollower.reset();
    	leftFollower = new EncoderFollower(leftTraj);
    	rightFollower = new EncoderFollower(rightTraj);
    	//Wheel diameter in feet
    	leftFollower.configureEncoder(Robot.drivetrain.leftBottomMotor.getSensorCollection().getQuadraturePosition(), RobotMap.DRIVETRAIN_ENCODERS_PULSES_PER_REVOLUTION, RobotMap.DRIVETRAIN_WHEEL_DIAMETER / 12);
    	rightFollower.configureEncoder(Robot.drivetrain.rightBottomMotor.getSensorCollection().getQuadraturePosition(), RobotMap.DRIVETRAIN_ENCODERS_PULSES_PER_REVOLUTION, RobotMap.DRIVETRAIN_WHEEL_DIAMETER / 12);
    	leftFollower.configurePIDVA(SmartDashboard.getNumber("Motion Profile P", 0.006), SmartDashboard.getNumber("Motion Profile I", 0), SmartDashboard.getNumber("Motion Profile D", 0.03), 1 / maxVelocity, 0);
    	rightFollower.configurePIDVA(SmartDashboard.getNumber("Motion Profile P", 0.006), SmartDashboard.getNumber("Motion Profile I", 0), SmartDashboard.getNumber("Motion Profile D", 0.03), 1 / maxVelocity, 0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double leftOutput = leftFollower.calculate(Robot.drivetrain.leftBottomMotor.getSensorCollection().getQuadraturePosition());
    	double rightOutput = rightFollower.calculate(Robot.drivetrain.rightBottomMotor.getSensorCollection().getQuadraturePosition());
    	double gyroHeading = Robot.drivetrain.getGyroAngle();
    	double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
    	double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
    	double turn = 0.8 * (-1.0 / 80.0) * angleDifference;
    	Robot.drivetrain.leftBottomMotor.set(leftOutput + turn);
    	Robot.drivetrain.rightBottomMotor.set(rightOutput - turn);
    	System.out.println("Left Power: " + (leftOutput + turn));
    	System.out.println("Right Power: " + (rightOutput - turn));
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(leftFollower.isFinished() && rightFollower.isFinished()){
    		System.out.println("Path has finished");
    		return true; 
    	}else {
    		return false; 
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.leftBottomMotor.set(0);
    	Robot.drivetrain.rightBottomMotor.set(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drivetrain.leftBottomMotor.set(0);
    	Robot.drivetrain.rightBottomMotor.set(0);
    }
}

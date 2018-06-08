package org.usfirst.frc.team694.robot.commands;

import java.io.File;

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
	
	//In ft/sec
	double maxVelocity; 
    public DrivetrainMotionProfileJaciCommand(String leftPoints, String rightPoints, double maxVelocity) {
    	requires(Robot.drivetrain);
    	leftCSV = new File(leftPoints);
    	rightCSV = new File(rightPoints);
    	//Pathfinder needs velocity in m/sec
    	this.maxVelocity = maxVelocity * 0.3048; 
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	leftTraj = Pathfinder.readFromCSV(leftCSV);
    	rightTraj = Pathfinder.readFromCSV(rightCSV);
    	leftFollower = new EncoderFollower(leftTraj);
    	rightFollower = new EncoderFollower(rightTraj);
    	//Wheel diameter in meters
    	leftFollower.configureEncoder(Robot.drivetrain.leftBottomMotor.getSensorCollection().getQuadraturePosition(), Robot.drivetrain.DRIVETRAIN_ENCODER_PULSES_PER_REVOLUTION, Robot.drivetrain.DRIVETRAIN_WHEEL_DIAMETER * 0.0254);
    	rightFollower.configureEncoder(Robot.drivetrain.rightBottomMotor.getSensorCollection().getQuadraturePosition(), Robot.drivetrain.DRIVETRAIN_ENCODER_PULSES_PER_REVOLUTION, Robot.drivetrain.DRIVETRAIN_WHEEL_DIAMETER * 0.0254);
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
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}

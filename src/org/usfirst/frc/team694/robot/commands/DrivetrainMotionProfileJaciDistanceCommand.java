package org.usfirst.frc.team694.robot.commands;

import java.io.File;

import org.usfirst.frc.team694.robot.Robot;
import org.usfirst.frc.team694.robot.RobotMap;
import org.usfirst.frc.team694.util.PathGenerator;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;

/**
 *
 */
public class DrivetrainMotionProfileJaciDistanceCommand extends Command {
	DistanceFollower leftFollower; 
	DistanceFollower rightFollower; 
	
	Trajectory leftTraj; 
	Trajectory rightTraj;
	
	File leftCSV; 
	File rightCSV; 
	
	Notifier profileProcessor;
	
    public DrivetrainMotionProfileJaciDistanceCommand(String nameOfPath, double dt, double maxVelocity) {
    	leftCSV = new File("/home/lvuser/Paths/" + nameOfPath + "_left_Jaci.csv");
    	rightCSV = new File("/home/lvuser/Paths/" + nameOfPath + "_right_Jaci.csv");
    	leftTraj = Pathfinder.readFromCSV(leftCSV);
    	rightTraj = Pathfinder.readFromCSV(rightCSV);
    	System.out.println("CSV has been locked and loaded");
    	profileProcessor = new Notifier(new RunProfile());
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }
    
    public DrivetrainMotionProfileJaciDistanceCommand(PathGenerator path) {
    	leftTraj = path.modifier.getLeftTrajectory();
    	rightTraj = path.modifier.getRightTrajectory();
    	profileProcessor = new Notifier(new RunProfile());
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	leftFollower = new DistanceFollower(leftTraj);
    	rightFollower = new DistanceFollower(rightTraj);
    	leftFollower.reset();
    	rightFollower.reset();
    	leftFollower.configurePIDVA(SmartDashboard.getNumber("kp", 0.0), SmartDashboard.getNumber("ki", 0.0), SmartDashboard.getNumber("kd", 0.0), RobotMap.kv, SmartDashboard.getNumber("ka", 0));
    	rightFollower.configurePIDVA(SmartDashboard.getNumber("kp", 0.0), SmartDashboard.getNumber("ki", 0.0), SmartDashboard.getNumber("kd", 0.0), RobotMap.kv, SmartDashboard.getNumber("ka", 0));
    	profileProcessor.startPeriodic(RobotMap.dt);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Code in Runnable 
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if(leftFollower.isFinished() && rightFollower.isFinished()) {
        	System.out.println("Path has finished");
        	return true; 
        }else {
        	return false; 
        }
    }

    // Called once after isFinished returns true
    protected void end() {
    	profileProcessor.stop(); 
    	Robot.drivetrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	profileProcessor.stop();
    	Robot.drivetrain.stop();
    }
  
  class RunProfile implements java.lang.Runnable {
	int segmentNumber = 0; 
	@Override
	public void run() {
		//Conversion to feet
    	double leftOutput = leftFollower.calculate(Robot.drivetrain.getLeftDistance() / 12);
    	double rightOutput =  rightFollower.calculate(Robot.drivetrain.getRightDistance() / 12);
    	double gyroHeading = Robot.drivetrain.getGyroAngle();
    	double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
    	//Pathfinder is counter-clockwise while gyro is clockwise so gyro heading is added 
    	double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading + gyroHeading);
    	double turn = 0.8 * (-1.0 * 80.0) * angleDifference;
    	Robot.drivetrain.tankDrive(leftOutput + turn, rightOutput - turn);
    	System.out.println(segmentNumber + "-Left Power: " + (leftOutput + turn) + "Right Power: " + (rightOutput - turn));
    	segmentNumber++; 
	}  
  }
}

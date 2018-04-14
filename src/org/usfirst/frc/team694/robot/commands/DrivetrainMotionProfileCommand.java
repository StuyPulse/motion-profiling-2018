package org.usfirst.frc.team694.robot.commands;

import org.usfirst.frc.team694.robot.Robot;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DrivetrainMotionProfileCommand extends Command {

	double[][] leftPoints;
	double[][] rightPoints;
	
    public double fGain;
    
    public int duration;
    
    public int profileSlot;
	
	MotionProfileStatus leftStatus;
	MotionProfileStatus rightStatus;
	
	SetValueMotionProfile setValue;
	
	public Notifier profileProcessor;
	
    public DrivetrainMotionProfileCommand(double[][] leftPoints, double[][] rightPoints) {
        requires(Robot.drivetrain);
        this.leftPoints = leftPoints;
        this.rightPoints = rightPoints;
        
        fGain = (Robot.drivetrain.MOTOR_OUTPUT * 1023)/Robot.drivetrain.VELOCITY;
        //Duration will be hard-coded! The default is 10, but can change. 
        duration = 10;
        
        profileSlot = 0; 
        
        leftStatus = new MotionProfileStatus();
        rightStatus = new MotionProfileStatus();
        
        setValue = SetValueMotionProfile.Disable;
        profileProcessor = new Notifier(new ProcessBuffer());
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setupMotionProfile(Robot.drivetrain.leftBottomMotor);
    	setupMotionProfile(Robot.drivetrain.rightBottomMotor);
    	setMotionProfile(setValue);
    	setGains(
    			profileSlot,
    			fGain,
    			SmartDashboard.getNumber("Motion Profile P", 0),
    			SmartDashboard.getNumber("Motion Profile I", 0),
    			SmartDashboard.getNumber("Motion Profile D", 0)
    			);
    	profileProcessor.startPeriodic(duration/1000);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	leftStatus = updateProfileStatus(Robot.drivetrain.leftBottomMotor);
    	rightStatus = updateProfileStatus(Robot.drivetrain.rightBottomMotor);
    	
    	if(leftStatus.hasUnderrun || rightStatus.hasUnderrun) {
    		setValue = SetValueMotionProfile.Disable;
    		System.out.println("Motors have underrun");
    	} else if(leftStatus.btmBufferCnt > 5 && rightStatus.btmBufferCnt > 5) {
    		setValue = SetValueMotionProfile.Enable; 
    		System.out.println("Path has started");
    	} else if(leftStatus.activePointValid && leftStatus.isLast && rightStatus.activePointValid && rightStatus.isLast) {
    		setValue = SetValueMotionProfile.Hold;
    		System.out.println("Path is up to the last point");
    	}
    	
    	setMotionProfile(setValue);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	leftStatus = updateProfileStatus(Robot.drivetrain.leftBottomMotor);
    	rightStatus = updateProfileStatus(Robot.drivetrain.rightBottomMotor);
    	if (leftStatus.isLast && rightStatus.isLast) {
    		System.out.println("Path has finished");
    	}
        return leftStatus.isLast && rightStatus.isLast;
    }

    // Called once after isFinished returns true
    protected void end() {
    	endMotionProfile();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	endMotionProfile();
    }
    public void setPercentOuput(double output) {
    	Robot.drivetrain.leftBottomMotor.set(ControlMode.PercentOutput, output);
    	Robot.drivetrain.rightBottomMotor.set(ControlMode.PercentOutput, output);
    }
   
    //mode can be Enable, Disable, or Hold
    public void setMotionProfile(SetValueMotionProfile mode) {
    	Robot.drivetrain.leftBottomMotor.set(ControlMode.MotionProfile, mode.value);
    	Robot.drivetrain.rightBottomMotor.set(ControlMode.MotionProfile, mode.value);
    }
    
    public void setupMotionProfile(WPI_TalonSRX talon){
    	clearMotionProfileUnderrun(talon);
    	talon.clearMotionProfileTrajectories();
    	talon.configMotionProfileTrajectoryPeriod(0, 0);
    	talon.changeMotionControlFramePeriod(duration/2);
    }

    public void endMotionProfile() {
    	profileProcessor.stop();
    	setMotionProfile(SetValueMotionProfile.Disable);
    	setPercentOuput(0.0);
    }
    
    public void setGains(int slot, double F, double P, double I, double D) {
    	Robot.drivetrain.leftBottomMotor.selectProfileSlot(slot, 0);
    	Robot.drivetrain.rightBottomMotor.selectProfileSlot(slot, 0);
    	
    	Robot.drivetrain.leftBottomMotor.config_kF(slot, F, 0);
    	Robot.drivetrain.leftBottomMotor.config_kP(slot, P, 0);
    	Robot.drivetrain.leftBottomMotor.config_kI(slot, I, 0);
    	Robot.drivetrain.leftBottomMotor.config_kD(slot, D, 0);
    	
    	Robot.drivetrain.rightBottomMotor.config_kF(slot, F, 0);
    	Robot.drivetrain.rightBottomMotor.config_kP(slot, P, 0);
    	Robot.drivetrain.rightBottomMotor.config_kI(slot, I, 0);
    	Robot.drivetrain.rightBottomMotor.config_kD(slot, D, 0);
    }
    
    public MotionProfileStatus updateProfileStatus(WPI_TalonSRX talon) {
    	MotionProfileStatus profileStatus = new MotionProfileStatus();
    	talon.getMotionProfileStatus(profileStatus);
    	return profileStatus;
    }
    
    public void clearMotionProfileUnderrun(WPI_TalonSRX talon) {
    	if(updateProfileStatus(talon).hasUnderrun) {
    		System.out.println(talon + "has underrun!!!!!!!!!!");
    		talon.clearMotionProfileHasUnderrun(0);
    	}
    }
    
    public int addTrajectoryPoints(WPI_TalonSRX talon, double[][] points, int slot, int lastPointSent) {
    	if (lastPointSent >= points.length){
    		return lastPointSent; 
    	}else if(lastPointSent < points.length){
    		TrajectoryPoint point = new TrajectoryPoint();
    		//Must be in native units!!!!!!!!!!
    		double position = points[lastPointSent][0];
    		double velocity = points[lastPointSent][1];
    		point.position = position; 
    		point.velocity = velocity; 
    		point.zeroPos = (lastPointSent==0) ? true:false; 
    		point.isLastPoint = (lastPointSent==points.length-1) ? true:false; 
    		point.profileSlotSelect0 = slot;
    		point.timeDur = getTrajectoryDuration((int) points[lastPointSent][2]);
    		talon.pushMotionProfileTrajectory(point);
    		lastPointSent++;
    	}
    	return lastPointSent; 
    }
    
    public TrajectoryDuration getTrajectoryDuration(int durationMs) {	 
		TrajectoryDuration retval = TrajectoryDuration.Trajectory_Duration_0ms;
		retval = retval.valueOf(durationMs);
		if (retval.value != durationMs) System.out.println("Time Duration not supported!!!!!!!!");
		return retval;
	}
    
    class ProcessBuffer implements java.lang.Runnable {
    	int leftLastPointSent = 0; 
    	int rightLastPointSent = 0;
    	@Override
    	public void run() {
    		addTrajectoryPoints(Robot.drivetrain.leftBottomMotor, leftPoints, profileSlot, leftLastPointSent);
    		addTrajectoryPoints(Robot.drivetrain.rightBottomMotor, rightPoints, profileSlot, rightLastPointSent);
    		Robot.drivetrain.leftBottomMotor.processMotionProfileBuffer();
    		Robot.drivetrain.rightBottomMotor.processMotionProfileBuffer();
    	}
    }
}

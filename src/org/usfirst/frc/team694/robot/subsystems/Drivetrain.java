package org.usfirst.frc.team694.robot.subsystems;

import org.usfirst.frc.team694.robot.Robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Drivetrain extends Subsystem {
	public WPI_VictorSPX leftTopMotor; 
	public WPI_VictorSPX rightTopMotor;
	public WPI_VictorSPX leftMiddleMotor; 
	public WPI_VictorSPX rightMiddleMotor; 
	public WPI_TalonSRX leftBottomMotor; 
	public WPI_TalonSRX rightBottomMotor; 
	
	private final int LEFT_TOP_MOTOR_PORT = 3;
	private final int LEFT_MIDDLE_MOTOR_PORT = 2; 
    private final int LEFT_BOTTOM_MOTOR_PORT = 1;
    private final int RIGHT_TOP_MOTOR_PORT = 7;
    private final int RIGHT_MIDDLE_MOTOR_PORT = 6; 
    private final int RIGHT_BOTTOM_MOTOR_PORT = 5; 

    // Must be in native units!!!!!!!!!!!!!!!!!!!!
    //TODO: Get the actual values
    //These will be constants in Robot Map in the future 
    public final double MOTOR_OUTPUT = 0;
    public final double VELOCITY = 0;
    public double fgain; 
    
    public AHRS navx;
    
    public final int DRIVETRAIN_WHEEL_DIAMETER = 6; 
    public final int DRIVETRAIN_ENCODER_PULSES_PER_REVOLUTION = 256;
    public final double DRIVETRAIN_ENCODER_INCHES_PER_REVOLUTION = Math.PI * DRIVETRAIN_WHEEL_DIAMETER;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public Drivetrain() {
    	leftTopMotor = new WPI_VictorSPX(LEFT_TOP_MOTOR_PORT);
    	rightTopMotor = new WPI_VictorSPX(RIGHT_TOP_MOTOR_PORT);
    	leftMiddleMotor = new WPI_VictorSPX(LEFT_MIDDLE_MOTOR_PORT);
    	rightMiddleMotor = new WPI_VictorSPX(RIGHT_MIDDLE_MOTOR_PORT);
    	leftBottomMotor = new WPI_TalonSRX(LEFT_BOTTOM_MOTOR_PORT);
    	rightBottomMotor = new WPI_TalonSRX(RIGHT_BOTTOM_MOTOR_PORT);
    
    	leftTopMotor.follow(leftBottomMotor);
    	leftMiddleMotor.follow(leftBottomMotor);
    	rightTopMotor.follow(rightBottomMotor);
    	rightMiddleMotor.follow(rightBottomMotor);
    	
    	rightTopMotor.setInverted(true);
    	rightMiddleMotor.setInverted(true);
    	rightBottomMotor.setInverted(true);
    	
    	fgain = (Robot.drivetrain.MOTOR_OUTPUT * 1023)/Robot.drivetrain.VELOCITY;
    	
    	navx = new AHRS(SPI.Port.kMXP);
    	
    	leftBottomMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    	rightBottomMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    }
    
    public void resetGyro(){
    	navx.reset();
    }
    
    public double getGyroAngle(){
    	return navx.getAngle();
    }
    
    public double getAcceleration(){
    	return Math.sqrt(Math.pow(navx.getWorldLinearAccelX(), 2) + Math.pow(navx.getWorldLinearAccelY(), 2)); 
    }
    
    public double getLeftDistance(){
    	return (leftBottomMotor.getSelectedSensorPosition(0) / DRIVETRAIN_ENCODER_PULSES_PER_REVOLUTION) * DRIVETRAIN_ENCODER_INCHES_PER_REVOLUTION;
    }
  
    public double getRightDistance(){
    	return -1 * (rightBottomMotor.getSelectedSensorPosition(0) / DRIVETRAIN_ENCODER_PULSES_PER_REVOLUTION) * DRIVETRAIN_ENCODER_INCHES_PER_REVOLUTION;
    }
    
    public double getDistance(){
    	return Math.max(getLeftDistance(), getRightDistance());
    }
    
    public double getLeftvelocity() {
    	return leftBottomMotor.getSelectedSensorVelocity(0);
    }
    
    public double getRightVelocity() {
    	return rightBottomMotor.getSelectedSensorVelocity(0);
    }
    
    public double getVelocity() {
    	return Math.max(getLeftvelocity(), getRightVelocity());
    }
    
    public void tankDrive(double l, double r) {
    	leftBottomMotor.set(l);
    	rightBottomMotor.set(r);
    }
}


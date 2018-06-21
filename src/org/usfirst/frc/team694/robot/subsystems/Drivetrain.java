package org.usfirst.frc.team694.robot.subsystems;

import org.usfirst.frc.team694.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    private final int RIGHT_TOP_MOTOR_PORT = 6;
    private final int RIGHT_MIDDLE_MOTOR_PORT = 5; 
    private final int RIGHT_BOTTOM_MOTOR_PORT = 4; 
    
    public AHRS navx;

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
    	
    	leftTopMotor.setInverted(true);
    	leftMiddleMotor.setInverted(true);
    	leftBottomMotor.setInverted(true);
    	
    	leftTopMotor.setNeutralMode(NeutralMode.Brake);
        leftMiddleMotor.setNeutralMode(NeutralMode.Brake);
        leftBottomMotor.setNeutralMode(NeutralMode.Brake);
        rightTopMotor.setNeutralMode(NeutralMode.Brake);
        rightMiddleMotor.setNeutralMode(NeutralMode.Brake);
        rightBottomMotor.setNeutralMode(NeutralMode.Brake);
    	
    	leftBottomMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    	rightBottomMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    	
    	navx = new AHRS(SPI.Port.kMXP);
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
    
    public void resetEncoders() {
    	leftBottomMotor.setSelectedSensorPosition(0, 0, 100);
    	rightBottomMotor.setSelectedSensorPosition(0, 0, 100);
    }
    
    public double getLeftDistance(){
    	return leftBottomMotor.getSelectedSensorPosition(0) * RobotMap.DRIVETRAIN_RAW_MULTIPLIER;
    }
  
    public double getRightDistance(){
    	return -1 * rightBottomMotor.getSelectedSensorPosition(0) * RobotMap.DRIVETRAIN_RAW_MULTIPLIER;
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
    
    public void stop() {
    	tankDrive(0, 0);
    }
}

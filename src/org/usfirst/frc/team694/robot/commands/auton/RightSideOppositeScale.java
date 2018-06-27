package org.usfirst.frc.team694.robot.commands.auton;

import org.usfirst.frc.team694.robot.Robot;
import org.usfirst.frc.team694.robot.commands.DrivetrainMotionProfileJaciEncoderCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightSideOppositeScale extends CommandGroup {

    public RightSideOppositeScale() {
    	//Use the robot
    	addSequential(new DrivetrainMotionProfileJaciEncoderCommand("RightSideOppositeScale", Robot.dt, 12));
    }
}

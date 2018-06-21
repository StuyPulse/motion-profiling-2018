package org.usfirst.frc.team694.robot.commands.auton;

import org.usfirst.frc.team694.robot.commands.DrivetrainMotionProfileJaciEncoderCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightSideOppositeScaleCommand extends CommandGroup {

    public RightSideOppositeScaleCommand() {
    	//Accel Gain: 0.014
    	//velocity parameter: 12
    	addSequential(new DrivetrainMotionProfileJaciEncoderCommand("RightSideOppositeScale", 12));
    }
}

package org.usfirst.frc.team694.robot.commands.auton;

import org.usfirst.frc.team694.robot.commands.DrivetrainMotionProfileJaciEncoderCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightSideOppositeScaleCommand extends CommandGroup {

    public RightSideOppositeScaleCommand() {
    	//Accel Gain: 0.014
    	//TODO: Give the csv a better name
    	addSequential(new DrivetrainMotionProfileJaciEncoderCommand("DifferentSideLeftScaleAuton", 29));
    }
}

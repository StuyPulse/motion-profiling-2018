package org.usfirst.frc.team694.robot.commands.auton;

import org.usfirst.frc.team694.robot.commands.DrivetrainMotionProfileJaciEncoderCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DifferentSideLeftScaleAutonCommand extends CommandGroup {

    public DifferentSideLeftScaleAutonCommand() {
    	//Accel Gain: 0.014
    	//velocity parameter: 12
    	addSequential(new DrivetrainMotionProfileJaciEncoderCommand("DifferentSideLeftScale", 29));
    }
}

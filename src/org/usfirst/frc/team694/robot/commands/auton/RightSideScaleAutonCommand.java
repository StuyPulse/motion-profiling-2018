package org.usfirst.frc.team694.robot.commands.auton;

import org.usfirst.frc.team694.robot.commands.DrivetrainMotionProfileCommand;
import org.usfirst.frc.team694.robot.motionprofiles.RightSideScaleAuton;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightSideScaleAutonCommand extends CommandGroup {

    public RightSideScaleAutonCommand() {
    	addSequential(new DrivetrainMotionProfileCommand(RightSideScaleAuton.leftPoints, RightSideScaleAuton.rightPoints));
    }
}

package org.usfirst.frc.team694.robot.commands.auton;

import org.usfirst.frc.team694.robot.commands.DrivetrainMotionProfileCommand;
import org.usfirst.frc.team694.robot.motionprofiles.RightSideScaleAutonProfile;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightSideScaleAutonCommand extends CommandGroup {

    public RightSideScaleAutonCommand() {
    	addSequential(new DrivetrainMotionProfileCommand(RightSideScaleAutonProfile.leftPoints, RightSideScaleAutonProfile.rightPoints));
    }
}

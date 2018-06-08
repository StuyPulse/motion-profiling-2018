package org.usfirst.frc.team694.robot.commands.auton;

import org.usfirst.frc.team694.robot.commands.DrivetrainMotionProfileTalonCommand;
import org.usfirst.frc.team694.robot.motionprofiles.RightSideScaleAutonProfileNormal;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightSideScaleAutonNormalCommand extends CommandGroup {

    public RightSideScaleAutonNormalCommand() {
    	addSequential(new DrivetrainMotionProfileTalonCommand(RightSideScaleAutonProfileNormal.leftPoints, RightSideScaleAutonProfileNormal.rightPoints));
    }
}

package org.usfirst.frc.team694.robot.commands.auton;

import org.usfirst.frc.team694.robot.commands.DrivetrainMotionProfileTalonCommand;
import org.usfirst.frc.team694.robot.motionprofiles.RightSideScaleAutonProfileTalonMod;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightSideScaleAutonTalonModCommand extends CommandGroup {

    public RightSideScaleAutonTalonModCommand() {
    	addSequential(new DrivetrainMotionProfileTalonCommand(RightSideScaleAutonProfileTalonMod.leftPoints, RightSideScaleAutonProfileTalonMod.rightPoints));
    }
}

package org.usfirst.frc.team694.robot.commands.auton;

import org.usfirst.frc.team694.robot.commands.DrivetrainMotionProfileJaciCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightSideScaleAutonJaciCommand extends CommandGroup {

    public RightSideScaleAutonJaciCommand() {
       addSequential(new DrivetrainMotionProfileJaciCommand("SameSideRightScaleAuton", 7.813));
    }
}

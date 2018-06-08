package org.usfirst.frc.team694.robot.commands.auton;

import org.usfirst.frc.team694.robot.commands.DrivetrainMotionProfileJaciCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightSideScaleAutonJaciCommand extends CommandGroup {

    public RightSideScaleAutonJaciCommand() {
       addSequential(new DrivetrainMotionProfileJaciCommand("src\\org\\usfirst\\frc\\team694\\robot\\motionprofiles\\SameSideRightScaleAuton_left_Jaci.csv", 
    		   "src\\org\\usfirst\\frc\\team694\\robot\\motionprofiles\\SameSideRightScaleAuton_right_Jaci.csv", 7.813));
    }
}

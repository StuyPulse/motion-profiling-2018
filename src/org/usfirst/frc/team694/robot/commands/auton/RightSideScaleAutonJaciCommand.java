package org.usfirst.frc.team694.robot.commands.auton;

import org.usfirst.frc.team694.robot.commands.DrivetrainMotionProfileJaciCommand;
//import org.usfirst.frc.team694.util.PathGenerator;

import edu.wpi.first.wpilibj.command.CommandGroup;
//import jaci.pathfinder.Pathfinder;
//import jaci.pathfinder.Waypoint;

/**
 *
 */
public class RightSideScaleAutonJaciCommand extends CommandGroup {
	/*Waypoint[] points = new Waypoint[] {
			new Waypoint(0, 3.25, 0),
			new Waypoint(10, 3.25, 0),
			new Waypoint(24, 3.25, 0),
			new Waypoint(27, 6, Pathfinder.d2r(90))
	};*/
    public RightSideScaleAutonJaciCommand() {
    	//With CSV
    	addSequential(new DrivetrainMotionProfileJaciCommand("SameSideRightScaleAuton", 7.813));
    	//By Generation
    	//addSequential(new DrivetrainMotionProfileJaciCommand(new PathGenerator(points, 0.01, 7.813, 6.104)));
    }
}

package org.usfirst.frc.team694.robot.commands.auton;

import org.usfirst.frc.team694.robot.commands.DrivetrainMotionProfileJaciEncoderCommand;
import org.usfirst.frc.team694.util.PathGenerator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Waypoint;

/**
 *
 */
public class ExampleMotionProfileCommand extends CommandGroup {
	Waypoint[] points = {
		//Your waypoints here
		//x, y, heading(in radians)
		//Use Pathfinder.d2r(degrees) to give headings in degrees
		new Waypoint(0, 0, 0) 
	}; 
    public ExampleMotionProfileCommand() {
    	//There are two ways to run a motion profile command
    	/*
    	 * Create a path using the path generator class provided in this project
    	 * Run a new DrivetrainMotionProfileEncoderCommand by giving it an instance of PathGenerator
    	 */
    	PathGenerator path = new PathGenerator(points, 0.05, 4, 3, 60); 
    	addSequential(new DrivetrainMotionProfileJaciEncoderCommand(path));
    	/*
    	 * Create pre calculated paths in csv format
    	 * Copy the _left.csv and _right.csv to home/lvuser/Paths on the roboRIO (_source.csv not needed) 
    	 * Make sure the left and right paths share the same name except for _left.csv/_right.csv part
    	 * Run a new DrivetrainMotionProfileEncoderCommand by giving it a string of the name of your path
    	 * Do not include the _left.csv or _right.csv portion
    	 */
    	addSequential(new DrivetrainMotionProfileJaciEncoderCommand("ExampleMotionProfilePath"));
    	//Note: DrivetrainMotionProfileJaciDistanceCommand can be implemented in the same way as DrivetrainMotionProfileEncoderCommand
    	//It has a slightly different algorithm, but should produce a similar result
    }
}

package org.usfirst.frc.team694.util;

import org.usfirst.frc.team694.robot.RobotMap;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class PathGenerator {
	public double dt; 
	
	public double maxVelocity; 
	public double maxAcceleration;
	
	public Waypoint[] points;
	
	public Trajectory.Config config; 
	
	public Trajectory traj; 
	
	public TankModifier modifier; 
	/*
	 * @param the waypoints of the path
	 * @param the time between each calculated point in milliseconds (1000 ms = 1 s)
	 * @param the max velocity in any unit, but make it consistent with max acceleration, and jerk
	 * @param the max acceleration in any unit, but make it consistent with max velocity, and jerk
	 * @param the max jerk in any unit, but make it consistent with max velocity, and acceleration (I recommend 60 ft/sec^3 or 18.288 m/sec^3)
	 */
	public PathGenerator(Waypoint[] points, double dt, double maxVelocity, double maxAcceleration, double maxJerk) {
		this.points = points; 
		this.config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, dt, maxVelocity, maxAcceleration, maxJerk);
		traj = Pathfinder.generate(this.points, config);
		//Wheel diameter in feet
		modifier = new TankModifier(traj).modify(RobotMap.DRIVETRAIN_WHEEL_DIAMETER / 12);
	}
}

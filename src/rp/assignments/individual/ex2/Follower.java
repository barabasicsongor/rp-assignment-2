package rp.assignments.individual.ex2;

import lejos.robotics.RangeFinder;
import lejos.util.Delay;
import rp.config.RangeFinderDescription;
import rp.robotics.DifferentialDriveRobot;
import rp.systems.StoppableRunnable;

public class Follower implements StoppableRunnable {
	
	private DifferentialDriveRobot robot;
	private RangeFinderDescription desc;
	private RangeFinder ranger;
	private float maxDistance;
	private boolean m_run;

	public Follower(DifferentialDriveRobot _robot, RangeFinderDescription _desc, RangeFinder _ranger, Float _maxDistance) {
		robot = _robot;
		desc = _desc;
		ranger = _ranger;
		maxDistance = _maxDistance;
		m_run = true;
	}

	@Override
	public void stop() {
		m_run = false;
	}
	
	@Override
	public void run() {
		
		float maxTravelSpeed = (float)robot.getDifferentialPilot().getMaxTravelSpeed();
		
		float minRange = desc.getMinRange();
		float maxRange = maxDistance;
		float setpoint = (maxRange + minRange)/2.0f;
		int delay = 10;
		
		float input, output=0, error;
		
		float kp = 0.45f;
		
		robot.getDifferentialPilot().forward();
		while(m_run) {
			
			input = ranger.getRange();
//			System.out.println(input);
			
			error = input - setpoint;
			
			output = kp*error*delay;
			output = Math.min(output, maxTravelSpeed);
			output = Math.max(output, 0);
			
			robot.getDifferentialPilot().setTravelSpeed(output);
			Delay.msDelay(delay);
		}
		
		robot.getDifferentialPilot().stop();
	}

}

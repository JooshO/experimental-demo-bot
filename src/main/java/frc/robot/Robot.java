// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.auton.Auton;
import frc.auton.AutonExample;
import frc.utils.Debug;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	
	// declare our systems object, teleop, and auton
	Systems m_systems;
	Teleop m_teleop;
	Auton m_auton;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// construct and init systems
		m_systems = new Systems();
		m_systems.init();

		// construct teleop and auton with systems
		m_teleop = new Teleop(m_systems);
		m_auton = new AutonExample(m_systems);

		// config debug mode
		Debug.setDebug(RobotMap.DEBUG_MODE);
	}

	@Override
	public void robotPeriodic() {
		m_systems.robotPeriodic();
	}

	@Override
	public void simulationPeriodic() {
		m_systems.simPeriodic();

		super.simulationPeriodic();
	}

	@Override
	public void autonomousInit() {
		m_auton.init();
	}

	@Override
	public void autonomousPeriodic() {
		m_auton.periodic();
	}

	@Override
	public void teleopInit() {
		m_teleop.init();
	}

	@Override
	public void teleopPeriodic() {
		m_teleop.periodic();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void testInit() {
		Debug.setDebug(true);
	}

	@Override
	public void testPeriodic() {
		// sample run of target drive for testing
		m_systems.getDrivetrain().targetDrive(2, 900);
	}
}

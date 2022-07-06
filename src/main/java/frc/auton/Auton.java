package frc.auton;

import frc.robot.Systems;

/**
 * Base class for auton, stores a system and declares init and periodic methods
 */
public abstract class Auton {
    /** The systems object that contains all needed objects to control the robot */
    protected Systems m_systems;

    /**
     * Constructor for abstract auton class (should only be called through super())
     * @param systems Systems needed to control the robot
     */
    public Auton(Systems systems) {
        this.m_systems = systems;
    }

    /** Set up for the autonomous period. This should be run in autonomousInit */
    public abstract void init();

    /**  Periodic method to control the robot autonomously. Should be run in autonomousPeriodic  */
    public abstract void periodic();
}

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.utils.Print;

public class Teleop {
    // declare systems, xbox controller
    private Systems m_systems;
    private XboxController m_controller;

    private SlewRateLimiter m_driveForwardLimiter; // value from 2022 was 0.5

    /** Class for storing class specific constants */
    private static class Constants {
        /** Port of the pilot controller */
        static final int kPilotControllerPort = 0;
        /** Deadband for the Pilot controller stick */
        static final double kPilotStickDeadband = 0.05;
    }

    /**
     * Contructor for Teleop object
     * @param systems Object containing all of our systems i.e. drivetrain, launcher
     */
    public Teleop(Systems systems) {
        this.m_systems = systems;
        this.m_controller = new XboxController(Constants.kPilotControllerPort);
    }

    /**
     * Should be called in teleop init
     */
    public void init() {
        Print.dPrintLn("TELEOP :: TELEOP INIT");
    }

    /**
     * Periodic method for controlling all systems.
     * Should be called in teleop periodic
     * TODO: implement deadband, forced ramp up, etc.
     */
    public void periodic() {
        // Sets the triggerInput variable equal to a number in [-1, 1] based on inputs from both triggers
        double triggerInput = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
        // Sets the leftStickXInput variable equal to a number in [-1, 1] based on inputs from the x-axis of the left joystick
        double leftStickXInput = adjustForDeadband(m_controller.getLeftX());

        m_systems.getDrivetrain().arcade(triggerInput, leftStickXInput);

        if (m_controller.getAButton()) {
            m_systems.getLauncher().setVelocity(1500);
        } else if (m_controller.getAButtonReleased()) {
            m_systems.getLauncher().setSpeed(0);
        }
    }

    
    /**
     * Take in input from a stick with drift, apply a deadband and then scale the input to remove a jump
     * @param stickInput The direct input from the joystick
     * @return the adjusted value for the deadband
     */
    public double adjustForDeadband(double stickInput) {
        //grab the absolute value of the stick input to reduce comparisons
        double absoluteStickInput = Math.abs(stickInput);

        //if our absolute stick input is withing our deadband, we set it equal to zero and early exit
        if (absoluteStickInput < Constants.kPilotStickDeadband) {
            return 0;
        }
        //otherwise, if we're outside of the deadband
        else {
            //reduce the input of the stick by the deadband to center the output on zero to prevent jumps
            absoluteStickInput -= Constants.kPilotStickDeadband;

            //then we assign the original sign to the modified input
            stickInput = Math.copySign(absoluteStickInput, stickInput);

            //then we output the stick input scaled to cover the whole range of values from 0 to 1
            return stickInput / (1.0 - Constants.kPilotStickDeadband);
        }
    }

}

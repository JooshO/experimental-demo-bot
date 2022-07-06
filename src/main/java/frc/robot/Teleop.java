package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.utils.Print;

public class Teleop {
    // declare systems, xbox controller
    private Systems m_systems;
    private XboxController m_controller;

    /** Class for storing class specific constants */
    private static class Constants {
        /** Port of the pilot controller */
        static final int kPilotControllerPort = 0;
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
     * TODO: implement deadband, forced ramp up
     */
    public void periodic() {
        // Sets the triggerInput variable equal to a number between 0 and 1 based on inputs from bother triggers and scales those inputs to the scaler that we enter on the shuffleboard
        double triggerInput = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
        //Sets the leftStickXInput variable equal to a number between 0 and 1 based on inputs from the x-axis of the left joystick and scales those inputs to the scaler we enter on the shuffleboard
        double leftStickXInput = m_controller.getLeftX();

        m_systems.getDrivetrain().arcade(triggerInput, leftStickXInput);

        if (m_controller.getAButton()) {
            m_systems.getLauncher().setVelocity(1500);
        } else if (m_controller.getAButtonReleased()) {
            m_systems.getLauncher().setSpeed(0);
        }
    }

}

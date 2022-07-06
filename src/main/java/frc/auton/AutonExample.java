package frc.auton;

import frc.robot.Drivetrain;
import frc.robot.Systems;
import frc.utils.Print;

/**
 * Example auton using AutonSteps to chain together a potentially quite long
 * auton consisting of mainly driving in a concise manner.
 */
public class AutonExample extends Auton {

    /**
     * Constructor for the Example Auton. Just calls the super constructor
     * @param systems Systems needed to control the robot
     */
    public AutonExample(Systems systems) {
        super(systems);
    }

    // tracks our current auto step. Default to step 1
    AutonStep autonStep = AutonStep.kStep1;

    // variables for tracking distance and heading.
    double endOfStepLeftPos = 0;
    double endOfStepYaw = 0;

    // types of auton steps, anything other than "other" should be pretty much just driving.
    // you can add additional functionality to simple steps by using the switch case at the bottom of periodic,
    // but if you want step progession to be determined by something other than distance or rotation, use other
    // end should always be your last step
    enum AutonStepType {
        kLinearDrive,
        kRotate,
        kEnd,
        kOther
    }

    
    enum AutonStep {
        kStep1(AutonStepType.kLinearDrive, 4, 0.4),
        kStep2(AutonStepType.kRotate, 30, 0.3),
        kStep3(AutonStepType.kLinearDrive, 2, 0.2),
        kStep4(AutonStepType.kRotate, -30, -0.4),
        kStepFinal(AutonStepType.kEnd, 0, 0);


        AutonStepType type;
        double delta; // degrees or meters
        double speed; // -1.0 to 1.0

        /**
         * Constructor for an auton step
         * @param type What type of step this is
         * @param delta What change in value you want. This would be measured in degrees for rotation or meters for distance
         * @param speed Percent speed you'd like the step to operate at, [-1, 1]
         */
        AutonStep (AutonStepType type, double delta, double speed) {
            this.type = type;
            this.delta = delta;
            this.speed = speed;
        }
    }

    @Override
    public void init() {
        // reset step counter, grab current positions
        autonStep = AutonStep.kStep1;

        // set starting values for auton
        endOfStepLeftPos = m_systems.getDrivetrain().getLeftSensorPosition();
        endOfStepYaw = m_systems.getDrivetrain().getYaw();
        
        Print.dPrintLn("AUTON :: AUTON INIT");
    }

    @Override
    public void periodic() {
        // if we are in the end step, 0 positions and reset end of step values in case of re-run
        if (autonStep.type == AutonStepType.kEnd) {
            m_systems.getDrivetrain().arcade(0, 0);
            endOfStepLeftPos = m_systems.getDrivetrain().getLeftSensorPosition();
            endOfStepYaw = m_systems.getDrivetrain().getYaw();
        }
        // if we are in a linear drive step, drive at a fixed speed until our left encoder has passed the position
        // this bypasses any issues with using the right sensor for a PID
        else if (autonStep.type == AutonStepType.kLinearDrive) {
            m_systems.getDrivetrain().arcade(autonStep.speed, 0);

            if (Math.abs(Drivetrain.nativeUnitsToDistanceMeters( m_systems.getDrivetrain().getLeftSensorPosition() - endOfStepLeftPos)) > Math.abs(autonStep.delta))
            {
                finishStep();
            }
        }
        // in a rotate step, rotate at a given speed until you hit the requested angle
        else if (autonStep.type == AutonStepType.kRotate) {
            m_systems.getDrivetrain().arcade(0, autonStep.speed);

            if (Math.abs(m_systems.getDrivetrain().getYaw() - endOfStepYaw) > Math.abs(autonStep.delta))
            {
                finishStep();
            }
        } 
        // otherwise, do something special determined in this switch case, defaulted to just finishing the step and moving on
        else {
            switch (autonStep) {
                default: finishStep();
            }
        }

    }

    /**
     * Called when finishing an auton step, ticks up our current step and grabs current position and yaw so the next step
     * can act relative to the end of the previous step.
     * Also, kill the drivetrain
     */
    private void finishStep() {
        // increment our step
        autonStep = AutonStep.values()[ autonStep.ordinal() + 1]; 
        
        // grab current position
        endOfStepLeftPos = m_systems.getDrivetrain().getLeftSensorPosition();
        endOfStepYaw = m_systems.getDrivetrain().getYaw();

        // stop drivetrain
        m_systems.getDrivetrain().arcade(0, 0);
    }

}
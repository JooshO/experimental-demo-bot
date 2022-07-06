package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Single flywheel launcher system using a TalonSRX. Supports simulation and has a velocity FPID
 */
public class Launcher {

    /**
     * Constants storage class for launcher
     */
    private static class Constants {
        /** CAN ID for the launch motor */
        static final int kMotorChannel = 10;
        /** conversion constant for RPM to units per 100ms */
        static final int kRPMtoUnitsPer100ms = 4096 / 600;

        // tuned to 1500 rpm
        /** PID gains for velocity control */
        static final Gains gains = new Gains(0.03, 0.5, 0.005, 0, 1000, 1);

        // default value
        /** Feedback coefficient for the velocity controller */
        static final double feedbackCoef = 1;
    }

    // declarations for motor, simulation objects, and debug entry
    private WPI_TalonSRX m_launchMotor;
    private TalonSRXSimCollection m_launchSim;
    private FlywheelSim m_flywheelSim;
    private NetworkTableEntry m_rpmEntry;

    /**
     * Constructor for the Launcher object
     */
    public Launcher() {
        // construct motor and sims
        m_launchMotor = new WPI_TalonSRX(Constants.kMotorChannel);
        m_launchSim = m_launchMotor.getSimCollection();
        m_flywheelSim = new FlywheelSim(DCMotor.getCIM(1), 1, 0.001); // we should use sysid tool and linear system, not my random guess at moment of inertia

        // Get the default instance of NetworkTables
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // Get the table within that instance that contains the data.
        NetworkTable table = inst.getTable("datatable");

        // Get a reference to the entry I'll use for tracking RPM
        m_rpmEntry = table.getEntry("rpm");
    }

    /**
     * Method to be called to init systems and config motors
     */
    public void init() {
        m_launchMotor.configFactoryDefault();
        m_launchMotor.setNeutralMode(NeutralMode.Coast);
        m_launchMotor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.QuadEncoder);
        m_launchMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.PID_PRIMARY, RobotMap.TIMEOUT_MS);
        Constants.gains.setConstants(m_launchMotor, RobotMap.PID_PRIMARY);
        m_launchMotor.configSelectedFeedbackCoefficient(Constants.feedbackCoef);
        m_launchMotor.configClosedLoopPeriod(RobotMap.PID_PRIMARY, RobotMap.TIMEOUT_MS);
    }

    /**
     * Set velocity using velocity controlled PID
     * @param rpm Revolutions per minute
     */
    public void setVelocity(double rpm) {
        double targetVelocity = rpmToUnitsPer100ms(rpm);        // covert from rpm to native units
        m_launchMotor.set(ControlMode.Velocity, targetVelocity);
    }

    /**
     * Set the speed directly, open loop
     * @param speed Percent input, [-1, 1]
     */
    public void setSpeed(double speed) {
        m_launchMotor.set(speed);
    }

    /**
     * @param units Rotational velocity in revolutions per minute
     * @return Rotational velocity in units per 100ms
     */
    public double rpmToUnitsPer100ms(double rpm) {
        return rpm * Constants.kRPMtoUnitsPer100ms;
    }

    /**
     * @param units Rotational velocity in units per 100ms
     * @return Rotational velocity in revolutions per minute
     */
    public double unitsPer100msToRPM(double units) {
        return units / Constants.kRPMtoUnitsPer100ms;
    }

    /**
     * Call during sim periodic, uppdates sim motor values and flywheel spin
     */
    public void simPeriodic() {
        // grab simulation battery voltage
        m_launchSim.setBusVoltage(RobotController.getBatteryVoltage());

        // set flywheel sim input from the motor sim and update
        m_flywheelSim.setInput(m_launchSim.getMotorOutputLeadVoltage());
        m_flywheelSim.update(RobotMap.SIM_PERIOD_MS);

        // set quadrature velocity manually from the si,
        m_launchSim.setQuadratureVelocity((int)rpmToUnitsPer100ms(m_flywheelSim.getAngularVelocityRPM()));

        // update our tracking network table entry
        m_rpmEntry.setDouble(m_flywheelSim.getAngularVelocityRPM());
    }



}

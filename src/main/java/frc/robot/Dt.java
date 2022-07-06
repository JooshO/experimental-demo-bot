package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// REFERENCES
// https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/drivetrain-model.html
// https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/DifferentialDrive_Simulation/src/main/java/frc/robot/Robot.java
// https://docs.ctre-phoenix.com/en/stable/ch15a_Simulation.html 
// https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/
// https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/ccbc278d944dae78c73b342003e65138934a1112/Java%20Talon%20FX%20(Falcon%20500)/MotionMagic_AuxStraightIntegratedSensor/src/main/java/frc/robot/Robot.java#L329
// redo with real gains from real robot

/**
 * Single speed differential drivetrain using 2 sets of TalonFX
 */
public class Drivetrain implements AutoCloseable {
    
    /**
     * Collection of constants for use with this drivetrain class.
     * <p>
     * Note: I'm obviously indecisive about naming convention for constants here.
     * Tradition dictates UPPER_CASE for public static values, but these don't need to be public.
     * I've used a k prefix like CTRE does, and is standard in some places, but the required
     * Constants reference should be clear enough I think.
     */
    private static class Constants {
        /** CAN ID of the right Lead motor */
        static final int kRightLeaderID = 9;
        /** CAN ID of the right follow motor */
        static final int kRightFollowerID = 8;
        /** CAN ID of the left Lead motor */
        static final int kLeftLeaderID = 7;
        /** CAN ID of the left follow motor */
        static final int kLeftFollowerID = 6;
        /** Radius of drive wheel in inches */
        static final double kWheelRadiusIn = 3; // we have 6 inch diameter wheels
        /** Circumference of drive wheel in inches */
        static final double kWheelCircumferenceIn = 2 * kWheelRadiusIn * Math.PI;
        /** Gear ration in the form of output torque / input torque */
        static final double kGearRatio = 15;
        /** Encoder ticks per revolution of the motor shaft */
        static final int kTicksPerRev = 2048;
        /** Track width (inside of left wheel to inside of right wheel) in inches */
        static final double kTrackWidthIn = 27;
        /** Robot weight in pounds including bumpers, battery, etc. */
        static final double kRobotWeightLbs = 125;
        /** Whether the right leader motor is inverted */
        static final boolean kRightSideInverted = true;
        /** Whether the left leader motor is inverted */
        static final boolean kLeftSideInverted = false;
        /** Minimum seconds from idle to full speed in open loop (acts as a cap on acceleration) */
        static final double kOpenLoopRampS = 2;

        /** PID Gains for driving straight with motion magic */
        static final Gains driveGains = new Gains(0.24, 0.15, 0.002, .015, 500, 0.5);//.24f
        /** PID Gains for rotation in motion magic */
        static final Gains turnGains = new Gains(0.24, .3, 0, 0, 200, 1.0);

        /** Motion magic acceleration value */
        static final double kMotionMagicAccel = 4000;
        /** Motion magic target cruise velocity */
        static final double kMotionMagicCruise = 6000;

        /** 
         * scale the rotation values down to ~10 units per degree. 276152 is the average measured sum (each side added together) value for rotating the robot 360 degrees in the sim
         */
        static final double ktenthDegreesPerRot = 3600.0/276152;
    }

    // physical declarations
    private WPI_TalonFX m_leaderRightMotor;
    private WPI_TalonFX m_leaderLeftMotor;
    private WPI_TalonFX m_followRightMotor;
    private WPI_TalonFX m_followLeftMotor;

    // getters for leader motors, necessary for unit testing
    public WPI_TalonFX getLeftLeader() {return m_leaderLeftMotor;}
    public WPI_TalonFX getRightLeader() {return m_leaderRightMotor;}

    // Declares the NavX gyro
    private AHRS m_gyro;

    // sim declarations
    // we only have sim collections for the leaders bc we only care about their sensors
    private TalonFXSimCollection m_leaderRightSim;
    private TalonFXSimCollection m_leaderLeftSim;
    private DifferentialDrivetrainSim m_driveSim;

    // getter for simulation, necessary for unit testing
    public DifferentialDrivetrainSim getSim() {return m_driveSim;}

    // general odometry / data collection
    private Field2d m_field;
    DifferentialDriveOdometry m_odometry;

    // network table entries for debugging
    NetworkTableEntry driveErrorEntry;
    NetworkTableEntry selectedSensorPosEntry;

    /**
     * Constructor for the drivetrain
     */
    public Drivetrain() {
        //instantiate motors
        m_leaderRightMotor = new WPI_TalonFX(Constants.kRightLeaderID);
        m_leaderLeftMotor = new WPI_TalonFX(Constants.kLeftLeaderID);
        m_followRightMotor = new WPI_TalonFX(Constants.kRightFollowerID);
        m_followLeftMotor = new WPI_TalonFX(Constants.kLeftFollowerID);

        //instantiating the gyro
        m_gyro = new AHRS(SPI.Port.kMXP);

        // construct our sim drive train
        m_leaderRightSim = m_leaderRightMotor.getSimCollection();
        m_leaderLeftSim = m_leaderLeftMotor.getSimCollection();

        // 7.5 is my guess at mass moment of inertia about drivetrain center
        m_driveSim = new DifferentialDrivetrainSim(DCMotor.getFalcon500(2), Constants.kGearRatio, 7.5, Units.lbsToKilograms(Constants.kRobotWeightLbs), 
                                        Units.inchesToMeters(Constants.kWheelRadiusIn), Units.inchesToMeters(Constants.kTrackWidthIn), null);


        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
        // first value is x and second is y?
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), new Pose2d(1.5, 3, new Rotation2d()));
    }

    /**
     * Call on robot init
     */
    public void init() {
        //Get the default instance of NetworkTables that was created automatically
        //when your program starts
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        //Get the table within that instance that contains the data. 
        NetworkTable table = inst.getTable("datatable");

        // grab references to the fields I am creating and zero them so they have to show up
        // these are useful so that I don't have to parse print statements and I can graph stuff
        driveErrorEntry = table.getEntry("closedLoopError");
        selectedSensorPosEntry = table.getEntry("selectedSensorPos");
        driveErrorEntry.setDouble(0);
        selectedSensorPosEntry.setDouble(0);

        // wipe any previous config
        m_leaderLeftMotor.configFactoryDefault();
        m_leaderRightMotor.configFactoryDefault();
        m_followLeftMotor.configFactoryDefault();
        m_followRightMotor.configFactoryDefault();

        // set inverted for our motors
        m_leaderRightMotor.setInverted(Constants.kRightSideInverted);
        m_leaderLeftMotor.setInverted(Constants.kLeftSideInverted);

        // NOTE: in some years our gearboxes have the motors on the same side spin into each other.
        // if that's the case, this obviously won't work
        m_followRightMotor.follow(m_leaderRightMotor);
        m_followLeftMotor.follow(m_leaderLeftMotor);
        m_followRightMotor.setInverted(InvertType.FollowMaster);
        m_followLeftMotor.setInverted(InvertType.FollowMaster);

        m_leaderRightMotor.configOpenloopRamp(Constants.kOpenLoopRampS);
        m_leaderLeftMotor.configOpenloopRamp(Constants.kOpenLoopRampS);

        //#region PID Config
        // the region tag lets me collapse this section in vscode. Everything contained configures the motion magic PID
        // config left selected sensor, point right at that sensor for use in turning/distance PID
        m_leaderLeftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotMap.PID_PRIMARY, RobotMap.TIMEOUT_MS);
        m_leaderRightMotor.configRemoteFeedbackFilter(m_leaderLeftMotor.getDeviceID(), RemoteSensorSource.TalonFX_SelectedSensor, 0, RobotMap.TIMEOUT_MS);

        // This section averages the values of the left and right encoders as a measurement of distance to make the PID
        // distance traveled more accurate. It basically sets the primary right selected sensor to the middle of the robot. This
        // breaks odometry, however, as it places the right wheel track as being in the "center" of the robot, and messes up turn radii.
        // This in turn causes the robot in the sim to look like it turns centered on the right side.
        // My current best fix is to set the sensor position in odometry to be the reverse average if that makese sense. See periodic()
        
        // now that right has that sensor, set up the right's primary feedback device as the difference of the remote and integrated
        m_leaderRightMotor.configSensorTerm(SensorTerm.Diff0, TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
        m_leaderRightMotor.configSensorTerm(SensorTerm.Diff1, TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice());
        
        m_leaderRightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.SensorDifference, RobotMap.PID_PRIMARY, RobotMap.TIMEOUT_MS);
        m_leaderRightMotor.configSelectedFeedbackCoefficient(0.5, RobotMap.PID_PRIMARY, RobotMap.TIMEOUT_MS); // since we add the two values together, we need to halve the value

        // now working on the turn PID (
        // sum the two values, which, since one sensor is reverse, will cancel out. Values will be 2x encoder ticks per rotation:
        m_leaderRightMotor.configSensorTerm(SensorTerm.Sum0, TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
        m_leaderRightMotor.configSensorTerm(SensorTerm.Sum1, TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice());
        m_leaderRightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.SensorSum, RobotMap.PID_AUX, RobotMap.TIMEOUT_MS);
        m_leaderRightMotor.configAuxPIDPolarity(true, RobotMap.TIMEOUT_MS);

        // scale the sensor to be 10th degrees
        m_leaderRightMotor.configSelectedFeedbackCoefficient(Constants.ktenthDegreesPerRot, RobotMap.PID_AUX, RobotMap.TIMEOUT_MS);

        // TODO config neutral deadband here 

        // set motion magic specific values
        m_leaderRightMotor.configMotionAcceleration(Constants.kMotionMagicAccel);
        m_leaderRightMotor.configMotionCruiseVelocity(Constants.kMotionMagicCruise);

        // set PID constants (P, I, D, F, izone, closedlooppeakoutput)
        Constants.driveGains.setConstants(m_leaderRightMotor, RobotMap.PID_PRIMARY);
        Constants.turnGains.setConstants(m_leaderRightMotor, RobotMap.PID_AUX);

        // setting period to the sim period, when working with hardware you may want that faster
        m_leaderRightMotor.configClosedLoopPeriod(RobotMap.PID_PRIMARY, RobotMap.SIM_PERIOD_MS);
        m_leaderRightMotor.configClosedLoopPeriod(RobotMap.PID_AUX, RobotMap.SIM_PERIOD_MS);

        m_leaderRightMotor.selectProfileSlot(0, RobotMap.PID_PRIMARY);
        m_leaderRightMotor.selectProfileSlot(1, RobotMap.PID_AUX);
        //#endregion PID Config
    }

    /**
     * This should be called in robot periodic 
     * Updates odometry, useful for both sim and actual robot
     */
    public void periodic() {
        // pull the integrated sensor value out of the average
        double rightIntegratedSensorValue = (m_leaderRightMotor.getSelectedSensorPosition(RobotMap.PID_PRIMARY) * 2) - m_leaderLeftMotor.getSelectedSensorPosition();

        m_odometry.update(new Rotation2d(Units.degreesToRadians( -m_gyro.getYaw())),
                    nativeUnitsToDistanceMeters(m_leaderLeftMotor.getSelectedSensorPosition()),
                    nativeUnitsToDistanceMeters(rightIntegratedSensorValue));
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    /**
     * Basic arcade drive
     * @param throttle Forward/Backward movement, on [-1, 1] where positive is forward
     * @param turn Rotation, on [-1, 1] where positive is clockwise
     */
    public void arcade(double throttle, double turn) {
        m_leaderLeftMotor.set(ControlMode.PercentOutput, throttle, DemandType.ArbitraryFeedForward, turn);
        m_leaderRightMotor.set(ControlMode.PercentOutput, throttle, DemandType.ArbitraryFeedForward, -turn);
        
        // follow
        m_followRightMotor.follow(m_leaderRightMotor);
        m_followLeftMotor.follow(m_leaderLeftMotor);
    }

    /**
     * Drive to a certain distance and achieve a certain angle by the end of the drive
     * It's possible the angle won't quite work out but it will do its best
     * @param distance Distance in meters
     * @param angle absolute angle in 10th degrees
     */
    public void targetDrive(double distance, double angle) {
        m_leaderRightMotor.set(TalonFXControlMode.MotionMagic, distanceToNativeUnits(distance), DemandType.AuxPID, angle);
        m_leaderLeftMotor.follow(m_leaderRightMotor, FollowerType.AuxOutput1);
        
        // technically faster to follow like this by ~15ms apparently
        m_followRightMotor.follow(m_leaderRightMotor);
        m_followLeftMotor.follow(m_leaderRightMotor, FollowerType.AuxOutput1);

        // update network table values for monitoring
        driveErrorEntry.setDouble(m_leaderRightMotor.getClosedLoopError(0));
        selectedSensorPosEntry.setDouble(m_leaderRightMotor.getSelectedSensorPosition(0));
    }

    /**
     * Should be called in sim periodic, updates drive train sim and heading as well as values for the drive speed controllers
     */
    public void simPeriodic() {
        // update sim voltage from sim battery
        m_leaderLeftSim.setBusVoltage(RobotController.getBatteryVoltage());
        m_leaderRightSim.setBusVoltage(RobotController.getBatteryVoltage());

        // set drive train sim volts and update
        m_driveSim.setInputs(m_leaderLeftSim.getMotorOutputLeadVoltage(), -m_leaderRightSim.getMotorOutputLeadVoltage());
        m_driveSim.update(RobotMap.SIM_PERIOD_MS / 1000);
        
        // manually set integrated sensor positions based on drive train
        m_leaderLeftSim.setIntegratedSensorRawPosition(distanceToNativeUnits(m_driveSim.getLeftPositionMeters()));
        m_leaderRightSim.setIntegratedSensorRawPosition(distanceToNativeUnits(-m_driveSim.getRightPositionMeters()));
        m_leaderLeftSim.setIntegratedSensorVelocity(velocityToNativeUnits(m_driveSim.getLeftVelocityMetersPerSecond()));
        m_leaderRightSim.setIntegratedSensorVelocity(velocityToNativeUnits(-m_driveSim.getRightVelocityMetersPerSecond()));

        // manually update NavX with heading
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(-m_driveSim.getHeading().getDegrees());
    }
    
    @Override
    public void close() throws Exception {
        m_leaderLeftMotor.close();
        m_leaderRightMotor.close();
        m_followLeftMotor.close();
        m_followRightMotor.close();
        m_gyro.close();
    }

    /**
     * Returns value of selected sensor for left motor controller
     * @return Value of left selected sensor in encoder ticks
     */
    public double getLeftSensorPosition() {
        return m_leaderLeftMotor.getSelectedSensorPosition();
    }

    /**
     * Note: if a PID is configured using this sensor, this may not be an accurate number
     * @return Value of right selected sensor in encoder ticks
     */
    public double getRightSensorPosition() {
        return m_leaderRightMotor.getSelectedSensorPosition();
    }

    /** 
     * @return Yaw (rotation) from the gyro
     */
    public double getYaw() {
        return m_gyro.getYaw();
    }

    //
    //  CONVERSION METHODS
    //

    /**
     * @param positionMeters Distance in meters
     * @return Distance in encoder ticks
     */
    public static int distanceToNativeUnits(double positionMeters){
        double wheelRotations = positionMeters/Units.inchesToMeters(Constants.kWheelCircumferenceIn);
        double motorRotations = wheelRotations * Constants.kGearRatio;
        int sensorCounts = (int)(motorRotations * Constants.kTicksPerRev);
        return sensorCounts;
    }
    
    /** 
     * @param velocityMetersPerSecond Velocity in Meters/Second
     * @return Velocity in Encoder ticks per 100ms
     */
    public static int velocityToNativeUnits(double velocityMetersPerSecond){
        double wheelRotationsPerSecond = velocityMetersPerSecond/Units.inchesToMeters(Constants.kWheelCircumferenceIn);
        double motorRotationsPerSecond = wheelRotationsPerSecond * Constants.kGearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / 10;
        int sensorCountsPer100ms = (int)(motorRotationsPer100ms * Constants.kTicksPerRev);
        return sensorCountsPer100ms;
    }
    
    /** 
     * @param sensorCounts Distance in encoder ticks
     * @return Distance in meters
     */
    public static double nativeUnitsToDistanceMeters(double sensorCounts){
        double motorRotations = (double)sensorCounts / Constants.kTicksPerRev;
        double wheelRotations = motorRotations / Constants.kGearRatio;
        double positionMeters = wheelRotations * Units.inchesToMeters(Constants.kWheelCircumferenceIn);
        return positionMeters;
    }

}

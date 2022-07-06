package frc.robot;

/**
 * Class that contains all of our systems on the robot.
 * This acts as a solution to give both auton and the pilot/copilot
 * access to everything they need.
 */
public class Systems {
    /** Robot drivetrain */
    private Drivetrain m_drivetrain;
    /** Velocity controlled launcher */
    private Launcher m_launcher;

    /** Constructor for the systems object */
    public Systems() {
        m_drivetrain = new Drivetrain();
        m_launcher = new Launcher();
    }

    /**
     * Call during Robot Periodic, calls all periodic methods of systems
     */
    public void robotPeriodic() {
        m_drivetrain.periodic();
    }

    /**
     * Call during sim periodic, calls all sim periodic methods of systems
     */
    public void simPeriodic() {
        m_drivetrain.simPeriodic();
        m_launcher.simPeriodic();
    }

    /** 
     * Call during robot init, calls all system init methods
     */
    public void init() {
        m_drivetrain.init();
        m_launcher.init();
    }

    /** 
     * @return A reference to the Launcher
     */
    public Launcher getLauncher() {
        return m_launcher;
    }

    /** 
     * @return A reference to the Drivetrain
     */
    public Drivetrain getDrivetrain() {
        return m_drivetrain;
    }
}

package frc.robot;

/** Class for storing project wide constants */
public class RobotMap {
    /** Whether we start robot runs as debugging. This is just for my prints */
    public static final boolean DEBUG_MODE = true;

    /** Default timeout for CTRE Pid configs */
    public static final int TIMEOUT_MS = 50;

    /** Period for the simulation (update frequency, etc.) */
    public static final int SIM_PERIOD_MS = 20;

    /** Constant representing the Primary slot for a CTRE Pid (should always be 0) */
    public static final int PID_PRIMARY = 0;
    /** Constant representing the Auxillary or turning slot for a CTRE Pid (should always be 1) */
    public static final int PID_AUX = 1;
}

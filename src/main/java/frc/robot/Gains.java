package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Class for storing gains values to shorten constants classes just a little
 */
public class Gains {
    final double f, p, i, d, izone, peakOutput;

    /**
     * Construct a gains object
     * @param f Feed-forward Constant
     * @param p Proportional Constant
     * @param i Integral Constant
     * @param d Derivative Constant
     * @param izone Integral Zone Constant (don't accumulate error while absolute current error is greater than this)
     * @param peakOutput Closed loop peak output
     */
    public Gains(double f, double p, double i, double d, double izone, double peakOutput) {
        this.f = f;
        this.p = p;
        this.i = i;
        this.d = d;
        this.izone = izone;
        this.peakOutput = peakOutput;
    }

    /**
     * Helper to set PID constants on a CTRE motor controller
     * @param controller The controller to set constants on
     * @param pidSlot The PID slot to set the constants on
     * @return The first error code raised, if one is raised
     */
    public ErrorCode setConstants(WPI_TalonFX controller, int pidSlot) {
        ArrayList<ErrorCode> errorCodes = new ArrayList<>();

        errorCodes.add(controller.config_kF(pidSlot, f, RobotMap.TIMEOUT_MS));
        errorCodes.add(controller.config_kP(pidSlot, p, RobotMap.TIMEOUT_MS));
        errorCodes.add(controller.config_kI(pidSlot, i, RobotMap.TIMEOUT_MS));
        errorCodes.add(controller.config_kD(pidSlot, d, RobotMap.TIMEOUT_MS));

        errorCodes.add(controller.config_IntegralZone(pidSlot, izone, RobotMap.TIMEOUT_MS));
        errorCodes.add(controller.configClosedLoopPeakOutput(pidSlot, peakOutput, RobotMap.TIMEOUT_MS));

        // remove all error codes that aren't errors
        errorCodes.removeIf(e -> (e.equals(ErrorCode.OK)));

        // return OK if we have no errors left, otherwise return the first error
        if (errorCodes.isEmpty()) return ErrorCode.OK;
        else return errorCodes.get(0);
    } 

    /**
     * Helper to set PID constants on a CTRE motor controller
     * @param controller The controller to set constants on
     * @param pidSlot The PID slot to set the constants on
     * @return The first error code raised, if one is raised
     */
    public ErrorCode setConstants(WPI_TalonSRX controller, int pidSlot) {
        ArrayList<ErrorCode> errorCodes = new ArrayList<>();

        errorCodes.add(controller.config_kF(pidSlot, f, RobotMap.TIMEOUT_MS));
        errorCodes.add(controller.config_kP(pidSlot, p, RobotMap.TIMEOUT_MS));
        errorCodes.add(controller.config_kI(pidSlot, i, RobotMap.TIMEOUT_MS));
        errorCodes.add(controller.config_kD(pidSlot, d, RobotMap.TIMEOUT_MS));

        errorCodes.add(controller.config_IntegralZone(pidSlot, izone, RobotMap.TIMEOUT_MS));
        errorCodes.add(controller.configClosedLoopPeakOutput(pidSlot, peakOutput, RobotMap.TIMEOUT_MS));
        
        // remove all error codes that aren't errors
        errorCodes.removeIf(e -> (e.equals(ErrorCode.OK)));

        // return OK if we have no errors left, otherwise return the first error
        if (errorCodes.isEmpty()) return ErrorCode.OK;
        else return errorCodes.get(0);
    } 
    
}

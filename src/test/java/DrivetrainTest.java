import static org.junit.Assert.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.RobotController;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import frc.utils.Print;
import frc.robot.*;
import org.junit.*;



public class DrivetrainTest {
    public static final double DELTA = 1e-2;
    Drivetrain drivetrain;
    private TalonFXSimCollection m_leaderRightSim;
    private TalonFXSimCollection m_leaderLeftSim;
    private DifferentialDrivetrainSim m_driveSim;

    @Before
    public void setup() {
        assert HAL.initialize(500, 0);
        drivetrain = new Drivetrain();

        // construct our sim drive train
        m_leaderRightSim = drivetrain.getRightLeader().getSimCollection();
        m_leaderLeftSim = drivetrain.getLeftLeader().getSimCollection();

        m_driveSim = drivetrain.getSim();
    }

    public void shutdown() throws Exception {
        drivetrain.arcade(0, 0);
        drivetrain.close();
    }

    @Test
    public void notMovingWithNoInput() {
        simUpdate();


        assertEquals(0.0, m_driveSim.getLeftVelocityMetersPerSecond(), DELTA);
    }

    // @Test
    // not working
    public void straightEqualForward() {
        for (int i = 0; i < 100; i++) {
            drivetrain.arcade(0.5, 0);
            try {
            Thread.sleep(1);
            } catch (Exception ignored) {};
            simUpdate();
        }
        
        Print.dPrintLn("Left: " + m_leaderLeftSim.toString());
        Print.dPrintF("Left Speed: %.4f m/s, Right Speed %.4f m/s\n", m_driveSim.getLeftVelocityMetersPerSecond(), m_driveSim.getRightVelocityMetersPerSecond());
        assertTrue(m_driveSim.getLeftVelocityMetersPerSecond() > 0);
        assertTrue(m_driveSim.getRightVelocityMetersPerSecond() > 0);
        assertEquals(m_driveSim.getLeftVelocityMetersPerSecond(), m_driveSim.getRightVelocityMetersPerSecond(), DELTA);
    }

    private void simUpdate() {
        Print.dPrintF("Battery Voltage: %.4f\n", RobotController.getBatteryVoltage());
        m_leaderLeftSim.setBusVoltage(RobotController.getBatteryVoltage());
        m_leaderRightSim.setBusVoltage(RobotController.getBatteryVoltage());

        m_driveSim.setInputs(m_leaderLeftSim.getMotorOutputLeadVoltage(), -m_leaderRightSim.getMotorOutputLeadVoltage());
        m_driveSim.update(0.02);
        
        m_leaderLeftSim.setIntegratedSensorRawPosition(Drivetrain.distanceToNativeUnits(m_driveSim.getLeftPositionMeters()));
        m_leaderRightSim.setIntegratedSensorRawPosition(Drivetrain.distanceToNativeUnits(-m_driveSim.getRightPositionMeters()));
        m_leaderLeftSim.setIntegratedSensorVelocity(Drivetrain.velocityToNativeUnits(m_driveSim.getLeftVelocityMetersPerSecond()));
        m_leaderRightSim.setIntegratedSensorVelocity(Drivetrain.velocityToNativeUnits(-m_driveSim.getRightVelocityMetersPerSecond()));
    }
}

package frc.robot.subsystems.SwerveModuleIO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {

    // Physics models (Neo = Drive, Neo550/Neo = Steer)
    private final DCMotorSim m_driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, 6.75), 
            DCMotor.getNEO(1));
            
    private final DCMotorSim m_steerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.005, 12.8), 
            DCMotor.getNEO(1));

    // Sim PID Controllers to mimic SparkMax Closed-Loop
    private final PIDController m_drivePID = new PIDController(0.1, 0.0, 0.0);
    private final PIDController m_steerPID = new PIDController(0.05, 0.0, 0.0);

    private double m_driveAppliedVolts = 0.0;
    private double m_steerAppliedVolts = 0.0;

    private double m_driveTargetVelocityMetersPerSec = 0.0;
    private double m_steerTargetAngleDeg = 0.0;

    // Simulation states
    private double m_simAbsoluteEncoderPositionRot = 0.0;
    private double m_steerRelativePositionDeg = 0.0;
    private boolean m_isOpenLoop = false;

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // 1. Calculate Closed-Loop Voltages (if not running open-loop for SysId)
        if (!m_isOpenLoop) {
            // Drive PID
            double currentVelocityMps = m_driveSim.getAngularVelocityRPM() / 60.0 * (Units.inchesToMeters(4.0) * Math.PI) / 6.75; // Approx conversion
            m_driveAppliedVolts = m_drivePID.calculate(currentVelocityMps, m_driveTargetVelocityMetersPerSec);
            
            // Steer PID
            m_steerAppliedVolts = m_steerPID.calculate(m_steerRelativePositionDeg, m_steerTargetAngleDeg);
        }

        // Clamp voltages to 12V battery limits
        m_driveAppliedVolts = Math.max(-12.0, Math.min(12.0, m_driveAppliedVolts));
        m_steerAppliedVolts = Math.max(-12.0, Math.min(12.0, m_steerAppliedVolts));

        m_driveSim.setInputVoltage(m_driveAppliedVolts);
        m_steerSim.setInputVoltage(m_steerAppliedVolts);

        // 2. Advance simulation by 20ms
        m_driveSim.update(0.020);
        m_steerSim.update(0.020);

        // 3. Update Steer Math
        m_steerRelativePositionDeg += Math.toDegrees(m_steerSim.getAngularVelocityRadPerSec() * 0.020);
        m_simAbsoluteEncoderPositionRot = (m_steerRelativePositionDeg % 360.0) / 360.0;
        if (m_simAbsoluteEncoderPositionRot < 0) m_simAbsoluteEncoderPositionRot += 1.0;

        // 4. Fill AdvantageKit Inputs
        inputs.drivePositionMeters = m_driveSim.getAngularPositionRad() * 0.05; // Mock radius multiplier
        inputs.drivePositionInches = Units.metersToInches(inputs.drivePositionMeters);
        inputs.driveVelocityMetersPerSec = m_driveSim.getAngularVelocityRPM() / 60.0 * 0.1; // Mock conversion
        inputs.driveAppliedVolts = m_driveAppliedVolts;
        inputs.driveCurrentAmps = m_driveSim.getCurrentDrawAmps();

        inputs.steerAbsolutePositionRot = m_simAbsoluteEncoderPositionRot;
        inputs.steerRelativePositionDeg = m_steerRelativePositionDeg;
        inputs.steerAppliedVolts = m_steerAppliedVolts;
        inputs.steerCurrentAmps = m_steerSim.getCurrentDrawAmps();
    }

    @Override
    public void setDriveVelocity(double speedMetersPerSec) {
        m_isOpenLoop = false;
        m_driveTargetVelocityMetersPerSec = speedMetersPerSec;
    }

    @Override
    public void setDrivePosition(double position) {
        // Not typically used for swerve drive motors, but required by interface
    }

    @Override
    public void setSteerAngle(double angleDeg) {
        m_steerTargetAngleDeg = angleDeg;
    }

    @Override
    public void setDriveVoltage(double volts) {
        m_isOpenLoop = true;
        m_driveAppliedVolts = volts;
    }

    @Override
    public void setSteerEncoderPosition(double positionDeg) {
        m_steerRelativePositionDeg = positionDeg;
    }

    @Override
    public void stop() {
        m_isOpenLoop = false;
        m_driveTargetVelocityMetersPerSec = 0.0;
        m_driveAppliedVolts = 0.0;
        m_steerAppliedVolts = 0.0;
    }
}
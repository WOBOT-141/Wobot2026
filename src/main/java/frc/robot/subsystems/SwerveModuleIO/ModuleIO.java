package frc.robot.subsystems.SwerveModuleIO;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    
    @AutoLog
    public static class ModuleIOInputs {
        // Drive Data
        public double drivePositionMeters = 0.0; // Used for SysId
        public double drivePositionInches = 0.0; 
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        // Steer Data
        public double steerAbsolutePositionRot = 0.0; // Raw CANcoder value
        public double steerRelativePositionDeg = 0.0; // SparkMax encoder value
        public double steerAppliedVolts = 0.0;
        public double steerCurrentAmps = 0.0;
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at a closed-loop velocity setpoint */
    public default void setDriveVelocity(double speedMetersPerSec) {}

    /** Run the drive motor to a closed-loop position setpoint */
    public default void setDrivePosition(double position) {}

    /** Run the steer motor to a closed-loop angle setpoint */
    public default void setSteerAngle(double angleDeg) {}

    /** Run the drive motor at an open-loop voltage (Required for SysId) */
    public default void setDriveVoltage(double volts) {}

    /** Override the internal SparkMax encoder position (Required for zeroing) */
    public default void setSteerEncoderPosition(double positionDeg) {}

    /** Stop all motors */
    public default void stop() {}
}
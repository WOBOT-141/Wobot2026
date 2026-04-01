package frc.robot.subsystems.GyroIO;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawRotation = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
        public double pitchDeg = 0.0;
        public double rollDeg = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void reset() {}
}
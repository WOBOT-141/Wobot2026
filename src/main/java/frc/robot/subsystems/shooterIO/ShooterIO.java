package frc.robot.subsystems.shooterIO;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        // Velocities
        public double leftVelocityIps = 0.0;
        public double rightVelocityIps = 0.0;
        public double middleVelocityIps = 0.0;
        
        // Hood
        public double hoodAngleDeg = 0.0;
        public double hoodCurrentAmps = 0.0;

        public double leftAppliedVolts = 0.0;
        public double rightAppliedVolts = 0.0;
        public double middleAppliedVolts = 0.0;
        public double hoodAppliedVolts = 0.0;
        
        public double[] motorTempsCelsius = new double[] {0.0, 0.0, 0.0, 0.0};
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setFlywheelVelocityIps(double velocity) {}
    
    public default void setHoodAngle(double angleDeg) {}
    
    public default void stop() {}
}
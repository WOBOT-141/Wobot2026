package frc.robot.subsystems.GyroIO;

import com.studica.frc.AHRS; // Using your specific import from the snippet

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIORealNavX implements GyroIO {
    private final AHRS gyro;

    public GyroIORealNavX() {
        gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k100Hz);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        // WPILib standard is CCW positive
        inputs.yawRotation = Rotation2d.fromDegrees(gyro.getAngle());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyro.getRawGyroZ());
        inputs.pitchDeg = gyro.getPitch();
        inputs.rollDeg = gyro.getRoll();
    }

    @Override
    public void reset() {
        gyro.reset();
    }
}
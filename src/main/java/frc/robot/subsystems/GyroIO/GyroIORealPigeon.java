package frc.robot.subsystems.GyroIO;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

// Corrected imports for Units
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond; 

public class GyroIORealPigeon implements GyroIO {
    private final Pigeon2 pigeon2;

    public GyroIORealPigeon(int id) {
        // Use the modern constructor (defaults to the RIO's native CAN bus)
        pigeon2 = new Pigeon2(id);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true; 

        // Position uses Degrees
        double yawDeg = pigeon2.getYaw().getValue().in(Degrees);
        inputs.yawRotation = Rotation2d.fromDegrees(yawDeg * -1);

        // Velocity MUST use DegreesPerSecond (or RadiansPerSecond)
        // This was the source of your "Incompatible Types" error
        double yawVelDegPerSec = pigeon2.getAngularVelocityZWorld().getValue().in(DegreesPerSecond);
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelDegPerSec * -1);

        inputs.pitchDeg = pigeon2.getPitch().getValue().in(Degrees) * -1;
        inputs.rollDeg = pigeon2.getRoll().getValue().in(Degrees) * -1;
    }

    @Override
    public void reset() {
        pigeon2.reset();
    }
}
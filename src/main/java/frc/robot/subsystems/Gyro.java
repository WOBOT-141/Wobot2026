package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.GyroIO.GyroIO;
import frc.robot.subsystems.GyroIO.GyroIOInputsAutoLogged;

/**
 * The Bridge: This subsystem implements your existing IGyro interface
 * but fetches all its data from the AdvantageKit IO layer.
 */
public class Gyro extends SubsystemBase implements IGyro {
    private final GyroIO io;
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    private double gyroOffset = 0;

    public Gyro(GyroIO io) {
        this.io = io;
    }


    @Override
    public void ResetGyro() {
        io.reset();
        gyroOffset = 0;
    }

    @Override
    public void setGyroOffset(double offset) {
        this.gyroOffset = offset;
    }

    @Override
    public double getGyroOffset() {
        return gyroOffset;
    }

    @Override
    public double getRawGyroAngle() {
        return inputs.yawRotation.getDegrees();
    }

    @Override
    public double getRealGyroAngle() {
        double angle = (getRawGyroAngle() + gyroOffset) % 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    @Override
    public double getPitch() { return inputs.pitchDeg; }

    @Override
    public double getYaw() { return inputs.yawRotation.getDegrees(); }

    @Override
    public double getRoll() { return inputs.rollDeg; }

    @Override
    public Rotation2d getRotationAs2d() {
        return inputs.yawRotation;
    }

    // --- AdvantageKit Periodic ---

    @Override
    public void periodic() {
        // 1. Update hardware inputs and log them
        io.updateInputs(inputs);
        Logger.processInputs("Gyro", inputs);

        // 2. Keep your existing dashboard outputs alive
        SmartDashboard.putNumber("Gyro/RawAngle", getRawGyroAngle());
        SmartDashboard.putNumber("Gyro/RealAngle", getRealGyroAngle());
        SmartDashboard.putBoolean("Gyro/Connected", inputs.connected);
    }
}
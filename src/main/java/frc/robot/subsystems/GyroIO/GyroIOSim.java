package frc.robot.subsystems.GyroIO;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
    private Rotation2d m_simYaw = new Rotation2d();
    private double m_simYawVelocityRadPerSec = 0.0;

    public GyroIOSim() {
        // Simulation starts connected and at 0
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawRotation = m_simYaw;
        inputs.yawVelocityRadPerSec = m_simYawVelocityRadPerSec;
        
        // Pitch and Roll usually stay flat in a basic 2D swerve sim
        inputs.pitchDeg = 0.0;
        inputs.rollDeg = 0.0;
    }

    @Override
    public void reset() {
        m_simYaw = new Rotation2d();
        m_simYawVelocityRadPerSec = 0.0;
    }

    /**
     * This is a "Sim-Only" method. 
     * The Drivetrain will call this every loop to update our virtual heading.
     */
    public void setSimYaw(Rotation2d yaw) {
        m_simYaw = yaw;
    }

    /**
     * Updates the simulated angular velocity.
     */
    public void setSimYawVelocity(double radPerSec) {
        m_simYawVelocityRadPerSec = radPerSec;
    }
}
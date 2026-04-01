package frc.robot.subsystems.shooterIO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
    
    // 1. Flywheel Physics Model (Using the LinearSystemId factory to fix compiler errors)
    private final FlywheelSim m_flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNEO(3), 0.005, 1.0),
        DCMotor.getNEO(3)
    );

    // 2. Hood Physics Model
    private final SingleJointedArmSim m_hoodSim = new SingleJointedArmSim(
        DCMotor.getNEO(1),
        ShooterConstants.simHoodGearing, 
        0.1, // Moment of inertia (jKgM2)
        0.2, // Arm length in meters
        Math.toRadians(ShooterConstants.hoodLowerLimit_deg),
        Math.toRadians(ShooterConstants.hoodUpperLimit_deg),
        false, // simulateGravity
        Math.toRadians(ShooterConstants.hoodLowerLimit_deg) // startingAngleRads
    );

    // 3. Sim Controllers and State
    // We need a PID controller to mock the SparkMax closed-loop controller for the hood
    private final PIDController m_hoodSimPID = new PIDController(10.0, 0.0, 0.0); 
    
    private double m_flywheelAppliedVolts = 0.0;
    private double m_hoodAppliedVolts = 0.0;
    private double m_hoodTargetAngleDeg = ShooterConstants.hoodLowerLimit_deg;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // 1. Calculate the voltage needed to move the hood to its target angle
        m_hoodAppliedVolts = m_hoodSimPID.calculate(
            Math.toDegrees(m_hoodSim.getAngleRads()), 
            m_hoodTargetAngleDeg
        );
        
        // Clamp voltages to realistically represent a 12V battery
        m_hoodAppliedVolts = Math.max(-12.0, Math.min(12.0, m_hoodAppliedVolts));
        m_flywheelAppliedVolts = Math.max(-12.0, Math.min(12.0, m_flywheelAppliedVolts));

        m_hoodSim.setInputVoltage(m_hoodAppliedVolts);
        m_flywheelSim.setInputVoltage(m_flywheelAppliedVolts);

        // 2. Advance the simulation by 20ms
        m_flywheelSim.update(0.020);
        m_hoodSim.update(0.020);

        // 3. Fill the AdvantageKit inputs
        double velocityIps = m_flywheelSim.getAngularVelocityRPM() * ShooterConstants.flywheelVelocity_RPMtoIPS;
        
        inputs.leftVelocityIps = velocityIps;
        inputs.rightVelocityIps = velocityIps;
        inputs.middleVelocityIps = velocityIps;
        
        inputs.hoodAngleDeg = Math.toDegrees(m_hoodSim.getAngleRads());
        inputs.hoodCurrentAmps = m_hoodSim.getCurrentDrawAmps();

        // Log the applied voltages (Helpful for debugging PID in sim)
        inputs.leftAppliedVolts = m_flywheelAppliedVolts;
        inputs.rightAppliedVolts = m_flywheelAppliedVolts;
        inputs.middleAppliedVolts = m_flywheelAppliedVolts;
        inputs.hoodAppliedVolts = m_hoodAppliedVolts;
    }

    @Override
    public void setFlywheelVelocityIps(double velocity) {
        // Simple feedforward mock: In reality, your subsystem PID would calculate this.
        // For basic testing, we just apply a voltage proportional to the requested speed.
        m_flywheelAppliedVolts = velocity * 0.05; 
    }

    @Override
    public void setHoodAngle(double angleDeg) {
        // We just update the setpoint here; the PID controller in updateInputs() does the math
        m_hoodTargetAngleDeg = angleDeg;
    }

    @Override
    public void stop() {
        m_flywheelAppliedVolts = 0.0;
        m_hoodTargetAngleDeg = Math.toDegrees(m_hoodSim.getAngleRads()); // Stop moving hood
    }
}
package frc.robot.subsystems.shooterIO;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.CANIdConstants;

public class ShooterIOReal implements ShooterIO {

    private final SparkMax m_rightMotor;
    private final SparkMax m_middleMotor;
    private final SparkMax m_leftMotor;
    private final SparkMax m_hoodMotor;

    private final RelativeEncoder m_rightEncoder;
    private final RelativeEncoder m_middleEncoder;
    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_hoodMotorEncoder;

    private final SparkClosedLoopController m_rightController;
    private final SparkClosedLoopController m_middleController;
    private final SparkClosedLoopController m_leftController;
    private final SparkClosedLoopController m_hoodController;

    private final SparkMaxConfig m_leftMotorConfig;
    private final SparkMaxConfig m_middleMotorConfig;
    private final SparkMaxConfig m_rightMotorConfig;
    private final SparkMaxConfig m_hoodMotorConfig;

    public ShooterIOReal() {
        m_rightMotor = new SparkMax(CANIdConstants.flyWheelRightMotorID, MotorType.kBrushless);
        m_middleMotor = new SparkMax(CANIdConstants.flyWheelMiddleMotorID, MotorType.kBrushless);
        m_leftMotor = new SparkMax(CANIdConstants.flyWheelLeftMotorID, MotorType.kBrushless);
        m_hoodMotor = new SparkMax(CANIdConstants.hoodMotorID, MotorType.kBrushless);

        m_rightMotorConfig = new SparkMaxConfig();
        m_middleMotorConfig = new SparkMaxConfig();
        m_leftMotorConfig = new SparkMaxConfig();

        m_rightMotorConfig.inverted(true);
        m_middleMotorConfig.inverted(false);
        m_leftMotorConfig.inverted(false);

        m_rightMotorConfig.encoder.velocityConversionFactor(ShooterConstants.flywheelVelocity_RPMtoIPS);
        m_middleMotorConfig.encoder.velocityConversionFactor(ShooterConstants.flywheelVelocity_RPMtoIPS);
        m_leftMotorConfig.encoder.velocityConversionFactor(ShooterConstants.flywheelVelocity_RPMtoIPS);

        // Note: Preserved your original mapping here, though double-check if leftP to rightMotorConfig is intentional!
        m_rightMotorConfig.closedLoop.pid(ShooterConstants.leftP, ShooterConstants.I, ShooterConstants.D).velocityFF(ShooterConstants.kF, ClosedLoopSlot.kSlot0);
        m_middleMotorConfig.closedLoop.pid(ShooterConstants.middleP, ShooterConstants.I, ShooterConstants.D).velocityFF(ShooterConstants.kF, ClosedLoopSlot.kSlot0);
        m_leftMotorConfig.closedLoop.pid(ShooterConstants.rightP, ShooterConstants.I, ShooterConstants.D).velocityFF(ShooterConstants.kF, ClosedLoopSlot.kSlot0);

        m_rightMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.flywheelCurrentLimit_amp, ShooterConstants.flywheelCurrentLimit_amp);
        m_middleMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.flywheelCurrentLimit_amp, ShooterConstants.flywheelCurrentLimit_amp);
        m_leftMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.flywheelCurrentLimit_amp, ShooterConstants.flywheelCurrentLimit_amp);

        m_hoodMotorConfig = new SparkMaxConfig();
        m_hoodMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ShooterConstants.hoodCurrentLim_amp)
            .openLoopRampRate(ShooterConstants.hoodSlewRate);
        m_hoodMotorConfig.encoder.positionConversionFactor(ShooterConstants.hoodDegPerMotorRot); 
        m_hoodMotorConfig.closedLoop
            .pid(ShooterConstants.hoodP, ShooterConstants.hoodI, ShooterConstants.hoodD)
            .minOutput(-0.3)
            .maxOutput(0.3);

        configureMotor(m_rightMotor, m_rightMotorConfig);
        configureMotor(m_middleMotor, m_middleMotorConfig);
        configureMotor(m_leftMotor, m_leftMotorConfig);
        configureMotor(m_hoodMotor, m_hoodMotorConfig);

        m_rightEncoder = m_rightMotor.getEncoder();
        m_middleEncoder = m_middleMotor.getEncoder();
        m_leftEncoder = m_leftMotor.getEncoder();
        m_hoodMotorEncoder = m_hoodMotor.getEncoder();

        m_rightController = m_rightMotor.getClosedLoopController();
        m_middleController = m_middleMotor.getClosedLoopController();
        m_leftController = m_leftMotor.getClosedLoopController();
        m_hoodController = m_hoodMotor.getClosedLoopController();
    }

    private void configureMotor(SparkMax motor, SparkMaxConfig config) {
        motor.configure(config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Velocities
        inputs.leftVelocityIps = m_leftEncoder.getVelocity();
        inputs.middleVelocityIps = m_middleEncoder.getVelocity();
        inputs.rightVelocityIps = m_rightEncoder.getVelocity();
        
        // Hood Position & Current
        inputs.hoodAngleDeg = m_hoodMotorEncoder.getPosition();
        inputs.hoodCurrentAmps = m_hoodMotor.getOutputCurrent();

        // Applied Voltages (Calculated by taking the fractional output [-1 to 1] and multiplying by bus voltage)
        inputs.leftAppliedVolts = m_leftMotor.getAppliedOutput() * m_leftMotor.getBusVoltage();
        inputs.middleAppliedVolts = m_middleMotor.getAppliedOutput() * m_middleMotor.getBusVoltage();
        inputs.rightAppliedVolts = m_rightMotor.getAppliedOutput() * m_rightMotor.getBusVoltage();
        inputs.hoodAppliedVolts = m_hoodMotor.getAppliedOutput() * m_hoodMotor.getBusVoltage();

        // Motor Temperatures
        inputs.motorTempsCelsius[0] = m_leftMotor.getMotorTemperature();
        inputs.motorTempsCelsius[1] = m_middleMotor.getMotorTemperature();
        inputs.motorTempsCelsius[2] = m_rightMotor.getMotorTemperature();
        inputs.motorTempsCelsius[3] = m_hoodMotor.getMotorTemperature();
    }

    @Override
    public void setFlywheelVelocityIps(double velocity) {
        m_leftController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        m_middleController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        m_rightController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setHoodAngle(double angleDeg) {
        m_hoodController.setSetpoint(angleDeg, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void stop() {
        m_leftMotor.stopMotor();
        m_middleMotor.stopMotor();
        m_rightMotor.stopMotor();
        m_hoodMotor.stopMotor();
    }
}
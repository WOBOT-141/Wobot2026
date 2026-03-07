package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;
import frc.robot.Constants.CANIdConstants;

public class Shooter extends SubsystemBase {

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
  private final SparkMaxConfig m_forwardMotorConfig;
  private final SparkMaxConfig m_backwardMotorConfig;
  private final SparkMaxConfig m_hoodMotorConfig;

  private double m_targetVelocity_ips = 0.0;

  // Shot detection
  private double m_lastAvgRPM_rpm = 0.0;
  private boolean m_shotDetected = false;
  private double m_lastShotTime_sec = -1.0;

  private boolean m_hoodAtPosition = false;
  private boolean m_hoodAtHome = false;
  private double m_hoodTargetPosition_deg = 0;

  public Shooter() {
    SmartDashboard.putNumber("Shooter P", ShooterConstants.P);
    SmartDashboard.putNumber("Shooter I", ShooterConstants.I);
    SmartDashboard.putNumber("Shooter D", ShooterConstants.D);

    SmartDashboard.putNumber("hood P", ShooterConstants.hoodP);
    SmartDashboard.putNumber("hood I", ShooterConstants.hoodI);
    SmartDashboard.putNumber("hood D", ShooterConstants.hoodD);

    m_rightMotor = new SparkMax(CANIdConstants.flyWheelRightMotorID, MotorType.kBrushless);
    m_middleMotor = new SparkMax(CANIdConstants.flyWheelMiddlehtMotorID, MotorType.kBrushless);
    m_leftMotor = new SparkMax(CANIdConstants.flyWheelLeftMotorID, MotorType.kBrushless);
    m_hoodMotor = new SparkMax(CANIdConstants.hoodMotorID, MotorType.kBrushless);

    m_forwardMotorConfig = new SparkMaxConfig();

    m_forwardMotorConfig.idleMode(IdleMode.kCoast);
    m_forwardMotorConfig.inverted(true);

    m_forwardMotorConfig.closedLoop
        .pid(ShooterConstants.P,
            ShooterConstants.I,
            ShooterConstants.D)
        .velocityFF(ShooterConstants.kF, ClosedLoopSlot.kSlot0);

    m_backwardMotorConfig = new SparkMaxConfig();

    m_backwardMotorConfig.idleMode(IdleMode.kCoast);
    m_backwardMotorConfig.inverted(true);

    m_backwardMotorConfig.closedLoop
        .pid(ShooterConstants.P,
            ShooterConstants.I,
            ShooterConstants.D)
        .velocityFF(ShooterConstants.kF, ClosedLoopSlot.kSlot0);

    m_hoodMotorConfig = new SparkMaxConfig();

    m_hoodMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit( 20, 20);

    m_hoodMotorConfig.encoder.positionConversionFactor(1/ ShooterConstants.hoodRevsPerDegree);

    m_hoodMotorConfig.closedLoop
      .pid(ShooterConstants.hoodP, ShooterConstants.hoodI, ShooterConstants.hoodD)
            .velocityFF(ShooterConstants.hoodFF, ClosedLoopSlot.kSlot0);

    m_hoodMotor.configure(m_hoodMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    configureMotor(m_rightMotor, m_forwardMotorConfig);
    configureMotor(m_middleMotor, m_forwardMotorConfig);
    configureMotor(m_leftMotor, m_forwardMotorConfig);
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

  public void homeHood() {
    m_hoodMotor.set(Constants.ShooterConstants.hoodHomeRPM);
    if (m_hoodMotor.getOutputCurrent() > Constants.ShooterConstants.hoodHomeCurrentLim_amp) {
      m_hoodMotor.stopMotor();
      m_hoodMotorEncoder.setPosition(0);
      m_hoodAtHome = true;
    }
  }

  public boolean hoodAtHome() {
    return m_hoodAtHome;
  }

  public void setHoodPosition(double targetPosition_deg) {
    targetPosition_deg = SmartDashboard.getNumber("hoodPosition", targetPosition_deg);
    double targetPositionTicks = targetPosition_deg / ShooterConstants.steerDegPerEncoderTick_deg;
    m_hoodController.setSetpoint(targetPositionTicks, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setPIDValues(double flywheelP, double flywheelI, double flywheelD) {

    m_forwardMotorConfig.closedLoop
        .pid(flywheelP, flywheelI, flywheelD);

    m_backwardMotorConfig.closedLoop
        .pid(flywheelP, flywheelI, flywheelD);

    configureMotor(m_rightMotor, m_backwardMotorConfig);
    configureMotor(m_middleMotor, m_backwardMotorConfig);
    configureMotor(m_leftMotor, m_forwardMotorConfig);

  }

  private void configureMotor(SparkMax motor, SparkMaxConfig config) {

    motor.configure(config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setShooter_ips(double target_ips) {

    m_targetVelocity_ips = target_ips;

    m_rightController.setSetpoint(m_targetVelocity_ips, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_middleController.setSetpoint(m_targetVelocity_ips, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_leftController.setSetpoint(m_targetVelocity_ips, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  // Speed checks
  public boolean rightAtSpeed() {
    return Math.abs(m_rightEncoder.getVelocity() - m_targetVelocity_ips) < ShooterConstants.FlywheelRPMTolerance_rpm;
  }

  public boolean middleAtSpeed() {
    return Math.abs(m_middleEncoder.getVelocity() - m_targetVelocity_ips) < ShooterConstants.FlywheelRPMTolerance_rpm;
  }

  public boolean leftAtSpeed() {
    return Math.abs(m_leftEncoder.getVelocity() - m_targetVelocity_ips) < ShooterConstants.FlywheelRPMTolerance_rpm;
  }

  public boolean allAtSpeed() {
    return rightAtSpeed() && middleAtSpeed() && leftAtSpeed();
  }

  public double getAverageRPM() {
    return (m_rightEncoder.getVelocity()
        + m_middleEncoder.getVelocity()
        + m_leftEncoder.getVelocity()) / 3.0;
  }

  // Shot detection via RPM dip
  public boolean updateShotDetection() {

    double currentRPM_rpm = getAverageRPM();
    double now_sec = Timer.getFPGATimestamp();

    m_shotDetected = false;

    if ((m_lastAvgRPM_rpm - currentRPM_rpm) > ShooterConstants.shotDipThreshold_rpm) {

      if (m_lastShotTime_sec < 0 ||
          (now_sec - m_lastShotTime_sec) > ShooterConstants.shotMinSeparation_sec) {

        m_shotDetected = true;
        m_lastShotTime_sec = now_sec;
      }
    }

    m_lastAvgRPM_rpm = currentRPM_rpm;
    return m_shotDetected;
  }

  public double getLastShotTime_sec() {
    return m_lastShotTime_sec;
  }

  public void cancel() {
    m_rightMotor.stopMotor();
    m_middleMotor.stopMotor();
    m_leftMotor.stopMotor();
    m_hoodMotor.stopMotor();
  }

  @Override
  public void periodic() {

    updateShotDetection();

    SmartDashboard.putNumber("Shooter Avg RPM", getAverageRPM());
    SmartDashboard.putBoolean("Shot Detected", m_shotDetected);
  }
}

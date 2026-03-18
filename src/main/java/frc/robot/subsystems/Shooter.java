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

  private final SparkMaxConfig m_leftMotorConfig;
  private final SparkMaxConfig m_middleMotorConfig;
  private final SparkMaxConfig m_rightMotorConfig;
  private final SparkMaxConfig m_hoodMotorConfig;

  private double m_targetVelocity_ips = 0.0;

  private double m_hoodTargetPosition_deg = ShooterConstants.hoodLowerLimit_deg;
  private boolean m_hoodAtHome = false;
  private double m_hoodHomeStartTime_sec = -1.0;
  private double m_readyStartTime = -1;

  // Shot detection
  private double m_lastAvgVelocity_ips = 0.0;
  private boolean m_shotDetected = false;
  private double m_lastShotTime_sec = -1.0;

  private static final String HOOD_POSITION_KEY = "Set Hood Position";

  public Shooter() {

    SmartDashboard.putNumber("hoodP", ShooterConstants.hoodP);
    SmartDashboard.putNumber("hoodI", ShooterConstants.hoodI);
    SmartDashboard.putNumber("hoodD", ShooterConstants.hoodD);
    SmartDashboard.putNumber(HOOD_POSITION_KEY, m_hoodTargetPosition_deg);

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

    m_rightMotorConfig.closedLoop.pid(ShooterConstants.leftP, ShooterConstants.I, ShooterConstants.D)
        .velocityFF(ShooterConstants.kF, ClosedLoopSlot.kSlot0);
    m_middleMotorConfig.closedLoop.pid(ShooterConstants.middleP, ShooterConstants.I, ShooterConstants.D)
        .velocityFF(ShooterConstants.kF, ClosedLoopSlot.kSlot0);
    m_leftMotorConfig.closedLoop.pid(ShooterConstants.rightP, ShooterConstants.I, ShooterConstants.D)
        .velocityFF(ShooterConstants.kF, ClosedLoopSlot.kSlot0);

    m_rightMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.flywheelCurrentLimit_amp);
    m_middleMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.flywheelCurrentLimit_amp);
    m_leftMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.flywheelCurrentLimit_amp);

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
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void homeHood() {

    if (m_hoodAtHome)
      return;

    if (m_hoodHomeStartTime_sec < 0)
      m_hoodHomeStartTime_sec = Timer.getFPGATimestamp();

    double elapsed_sec = Timer.getFPGATimestamp() - m_hoodHomeStartTime_sec;

    m_hoodMotor.set(ShooterConstants.hoodHomeSpeed_perc);

    if (m_hoodMotor.getOutputCurrent() > ShooterConstants.hoodCurrentLim_amp) {

      m_hoodMotor.stopMotor();
      m_hoodMotorEncoder.setPosition(ShooterConstants.hoodUpperLimit_deg);
      m_hoodAtHome = true;
      m_hoodHomeStartTime_sec = -1;

      return;
    }

    if (elapsed_sec > ShooterConstants.hoodHomingTimeout_sec) {

      m_hoodMotor.stopMotor();
      m_hoodHomeStartTime_sec = -1;

      SmartDashboard.putBoolean("Hood Homing Timeout", true);
    }
  }

  public boolean isHoodAtHome() {
    SmartDashboard.putNumber("Hood Encoder", m_hoodMotorEncoder.getPosition());
    return m_hoodAtHome;
  }

  public void setHoodAtHome(boolean atHome) {
    m_hoodAtHome = atHome;

    if (!atHome)
      m_hoodHomeStartTime_sec = -1;
  }

  public void setHoodAngle(double target_deg) {

    if (!m_hoodAtHome)
      return;

    target_deg = Math.max(target_deg, ShooterConstants.hoodLowerLimit_deg);
    target_deg = Math.min(target_deg, ShooterConstants.hoodUpperLimit_deg);

    m_hoodTargetPosition_deg = target_deg;
  }

  public void setHoodPositionFromDashboard() {
    double targetPosition_deg = SmartDashboard.getNumber(HOOD_POSITION_KEY, m_hoodTargetPosition_deg);
    setHoodAngle(targetPosition_deg);
  }

  public void setPIDValues(double hoodP, double hoodI, double hoodD) {

    m_hoodMotorConfig.closedLoop.pid(hoodP, hoodI, hoodD);
    m_hoodMotor.configure(m_hoodMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setShooterVelocity(double targetVelocity_ips) {

    if (targetVelocity_ips == m_targetVelocity_ips)
      return;

    m_targetVelocity_ips = targetVelocity_ips * ShooterConstants.flywheelSlipFactor;

    m_rightController.setSetpoint(m_targetVelocity_ips, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_middleController.setSetpoint(m_targetVelocity_ips, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_leftController.setSetpoint(m_targetVelocity_ips, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public boolean rightAtSpeed() {
    return Math
        .abs(m_rightEncoder.getVelocity() - m_targetVelocity_ips) < ShooterConstants.flywheelVelocityTolerance_ips;
  }

  public boolean middleAtSpeed() {
    return Math
        .abs(m_middleEncoder.getVelocity() - m_targetVelocity_ips) < ShooterConstants.flywheelVelocityTolerance_ips;
  }

  public boolean leftAtSpeed() {
    return Math
        .abs(m_leftEncoder.getVelocity() - m_targetVelocity_ips) < ShooterConstants.flywheelVelocityTolerance_ips;
  }

  public boolean allAtSpeed() {
    return rightAtSpeed() && middleAtSpeed() && leftAtSpeed();
  }

  public boolean hoodAtPosition() {
    return Math
        .abs(m_hoodMotorEncoder.getPosition() - m_hoodTargetPosition_deg) < ShooterConstants.hoodPositionTolerance_deg;
  }

  public boolean isReadyToShoot() {

    boolean atSpeed = allAtSpeed();
    boolean atAngle = hoodAtPosition();

    if (atSpeed && atAngle) {

      if (m_readyStartTime < 0) {
        m_readyStartTime = Timer.getFPGATimestamp();
      }

      double timeReady = Timer.getFPGATimestamp() - m_readyStartTime;

      return timeReady > 0.1; // ✅ 100 ms stable

    } else {
      m_readyStartTime = -1;
      return false;
    }
  }

  public double getAverageVelocity_ips() {
    return (m_rightEncoder.getVelocity()
        + m_middleEncoder.getVelocity()
        + m_leftEncoder.getVelocity()) / 3.0;
  }

  public boolean updateShotDetection() {

    double currentVelocity_ips = getAverageVelocity_ips();
    double now_sec = Timer.getFPGATimestamp();

    m_shotDetected = false;

    if ((m_lastAvgVelocity_ips - currentVelocity_ips) > ShooterConstants.shotDipThreshold_rpm) {

      if (m_lastShotTime_sec < 0 || (now_sec - m_lastShotTime_sec) > ShooterConstants.shotMinSeparation_sec) {

        m_shotDetected = true;
        m_lastShotTime_sec = now_sec;
      }
    }

    m_lastAvgVelocity_ips = currentVelocity_ips;
    return m_shotDetected;
  }

  public void cancel() {
    m_rightMotor.stopMotor();
    m_middleMotor.stopMotor();
    m_leftMotor.stopMotor();
    m_hoodMotor.stopMotor();
    m_targetVelocity_ips = 0;
    m_hoodTargetPosition_deg = ShooterConstants.hoodLowerLimit_deg;
  }

  public void stopHood() {
    m_hoodMotor.stopMotor();
  }

  @Override
  public void periodic() {

    if (m_hoodAtHome) {

      m_hoodController.setSetpoint(m_hoodTargetPosition_deg, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    updateShotDetection();

    SmartDashboard.putNumber("Shooter Commanded IPS", m_targetVelocity_ips);
    SmartDashboard.putNumber("Shooter Actual IPS", getAverageVelocity_ips());

    SmartDashboard.putBoolean("Hood Homed", m_hoodAtHome);

    SmartDashboard.putNumber("Hood Commanded Position", m_hoodTargetPosition_deg);
    SmartDashboard.putNumber("Hood Actual Position", m_hoodMotorEncoder.getPosition());
    SmartDashboard.putNumber("Hood Current", m_hoodMotor.getOutputCurrent());

    SmartDashboard.putBoolean("Shot Detected", m_shotDetected);

    SmartDashboard.putNumber("Right motor RPM", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("Middle motor RPM", m_middleEncoder.getVelocity());
    SmartDashboard.putNumber("Left motor RPM", m_leftEncoder.getVelocity());
  }
}
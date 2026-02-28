// Copyright (c) FIRST and other WPILib contributors.

package frc.robot.subsystems;

import static frc.robot.Constants.FeederConstants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class MagazineAndFeed extends SubsystemBase {

  
  // Hardware
  
  private SparkMax m_feedMotor;
  private SparkMax m_magazineMotor;

  private RelativeEncoder m_feedEncoder;
  private RelativeEncoder m_magazineEncoder;

  private Shooter m_shooter;

  
  // Jam protection state
  
  private boolean m_jamDetected = false;
  private boolean m_reversing = false;

  private double m_jamStartTime_sec = 0.0;
  private double m_reverseStartTime_sec = 0.0;

  
  // Shot recovery state
 
  private double m_lastShotTime_sec = -999.0;

  
  // Dashboard change tracking
  
  private boolean m_lastFeedAllowed = false;

 
  // Constructor
 
  public void MagazineAndFeeder(Shooter shooter) {

    m_shooter = shooter;

    m_feedMotor =
        new SparkMax(FeederConstants.feedMotorID, MotorType.kBrushless);

    m_magazineMotor =
        new SparkMax(FeederConstants.magazineMotorID, MotorType.kBrushless);

    configureMotor(m_feedMotor);
    configureMotor(m_magazineMotor);

    m_feedEncoder = m_feedMotor.getEncoder();
    m_magazineEncoder = m_magazineMotor.getEncoder();
  }

 
  // Motor config
 
  private void configureMotor(SparkMax motor) {

    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake)
          .smartCurrentLimit(FeederConstants.freeLimit_amp);

    motor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  
  // Public control
 
  /** Magazine can run anytime */
  public void runMagazine(double speed_perc) {
    m_magazineMotor.set(speed_perc);
  }

  /** Smart feed with full protection */
  public void feedIfReady(double baseSpeed_perc) {

    boolean shooterReady = m_shooter.allAtSpeed();
    boolean recoveryDone = isRecoveryComplete();

    boolean allowed = shooterReady && recoveryDone;

    // ---------- dashboard change-only ----------
    if (allowed != m_lastFeedAllowed) {
      SmartDashboard.putBoolean("Feeder Allowed", allowed);
      m_lastFeedAllowed = allowed;
    }

    if (!allowed) {
      m_feedMotor.stopMotor();
      return;
    }

    // ---------- shot detection ----------
    if (m_shooter.didJustShoot()) {
      m_lastShotTime_sec = Timer.getFPGATimestamp();
    }

    // ---------- jam reverse ----------
    if (m_reversing) {
      handleJamReverse();
      return;
    }

    // ---------- brownout scaling ----------
    double commandedSpeed_perc =
        applyBrownoutScale(baseSpeed_perc);

    // ---------- speed matching ----------
    double shooterRPM = m_shooter.getAverageRPM();
    double scale = shooterRPM / FeederConstants.shooterNominalRPM_rpm;
    scale = Math.max(0.3, Math.min(1.2, scale));

    commandedSpeed_perc *= scale;

    m_feedMotor.set(commandedSpeed_perc);

    detectJam();
  }

  /** Manual override (testing only) */
  public void forceFeed(double speed_perc) {
    m_feedMotor.set(speed_perc);
  }

  public void stopAll() {
    m_feedMotor.stopMotor();
    m_magazineMotor.stopMotor();
  }

  // =========================
  // Shot recovery logic
  // =========================

  private boolean isRecoveryComplete() {

    // ---------- Elite mode (RPM recovered) ----------
    if (m_shooter.allAtSpeed()) {
      return true;
    }

    // ---------- Time fallback ----------
    double now_sec = Timer.getFPGATimestamp();
    return (now_sec - m_lastShotTime_sec) > FeederConstants.shotRecoveryTime_sec;
  }

  // =========================
  // Brownout protection
  // =========================

  private double applyBrownoutScale(double commandedSpeed_perc) {

    double voltage_V = RobotController.getBatteryVoltage();

    if (voltage_V < FeederConstants.BrownoutCriticalVoltage_V) {
      return 0.0;
    }

    if (voltage_V < FeederConstants.BrownoutStartVoltage_V) {
      return commandedSpeed_perc * FeederConstants.brownoutScale_perc;
    }

    return commandedSpeed_perc;
  }

  
  // Jam detection
 
  private void detectJam() {

    double rpm = Math.abs(m_feedEncoder.getVelocity());
    double now_sec = Timer.getFPGATimestamp();

    if (rpm < FeederConstants.jamRPMThreshold_rpm) {

      if (m_jamStartTime_sec == 0.0) {
        m_jamStartTime_sec = now_sec;
      }

      if ((now_sec - m_jamStartTime_sec) > FeederConstants.jamTime_sec) {
        m_jamDetected = true;
        m_reversing = true;
        m_reverseStartTime_sec = now_sec;
        m_jamStartTime_sec = 0.0;
      }

    } else {
      m_jamStartTime_sec = 0.0;
    }
  }

  private void handleJamReverse() {

    double now_sec = Timer.getFPGATimestamp();

    m_feedMotor.set(FeederConstants.reverseSpeed_perc);

    if ((now_sec - m_reverseStartTime_sec) > FeederConstants.jamClearTime_sec) {
      m_reversing = false;
      m_jamDetected = false;
    }
  }

  
  // Periodic
 
  @Override
  public void periodic() {

    SmartDashboard.putNumber(
        "Feed Motor RPM",
        m_feedEncoder.getVelocity());

    SmartDashboard.putNumber(
        "Magazine Motor RPM",
        m_magazineEncoder.getVelocity());

    SmartDashboard.putBoolean(
        "Feeder Jam Detected",
        m_jamDetected);

    SmartDashboard.putBoolean(
        "Feeder Reversing",
        m_reversing);
  }
}

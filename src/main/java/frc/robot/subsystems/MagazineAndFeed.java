// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotController;

import frc.robot.Constants.FeederConstants;

public class MagazineAndFeed extends SubsystemBase {

  private final SparkMax m_feedMotor;
  private final SparkMax m_magazineMotor;

  private final RelativeEncoder m_feedEncoder;
  private final RelativeEncoder m_magazineEncoder;

  private final Shooter m_shooter;

  // Jam detection state
  private boolean m_jamDetected = false;
  private double m_jamStartTime_sec = 0.0;
  private double m_reverseStartTime_sec = 0.0;
  private boolean m_reversing = false;

  // recovery tracking
  private boolean m_waitingForRecovery = false;
  private boolean m_lastAllAtSpeed = false;

  // optional time safety net
  private double m_lastShotTime_sec = -999.0;

  // Dashboard state tracking
  private boolean m_lastFeedAllowed = false;

  public MagazineAndFeed(Shooter shooter) {

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

  private void configureMotor(SparkMax motor) {

    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake)
          .smartCurrentLimit(FeederConstants.freeLimit_amp);

    motor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }
  
  // Brownout protection helper
  private double applyBrownoutScale(double commandedSpeed_perc) {

    double batteryVoltage_V = RobotController.getBatteryVoltage();

    if (batteryVoltage_V < FeederConstants.BrownoutCriticalVoltage_V) {
      return 0.0;
    }

    if (batteryVoltage_V < FeederConstants.BrownoutStartVoltage_V) {
      return commandedSpeed_perc * FeederConstants.brownoutScale_perc;
    }

    return commandedSpeed_perc;
  }

 // recovery state update
   private void updateShotRecoveryState() {

    boolean allAtSpeed = m_shooter.allAtSpeed();

    // detect shot (falling edge)
    if (m_lastAllAtSpeed && !allAtSpeed) {
      m_waitingForRecovery = true;
      m_lastShotTime_sec = Timer.getFPGATimestamp();
    }

    // detect full recovery
    if (m_waitingForRecovery && allAtSpeed) {
      m_waitingForRecovery = false;
    }

    m_lastAllAtSpeed = allAtSpeed;
  }
 
  // Magazine control
   public void runMagazine(double speed_perc) {
    double safeSpeed_perc = applyBrownoutScale(speed_perc);
    m_magazineMotor.set(safeSpeed_perc);
  }

  // Smart feed with protection
   public void feedIfReady(double baseSpeed_perc) {

    boolean allowed = m_shooter.allAtSpeed();

    if (!allowed) {
      m_feedMotor.stopMotor();
      return;
    }

    // ---------- RPM recovery ----------
    double now_sec = Timer.getFPGATimestamp();

    if (m_waitingForRecovery &&
        (now_sec - m_lastShotTime_sec)
            < FeederConstants.shotRecoveryTime_sec) {

      m_feedMotor.stopMotor();
      return;
    }

    // ---------- Jam reverse ----------
    if (m_reversing) {
      handleJamReverse();
      return;
    }

    // ---------- Speed matching ----------
    double shooterRPM_rpm = m_shooter.getAverageRPM();

    double scale = shooterRPM_rpm / FeederConstants.shooterNominalRPM_rpm;

    scale = Math.max(0.3, Math.min(1.2, scale));

    double commandedSpeed_perc = baseSpeed_perc * scale;

    // ---------- Brownout protection ----------
    double safeSpeed_perc = applyBrownoutScale(commandedSpeed_perc);

    m_feedMotor.set(safeSpeed_perc);

    // ---------- Jam detection ----------
    detectJam();

    // ---------- Dashboard change-only ----------
    if (allowed != m_lastFeedAllowed) {
      SmartDashboard.putBoolean("Feeder Allowed", allowed);
      m_lastFeedAllowed = allowed;
    }
  }
  
  // Jam detection
   private void detectJam() {

    double rpm = Math.abs(m_feedEncoder.getVelocity());
    double now_sec = Timer.getFPGATimestamp();

    if (rpm < FeederConstants.jamRPMThreshold_rpm) {

      if (m_jamStartTime_sec == 0.0) {
        m_jamStartTime_sec = now_sec;
      }

      if ((now_sec - m_jamStartTime_sec)
          > FeederConstants.jamTime_sec) {

        m_jamDetected = true;
        m_reversing = true;
        m_reverseStartTime_sec = now_sec;
        m_jamStartTime_sec = 0.0;
      }

    } else {
      m_jamStartTime_sec = 0.0;
    }
  }

  
  // Jam clear reverse
   private void handleJamReverse() {

    double now_sec = Timer.getFPGATimestamp();

    double safeReverse_perc =
        applyBrownoutScale(FeederConstants.reverseSpeed_perc);

    m_feedMotor.set(safeReverse_perc);

    if ((now_sec - m_reverseStartTime_sec)
        > FeederConstants.jamClearTime_sec) {

      m_reversing = false;
      m_jamDetected = false;
    }
  }

  public void stopAll() {
    m_feedMotor.stopMotor();
    m_magazineMotor.stopMotor();
  }

  @Override
  public void periodic() {

    updateShotRecoveryState();

    SmartDashboard.putNumber("Feed Motor RPM", m_feedEncoder.getVelocity());
    SmartDashboard.putNumber("Magazine Motor RPM", m_magazineEncoder.getVelocity());
    SmartDashboard.putBoolean("Feeder Jam Detected", m_jamDetected);
    SmartDashboard.putBoolean("Feeder Reversing", m_reversing);
    SmartDashboard.putBoolean("Feeder Waiting For Recovery", m_waitingForRecovery);
    SmartDashboard.putNumber("Feeder Battery Voltage_V", RobotController.getBatteryVoltage());
  }
}

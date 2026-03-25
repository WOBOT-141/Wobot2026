// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import pabeles.concurrency.IntRangeTask;

public class Intake extends SubsystemBase {

  // Hardware
  private final SparkMax m_intakeExtensionRightMotor;
  private final SparkMax m_intakeExtensionLeftMotor;
  private final SparkFlex m_intakeRollerLeaderMotor;
  private final SparkFlex m_intakeRollerFollowerMotor;

  private final RelativeEncoder m_intakeRightEncoder;
  private final RelativeEncoder m_intakeLeftEncoder;
  private final RelativeEncoder m_intakeRollerLeaderEncoder;
  private final RelativeEncoder m_intakeRollerFollowerEncoder;

  // State
  private boolean m_isHomed = true;
  private boolean m_homingActive = false;

  private double m_moveStartTime_sec = 0.0;

  // Dashboard change detection
  private boolean m_lastAtUp = false;
  private boolean m_lastAtDown = false;

  // Constructor
  public Intake() {

    m_intakeExtensionRightMotor = new SparkMax(CANIdConstants.intakeElevationRightMotorID, MotorType.kBrushless);
    m_intakeExtensionLeftMotor = new SparkMax(CANIdConstants.intakeElevationLeftMotorID, MotorType.kBrushless);
    m_intakeRollerLeaderMotor = new SparkFlex(CANIdConstants.intakeRollerLeaderMotorID, MotorType.kBrushless);
    m_intakeRollerFollowerMotor = new SparkFlex(CANIdConstants.intakeRollerFollowerMotorID, MotorType.kBrushless);

    configureIntakeRollerMotor();
    configureIntaknMotors();

    m_intakeRightEncoder = m_intakeExtensionRightMotor.getEncoder();
    m_intakeLeftEncoder = m_intakeExtensionLeftMotor.getEncoder();
    m_intakeRollerLeaderEncoder = m_intakeRollerLeaderMotor.getEncoder();
    m_intakeRollerFollowerEncoder = m_intakeRollerFollowerMotor.getEncoder();
  }

  // Motor configuration
  private void configureIntaknMotors() {
    SparkMaxConfig intakeRightConfig = new SparkMaxConfig();
    SparkMaxConfig intakeLeftConfig = new SparkMaxConfig();

    intakeRightConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(IntakeConstants.elevationCurrentLimit_amp, IntakeConstants.elevationCurrentLimit_amp);
    intakeLeftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.elevationCurrentLimit_amp, IntakeConstants.elevationCurrentLimit_amp);

    m_intakeExtensionRightMotor.configure(
        intakeRightConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_intakeExtensionLeftMotor.configure(
        intakeLeftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private void configureIntakeRollerMotor(){
    SparkFlexConfig intakeRollerLeaderMotorConfig = new SparkFlexConfig();
    SparkFlexConfig intakeRollerFollowerMotorConfig = new SparkFlexConfig();
    
    intakeRollerLeaderMotorConfig.closedLoop.pid(IntakeConstants.intakeRollerP, IntakeConstants.intakeRollerI, IntakeConstants.intakeRollerD).feedForward.kV(IntakeConstants.intakeRollerFF, ClosedLoopSlot.kSlot0);
    intakeRollerFollowerMotorConfig.closedLoop.pid(IntakeConstants.intakeRollerP, IntakeConstants.intakeRollerI, IntakeConstants.intakeRollerD).feedForward.kV(IntakeConstants.intakeRollerFF, ClosedLoopSlot.kSlot0);

    intakeRollerLeaderMotorConfig
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .smartCurrentLimit(IntakeConstants.rollerCurrentLimit_amp, IntakeConstants.rollerCurrentLimit_amp);
    intakeRollerFollowerMotorConfig
        .idleMode(IdleMode.kCoast)
        .follow(CANIdConstants.intakeRollerLeaderMotorID, true)
        .smartCurrentLimit(IntakeConstants.rollerCurrentLimit_amp, IntakeConstants.rollerCurrentLimit_amp);
    
    m_intakeRollerLeaderMotor.configure(
        intakeRollerLeaderMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_intakeRollerFollowerMotor.configure(
        intakeRollerFollowerMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public boolean isAtRetractedPosition() {
    return m_intakeRightEncoder.getPosition() <= 0 && m_intakeLeftEncoder.getPosition() <= 0;
  }

  public boolean isAtExtendedPosition() {
    return m_intakeRightEncoder.getPosition() >= IntakeConstants.intakeRawExtension_rot
        && m_intakeLeftEncoder.getPosition() >= IntakeConstants.intakeRawExtension_rot;
  }

  // Roller control
  public void intakeRun(double speed_rpm) {
    m_intakeRollerLeaderMotor.getClosedLoopController().setSetpoint(speed_rpm, ControlType.kVelocity);
  }

  public void intakeRollerStop() {
    m_intakeRollerLeaderMotor.stopMotor();
  }

  // Arm control
  public void extendFollowerIntake() {
    if (m_intakeLeftEncoder.getPosition() > IntakeConstants.intakeRawExtension_rot) {
      m_intakeExtensionLeftMotor.set(0);
      return;
    }
    if (m_intakeLeftEncoder.getPosition() > (IntakeConstants.intakeRawExtension_rot - IntakeConstants.SlowSpot)) {
      m_intakeExtensionLeftMotor.set(applyBrownoutScale(IntakeConstants.intakeExtendSpeed_perc / 2));
    } else {
      m_intakeExtensionLeftMotor.set(applyBrownoutScale(IntakeConstants.intakeExtendSpeed_perc));
    }

    // m_moveStartTime_sec = Timer.getFPGATimestamp();
    System.out.println("intakeMovingDown");
  }

  public void extendLeaderIntake() {
    if (m_intakeRightEncoder.getPosition() > IntakeConstants.intakeRawExtension_rot) {
      m_intakeExtensionRightMotor.set(0);
      return;
    }
    if (m_intakeRightEncoder.getPosition() > (IntakeConstants.intakeRawExtension_rot - IntakeConstants.SlowSpot)) {
      m_intakeExtensionRightMotor.set(applyBrownoutScale(IntakeConstants.intakeExtendSpeed_perc / 2));
    } else {
      m_intakeExtensionRightMotor.set(applyBrownoutScale(IntakeConstants.intakeExtendSpeed_perc));
    }

    // m_moveStartTime_sec = Timer.getFPGATimestamp();
    System.out.println("intakeMovingDown");
  }

  public void retractFollowerIntake() {
    if (m_intakeLeftEncoder.getPosition() < 0) {
      m_intakeExtensionLeftMotor.set(0);
      return;
    }
    if (m_intakeLeftEncoder.getPosition() < IntakeConstants.SlowSpot) {
      m_intakeExtensionLeftMotor.set(applyBrownoutScale(IntakeConstants.intakeRetractSpeed_perc / 2));
    } else {
      m_intakeExtensionLeftMotor.set(applyBrownoutScale(IntakeConstants.intakeRetractSpeed_perc));
    }

    // m_moveStartTime_sec = Timer.getFPGATimestamp();
    System.out.println("intakeMovingDown");
  }

  public void retractLeaderIntake() {
    if (m_intakeRightEncoder.getPosition() < 0) {
      m_intakeExtensionRightMotor.set(0);
      return;
    }

    if (m_intakeRightEncoder.getPosition() < IntakeConstants.SlowSpot) {
      m_intakeExtensionRightMotor.set(applyBrownoutScale(IntakeConstants.intakeRetractSpeed_perc / 2));
    } else {
      m_intakeExtensionRightMotor.set(applyBrownoutScale(IntakeConstants.intakeRetractSpeed_perc));
    }
    // m_moveStartTime_sec = Timer.getFPGATimestamp();
    System.out.println("intakeMovingDown");
  }

  public void extendIntake() {
    extendFollowerIntake();
    extendLeaderIntake();
  }

  public void retractIntake() {
    retractFollowerIntake();
    retractLeaderIntake();
  }

  public double getElevationCurrent() {
    return m_intakeExtensionRightMotor.getOutputCurrent();
  }

  public void stopArm() {
    m_intakeExtensionRightMotor.stopMotor();
    m_intakeExtensionLeftMotor.stopMotor();
  }

  // Homing (current based)
  // public void intakeHome() {

  // m_homingActive = true;
  // m_isHomed = false;
  // SmartDashboard.putBoolean("Intake Homed", m_isHomed);

  // m_intakeElevationRightMotor.set(
  // applyBrownoutScale(IntakeConstants.homingSpeed_perc));
  // }

  // private void updateHoming() {

  // if (!m_homingActive)
  // return;

  // double current_amp = m_intakeElevationRightMotor.getOutputCurrent();
  // SmartDashboard.putNumber(
  // "Intake current",
  // current_amp);
  // SmartDashboard.putNumber(
  // "Intake current thr",
  // IntakeConstants.homingCurrentThreshold_amp);

  // if (current_amp > IntakeConstants.homingCurrentThreshold_amp) {
  // // HARD STOP detected
  // m_intakeElevationRightEncoder.setPosition(0.0);

  // m_isHomed = true;
  // m_homingActive = false;

  // m_intakeElevationRightMotor.stopMotor();

  // SmartDashboard.putBoolean("Intake Homed", true);
  // }
  // }

  // public boolean isHomed() {
  // return m_isHomed;
  // }

  // Brownout protection
  private double applyBrownoutScale(double commandedSpeed_perc) {

    double voltage_V = RobotController.getBatteryVoltage();
    if (voltage_V < IntakeConstants.brownoutCriticalVoltage_V) {
      SmartDashboard.putBoolean("Brownout critical voltage", true);
      return commandedSpeed_perc
          * IntakeConstants.brownoutScale_perc;
    }

    if (voltage_V < IntakeConstants.brownoutStartVoltage_V) {
      SmartDashboard.putBoolean("Brownout start voltage", true);
      return commandedSpeed_perc * 0.7;
    }

    return commandedSpeed_perc;
  }

  @Override
  public void periodic() {
    // updateHoming();

    // Safety timeout while moving
    // if (Math.abs(m_intakeElevationMotor.get()) > 0.01) {
    double elapsed_sec = Timer.getFPGATimestamp() - m_moveStartTime_sec;

    // if (elapsed_sec > IntakeConstants.moveTimeout_sec) {
    // m_intakeElevationMotor.stopMotor();
    // }
    // }

    // Dashboard numbers
    SmartDashboard.putNumber(
        "Intake Leader Position",
        m_intakeRightEncoder.getPosition());

    SmartDashboard.putNumber(
        "Intake Follower Position",
        m_intakeLeftEncoder.getPosition());

    SmartDashboard.putNumber(
        "Intake Roller RPM",
        m_intakeRollerLeaderEncoder.getVelocity());

    // Change-only status updates
    boolean atUpNow = isAtExtendedPosition();
    boolean atDownNow = isAtRetractedPosition();

    if (atUpNow != m_lastAtUp) {
      SmartDashboard.putBoolean("Intake At Up", atUpNow);
      m_lastAtUp = atUpNow;
    }

    if (atDownNow != m_lastAtDown) {
      SmartDashboard.putBoolean("Intake At Down", atDownNow);
      m_lastAtDown = atDownNow;
    }
  }
}
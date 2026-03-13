// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
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

public class Intake extends SubsystemBase {

  // Hardware
  private final SparkMax m_intakeElevationLeaderMotor;
  private final SparkMax m_intakeElevationFollowerMotor;
  private final SparkFlex m_intakeRollerMotor;

  private final RelativeEncoder m_intakeElevationLeaderEncoder;
  private final RelativeEncoder m_intakeElevationFollowerEncoder;
  private final RelativeEncoder m_intakeRollerEncoder;

  // State
  private boolean m_isHomed = true;
  private boolean m_homingActive = false;

  private double m_moveStartTime_sec = 0.0;

  // Dashboard change detection
  private boolean m_lastAtUp = false;
  private boolean m_lastAtDown = false;

  // Constructor
  public Intake() {

    m_intakeElevationLeaderMotor = new SparkMax(CANIdConstants.intakeElevationRightMotorID, MotorType.kBrushless);
    m_intakeElevationFollowerMotor = new SparkMax(CANIdConstants.intakeElevationLeftMotorID, MotorType.kBrushless);
    m_intakeRollerMotor = new SparkFlex(CANIdConstants.intakeRollerMotorID, MotorType.kBrushless);

    configureIntakeRollerMotor(m_intakeRollerMotor);
    configureIntakeElevationMotors();

    m_intakeElevationLeaderEncoder = m_intakeElevationLeaderMotor.getEncoder();
    m_intakeElevationFollowerEncoder = m_intakeElevationFollowerMotor.getEncoder();
    m_intakeRollerEncoder = m_intakeRollerMotor.getEncoder();
  }

  // Motor configuration
  private void configureIntakeElevationMotors() {
    SparkMaxConfig intakeElevationMotorLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig intakeElevationMotorFollowerConfig = new SparkMaxConfig();

    intakeElevationMotorLeaderConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(IntakeConstants.elevationCurrentLimit_amp, IntakeConstants.elevationCurrentLimit_amp);

    intakeElevationMotorFollowerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.elevationCurrentLimit_amp, IntakeConstants.elevationCurrentLimit_amp);
    // intakeElevationMotorLeftConfig.encoder
    // .positionConversionFactor(IntakeConstants.degPerMotorRotation_deg);

    intakeElevationMotorFollowerConfig.follow(CANIdConstants.intakeElevationRightMotorID, true);

    m_intakeElevationLeaderMotor.configure(
        intakeElevationMotorLeaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_intakeElevationFollowerMotor.configure(
        intakeElevationMotorFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private void configureIntakeRollerMotor(SparkFlex intakeRollerMotor) {
    SparkFlexConfig intakeRollerMotorConfig = new SparkFlexConfig();
    intakeRollerMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.rollerCurrentLimit_amp, IntakeConstants.rollerCurrentLimit_amp);

    intakeRollerMotor.configure(
        intakeRollerMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public boolean isAtUpPosition() {
    return m_intakeElevationLeaderEncoder.getPosition() <=0;
  }

  public boolean isAtDownPosition() {
    return m_intakeElevationLeaderEncoder.getPosition() >= IntakeConstants.intakeRawExtension_rot;
  }

  // Roller control
  public void intakeRun(double speed_perc) {
    m_intakeRollerMotor.set(speed_perc);
  }

  public void intakeRollerStop() {
    m_intakeRollerMotor.stopMotor();
  }

  // Arm control
  public void intakeDown() {

    if (!isIntakeWithinRange()){
      m_intakeElevationLeaderMotor.set(0);
      return;
    }

    m_intakeElevationLeaderMotor.set(applyBrownoutScale(IntakeConstants.intakeDownSpeed_perc));
    // m_moveStartTime_sec = Timer.getFPGATimestamp();
    System.out.println("intakeMovingDown");
  }

  public void intakeUp() {
    if (!isIntakeWithinRange()){
      m_intakeElevationLeaderMotor.set(0);
      return;
    }

    m_intakeElevationLeaderMotor.set(applyBrownoutScale(IntakeConstants.intakeUpSpeed_perc));

    // if (!m_isHomed)
    // return;

    // m_intakeElevationRightMotor.set(applyBrownoutScale(IntakeConstants.intakeDownSpeed_perc));
    // // m_moveStartTime_sec = Timer.getFPGATimestamp();
    // System.out.println("intakeMovingup");

  }

  public boolean isIntakeWithinRange(){
    return m_intakeElevationFollowerEncoder.getPosition() > 0 &&
        m_intakeElevationLeaderEncoder.getPosition() > 0 &&
        m_intakeElevationFollowerEncoder.getPosition() < IntakeConstants.intakeRawExtension_rot &&
        m_intakeElevationLeaderEncoder.getPosition() < IntakeConstants.intakeRawExtension_rot;
      
  }

  public double getElevationCurrent() {
    return m_intakeElevationLeaderMotor.getOutputCurrent();
  }

  public void stopArm() {
    m_intakeElevationLeaderMotor.stopMotor();
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
        m_intakeElevationLeaderEncoder.getPosition());

    SmartDashboard.putNumber(
        "Intake Follower Position",
        m_intakeElevationFollowerEncoder.getPosition());

    SmartDashboard.putNumber(
        "Intake Roller RPM",
        m_intakeRollerEncoder.getVelocity());

    // Change-only status updates
    boolean atUpNow = isAtUpPosition();
    boolean atDownNow = isAtDownPosition();

    if (atUpNow != m_lastAtUp) {
      SmartDashboard.putBoolean("Intake At Up", atUpNow);
      m_lastAtUp = atUpNow;
    }

    if (atDownNow != m_lastAtDown) {
      SmartDashboard.putBoolean("Intake At Down", atDownNow);
      m_lastAtDown = atDownNow;
    }

    if (!isIntakeWithinRange()){
      m_intakeElevationLeaderMotor.set(0);
      return;
    }
  }
}
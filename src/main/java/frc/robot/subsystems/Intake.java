// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.CANIdConstants;

public class Intake extends SubsystemBase {

  // Hardware
  private final SparkMax m_intakeElevationMotor;
  private final SparkFlex m_intakeRollerMotor;

  private final RelativeEncoder m_intakeEncoder;
  private final RelativeEncoder m_intakeRollerEncoder;

  // State
  private boolean m_isHomed = false;
  private boolean m_homingActive = false;

  private double m_moveStartTime_sec = 0.0;

  // Dashboard change detection
  private boolean m_lastAtUp = false;
  private boolean m_lastAtDown = false;


  // Constructor
   public Intake() {

    m_intakeElevationMotor = new SparkMax(CANIdConstants.intakeElevationMotorID, MotorType.kBrushless);
    m_intakeRollerMotor = new SparkFlex(CANIdConstants.intakeRollerMotorID, MotorType.kBrushless);
    
    configureIntakeRollerMotor(m_intakeRollerMotor);
    configureIntakeElevationMotor(m_intakeElevationMotor);

    m_intakeEncoder = m_intakeElevationMotor.getEncoder();
    m_intakeRollerEncoder = m_intakeRollerMotor.getEncoder();
  }

 
  // Motor configuration
  private void configureIntakeElevationMotor(SparkMax intakeElevationMotor) {
    SparkMaxConfig intakeElevationMotorConfig = new SparkMaxConfig();
    intakeElevationMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.elevationCurrentLimit_amp, IntakeConstants.elevationCurrentLimit_amp);
    intakeElevationMotorConfig.encoder
        .positionConversionFactor(IntakeConstants.degPerMotorRotation_deg);
    
    intakeElevationMotor.configure(
        intakeElevationMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private void configureIntakeRollerMotor(SparkFlex intakeRollerMotor) {
    SparkFlexConfig intakeRollerMotorConfig = new SparkFlexConfig();
    intakeRollerMotorConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.rollerCurrentLimit_amp, IntakeConstants.rollerCurrentLimit_amp);

    intakeRollerMotor.configure(
        intakeRollerMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public boolean isAtUpPosition() {
    return Math.abs(m_intakeEncoder.getPosition() - IntakeConstants.upAngle_deg)
        < IntakeConstants.positionTolerance_deg;
  }

  public boolean isAtDownPosition() {
    return Math.abs(m_intakeEncoder.getPosition() - IntakeConstants.downAngle_deg)
        < IntakeConstants.positionTolerance_deg;
  }

 
  // Roller control
  public void intakeRun(double speed_perc) {
    m_intakeRollerMotor.set(speed_perc);
  }

  public void intakeStop() {
    m_intakeRollerMotor.stopMotor();
  }

 
  // Arm control
  public void intakeDown() {

    if (!m_isHomed)
      return;

    m_moveStartTime_sec = Timer.getFPGATimestamp();

    m_intakeElevationMotor.set(
        applyBrownoutScale(IntakeConstants.intakeDownSpeed_perc));
  }

  public void intakeUp() {

    if (!m_isHomed)
      return;

    m_moveStartTime_sec = Timer.getFPGATimestamp();

    m_intakeElevationMotor.set(
        applyBrownoutScale(IntakeConstants.intakeUpSpeed_perc));
  }

  public void stopArm() {
    m_intakeElevationMotor.stopMotor();
  }

  // Homing (current based)
  public void intakeHome() {

    m_homingActive = true;
    m_isHomed = false;
    SmartDashboard.putBoolean("Intake Homed", m_isHomed);

    m_intakeElevationMotor.set(
        applyBrownoutScale(IntakeConstants.homingSpeed_perc));
  }

  private void updateHoming() {

    if (!m_homingActive)
      return;

    double current_amp = m_intakeElevationMotor.getOutputCurrent();
    SmartDashboard.putNumber(
        "Intake current",
        current_amp);
        SmartDashboard.putNumber(
        "Intake current thr",
        IntakeConstants.homingCurrentThreshold_amp);

    if (current_amp > IntakeConstants.homingCurrentThreshold_amp) {

      // HARD STOP detected
      m_intakeEncoder.setPosition(0.0);

      m_isHomed = true;
      m_homingActive = false;

      m_intakeElevationMotor.stopMotor();

      SmartDashboard.putBoolean("Intake Homed", true);
    }
  }

  public boolean isHomed() {
    return m_isHomed;
  }


  // Brownout protection
  private double applyBrownoutScale(double commandedSpeed_perc) {

    double voltage_V = RobotController.getBatteryVoltage();

    if (voltage_V < IntakeConstants.brownoutCriticalVoltage_V) {
      return commandedSpeed_perc
          * IntakeConstants.brownoutScale_perc;
    }

    if (voltage_V < IntakeConstants.brownoutStartVoltage_V) {
      return commandedSpeed_perc * 0.7;
    }

    return commandedSpeed_perc;
  }

  @Override
  public void periodic() {

    updateHoming();

    // Safety timeout while moving
    if (Math.abs(m_intakeElevationMotor.get()) > 0.01) {
      double elapsed_sec = Timer.getFPGATimestamp() - m_moveStartTime_sec;

      if (elapsed_sec > IntakeConstants.moveTimeout_sec) {
        m_intakeElevationMotor.stopMotor();
      }
    }

    // Dashboard numbers
    SmartDashboard.putNumber(
        "Intake Angle deg",
        m_intakeEncoder.getPosition());

    SmartDashboard.putNumber(
        "Intake Motor RPM",
        m_intakeEncoder.getVelocity());

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
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIdConstants;

public class Intake extends SubsystemBase {
  private final SparkMax m_slapdownMotor;
  private final SparkMax m_rollerMotor;
  private AbsoluteEncoder m_absoluteEncoder;

  /** Creates a new Intake. */
  public Intake() { // TODO set ids
    m_slapdownMotor = new SparkMax(CANIdConstants.intakeSlapdownMotor, MotorType.kBrushless);
    m_rollerMotor = new SparkMax(CANIdConstants.intakeRollerMotor, MotorType.kBrushless);

    SparkMaxConfig slapdownMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();

    slapdownMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20, 20);
    slapdownMotorConfig.closedLoop
        .pid(0.001, 0, 0);
    slapdownMotorConfig.encoder.positionConversionFactor(0); // TODO set factor

    rollerMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20, 20);
    rollerMotorConfig.closedLoop
        .pid(0.001, 0, 0);

    m_slapdownMotor.configure(slapdownMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_absoluteEncoder = m_slapdownMotor.getAbsoluteEncoder();

  }

  public void extend() {

  }

  public void retract() {

  }

  public void run(double percent) {

  }

  // public void fuelIntaking() {
  // rollerMotor.set(0.1);
  // }

  /*
   * public void SlappingDown() {
   * slapdownMotor.set(0.1);
   * 
   * }
   * 
   * public void PullingUp() { //if slapdwon motor is
   * if (slapdownMotor >)
   * slapdownMotor.set(-0.1);
   * 
   * }
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

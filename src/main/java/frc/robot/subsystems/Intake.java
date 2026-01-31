// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  private SparkMax slapdownMotor;
  private SparkMax rollerMotor;
  /** Creates a new Intake. */
  public Intake() { //TODO set ids
     slapdownMotor = new SparkMax(0, MotorType.kBrushless);
     rollerMotor = new SparkMax(0, MotorType.kBrushless);

     SparkMaxConfig slapdownMotorConfig = new SparkMaxConfig();
     SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();

     slapdownMotorConfig
     .idleMode(IdleMode.kBrake)
     .smartCurrentLimit(20, 20);
     slapdownMotorConfig.closedLoop
     .pid(0.001, 0, 0);

     rollerMotorConfig
     .idleMode(IdleMode.kCoast)
     .smartCurrentLimit(20, 20);
     rollerMotorConfig.closedLoop
     .pid(0.001, 0, 0);

  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // rollerMotor.set(.1);
  }
}

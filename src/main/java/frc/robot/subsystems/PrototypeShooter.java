// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PrototypeShooter extends SubsystemBase {
  private final SparkMax m_flywheelMotor;
  private final SparkMax m_feedMotor;
  private final SparkMaxConfig flywheelMotorConfig;
  private final SparkMaxConfig feedMotorConfig;

  private final RelativeEncoder M_flywheelEncoder;
  private final RelativeEncoder M_feedMotorEncoder;
  private final SparkClosedLoopController M_flywheelController;
  private final SparkClosedLoopController M_feedmotorController;

  /** Creates a new PrototypeShooter. */
//TODO add manual mode form RPM fixed point with button increase and decrease 
  public PrototypeShooter() {
    SmartDashboard.putNumber("flywheelRPM", 0.0);
    SmartDashboard.putNumber("feedMotorRPM", 0.0);
    SmartDashboard.putNumber("flywheelP", 0.0);
    SmartDashboard.putNumber("flywheelI", 0.0);
    SmartDashboard.putNumber("flywheelD", 0.0);
    SmartDashboard.putNumber("feedMotorP", 0.0);
    SmartDashboard.putNumber("feedMotorI", 0.0);
    SmartDashboard.putNumber("feedMotorD", 0.0);

    m_flywheelMotor = new SparkMax(14, MotorType.kBrushless);
    m_feedMotor = new SparkMax(13, MotorType.kBrushless);

    flywheelMotorConfig = new SparkMaxConfig();
    feedMotorConfig = new SparkMaxConfig();

    flywheelMotorConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(60, 60);
    flywheelMotorConfig.closedLoop
    .pid(0, 0, 0);//TODO setup motor constences  Pid values on smartdashboard add reset 

    feedMotorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(60, 60);
    feedMotorConfig.closedLoop
    .pid(0, 0, 0); //TODO setup motor constences   Pid values on smartdashboard

    m_flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_feedMotor.configure(feedMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    M_flywheelEncoder = m_flywheelMotor.getEncoder();
    M_feedMotorEncoder = m_feedMotor.getEncoder();

    M_flywheelController = m_flywheelMotor.getClosedLoopController();
    M_feedmotorController = m_feedMotor.getClosedLoopController();
  }
  public void setShooterRPM(double flywheelRPM, double feedMotorRPM ){
    M_flywheelController.setSetpoint(flywheelRPM, SparkMax.ControlType.kVelocity);
    M_feedmotorController.setSetpoint(feedMotorRPM,SparkMax.ControlType.kVelocity);
  }  
  public double getFlywheelRPM(){
    return M_flywheelEncoder.getVelocity();
  }
  public double getFeedMotorRPM(){
    return M_feedMotorEncoder.getVelocity();
  }
  

  @Override
  public void periodic() {
    double flywheelP = SmartDashboard.getNumber("flywheelP", 0.0);
    double flywheelI = SmartDashboard.getNumber("flywheelI", 0.0);
    double flywheelD = SmartDashboard.getNumber("flywheelD", 0.0);
    double feedMotorP = SmartDashboard.getNumber("feedMotorP", 0.0);
    double feedMotorI = SmartDashboard.getNumber("feedMotorI", 0.0);
    double feedMotorD = SmartDashboard.getNumber("feedMotorD", 0.0);

    flywheelMotorConfig.closedLoop
    .pid(flywheelP, flywheelI, flywheelD);
    feedMotorConfig.closedLoop
    .pid(feedMotorP, feedMotorI, feedMotorD);

    m_flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_feedMotor.configure(feedMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    double flywheelRPM = SmartDashboard.getNumber("flywheelRPM", 0.0);
    double feedMotorRPM = SmartDashboard.getNumber("feedMotorRPM", 0.0);
    setShooterRPM(flywheelRPM, feedMotorRPM); //TODO Setup periodic table with actual values 
    // This method will be called once per scheduler run

  }
}

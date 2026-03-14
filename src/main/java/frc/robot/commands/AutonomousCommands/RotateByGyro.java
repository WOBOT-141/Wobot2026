// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RotateCommandConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;

public class RotateByGyro extends Command {
  Drivetrain m_drive;
  IGyro m_gyro;

  double m_wantedAngle;
  double m_currentAngle;
  double m_finalAngle;
  PIDController m_turnPID;

  public RotateByGyro(double wantedAngle, boolean botOriented, Drivetrain drive, IGyro gyro) {
    m_drive = drive;
    m_gyro = gyro;
    m_wantedAngle = wantedAngle;

    m_turnPID = new PIDController(Constants.RotateCommandConstants.P, Constants.RotateCommandConstants.I,
        Constants.RotateCommandConstants.D);
    m_turnPID.setTolerance(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnPID.reset();
    m_currentAngle = m_gyro.getRawGyroAngle();
    m_finalAngle = m_currentAngle + m_wantedAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("RotateByGyro current", currentAngle);
    // SmartDashboard.putNumber("RotateByGyro final", finalAngle);
    m_currentAngle = m_gyro.getRawGyroAngle();
    double rotationalOutput = m_turnPID.calculate(m_currentAngle, m_finalAngle) / 90d;
    if (rotationalOutput > 1) {
      rotationalOutput = 1;
    } else if (rotationalOutput < -1) {
      rotationalOutput = -1;
    }
    // SmartDashboard.putNumber("Rotational output: ", rotationalOutput);
    // System.out.println("PID output----------- " + rotationalOutput);

    m_drive.driveWithoutGyro(0, 0, rotationalOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double degreeDifference = Math.abs(m_wantedAngle - m_currentAngle) % 360;
    return degreeDifference < RotateCommandConstants.rotateTargetTolerance_deg;
  }
}
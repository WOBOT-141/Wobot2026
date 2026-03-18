// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.RotateCommandConstants;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.subsystems.Vision.VisionServer;
import frc.robot.subsystems.Vision.Detections.IDetection;
import frc.robot.subsystems.Vision.Detections.NullDetection;

public class MovingRotateToTag extends Command {

  private final VisionServer m_visionServer;
  private final StickCommand m_stickCommand;

  private int m_closestTagId;
  private final PIDController m_pidController;
  private IDetection m_closestDetection;

  public MovingRotateToTag(VisionServer visionServer, StickCommand stickCommand) {

    m_visionServer = visionServer;
    m_stickCommand = stickCommand;

    m_pidController = new PIDController(
        RotateCommandConstants.P,
        RotateCommandConstants.I,
        RotateCommandConstants.D);

    m_pidController.setSetpoint(0);
    m_pidController.setTolerance(2.0);
  }

  @Override
  public void initialize() {

    m_pidController.reset();

    m_closestDetection = m_visionServer.getClosestDetection();

    if (!m_closestDetection.isValid()) {
      m_closestTagId = 0;
      return;
    }

    m_closestTagId = (int) m_closestDetection.getId();
  }

  @Override
  public void execute() {

    IDetection currentDetection =
        m_visionServer.getDetectionList().getDetectionById(m_closestTagId);

    if (!currentDetection.isValid()) {
      m_stickCommand.setAlternateRotation(Optional.empty());
      return;
    }

    double angle_deg = currentDetection.getAngle();

    double pidOutput = -m_pidController.calculate(angle_deg);

    if (m_pidController.atSetpoint()) {
      pidOutput = 0;
    }

    pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

    double pidOutput = -m_pidController.calculate(currentDetection.getBearing());
    m_stickCommand.setAlternateRotation(Optional.of(pidOutput));

    SmartDashboard.putNumber("Tag Angle", angle_deg);
    SmartDashboard.putNumber("PIDOutput", pidOutput);
  }

  @Override
  public void end(boolean interrupted) {

    m_stickCommand.setAlternateRotation(Optional.empty());
    m_closestDetection = NullDetection.Instance;

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

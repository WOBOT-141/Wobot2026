// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.AngleMagVector;
import frc.robot.Constants.RotateCommandConstants;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.VisionServer;
import frc.robot.subsystems.Vision.AprilTags.AprilTagFilter;
import frc.robot.subsystems.Vision.AprilTags.FieldElement;

public class AimAtHubCommand extends Command {

  private final StickCommand m_stickCommand;
  private final VisionServer m_visionServer;
  private final Shooter m_shooter;
  private final AprilTagFilter m_aprilTagFilter;

  private final PIDController m_pidController;

  public AimAtHubCommand(
      StickCommand stickCommand,
      VisionServer visionServer,
      Shooter shooter,
      AprilTagFilter aprilTagFilter) {

    m_stickCommand = stickCommand;
    m_visionServer = visionServer;
    m_shooter = shooter;
    m_aprilTagFilter = aprilTagFilter;

    m_pidController = new PIDController(
        RotateCommandConstants.P,
        RotateCommandConstants.I,
        RotateCommandConstants.D);

    m_pidController.setSetpoint(0);
    m_pidController.setTolerance(2.0);

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_pidController.reset();
  }

  @Override
  public void execute() {

    Optional<AngleMagVector> vectorOptional =
        m_aprilTagFilter.getFloorVectorToTaggedElement(
            m_visionServer.getDetectionList(),
            FieldElement.HUB);

    if (vectorOptional.isEmpty()) {
      m_stickCommand.setAlternateRotation(Optional.empty());
      return;
    }

    AngleMagVector vector = vectorOptional.get();
    double bearing_deg = vector.Angle;

    double pidOutput = m_pidController.calculate(bearing_deg);

    if (m_pidController.atSetpoint()) {
      pidOutput = 0;
    }

    pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

    SmartDashboard.putNumber("Hub Angle", bearing_deg);
    SmartDashboard.putNumber("Hub PID Output", pidOutput);

    m_stickCommand.setAlternateRotation(Optional.of(pidOutput));
  }

  @Override
  public void end(boolean interrupted) {
    m_stickCommand.setAlternateRotation(Optional.empty());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

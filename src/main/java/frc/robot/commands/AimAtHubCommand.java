// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
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

  // Track when we last saw the tag
  private double m_lastSeenTime_sec = -1;

  // How long we will trust the last vision reading before giving control back
  private static final double kVisionTimeout_sec = 0.5;

  public AimAtHubCommand(
      StickCommand stickCommand,
      VisionServer visionServer,
      Shooter shooter,
      AprilTagFilter aprilTagFilter) {

    m_stickCommand = stickCommand;
    m_visionServer = visionServer;
    m_shooter = shooter;
    m_aprilTagFilter = aprilTagFilter;

    m_pidController =
        new PIDController(
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
    m_lastSeenTime_sec = -1;
  }

  @Override
  public void execute() {

    Optional<AngleMagVector> vectorOptional =
        m_aprilTagFilter.getFloorVectorToTaggedElement(
            m_visionServer.getDetectionList(),
            FieldElement.HUB);

    double now = Timer.getFPGATimestamp();

    if (vectorOptional.isPresent()) {

      AngleMagVector vectorToHub = vectorOptional.get();

      double pidOutput = m_pidController.calculate(vectorToHub.Angle);

      m_stickCommand.setAlternateRotation(Optional.of(pidOutput));

      // Remember when we last saw the target
      m_lastSeenTime_sec = now;
    }
    else {

      // If vision disappeared briefly, continue using last PID output
      if (m_lastSeenTime_sec < 0 || (now - m_lastSeenTime_sec) > kVisionTimeout_sec) {

        // Vision has been gone too long — return control to driver
        m_stickCommand.setAlternateRotation(Optional.empty());
      }
    }
  }

  @Override
  public void end(boolean interrupted) {

    // Return control of rotation back to the driver
    m_stickCommand.setAlternateRotation(Optional.empty());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

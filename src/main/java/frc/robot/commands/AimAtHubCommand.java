// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.AngleMagVector;
import frc.robot.Constants.RotateCommandConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.ShooterCalculations;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.VisionServer;
import frc.robot.subsystems.Vision.AprilTags.AprilTagFilter;
import frc.robot.subsystems.Vision.AprilTags.FieldElement;
import frc.robot.subsystems.Vision.Detections.DetectionList;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtHubCommand extends Command {

  private final StickCommand m_stickCommand;
  private final VisionServer m_visionServer;
  private final Shooter m_shooter;
  private final AprilTagFilter m_aprilTagFilter;
  private PIDController m_pidController;
  private CommandXboxController m_controller;

  public AimAtHubCommand(
      StickCommand stickCommand,
      VisionServer visionServer,
      Shooter shooter,
      AprilTagFilter aprilTagFilter,
      CommandXboxController controller) {

    m_stickCommand = stickCommand;
    m_shooter = shooter;
    m_visionServer = visionServer;
    m_aprilTagFilter = aprilTagFilter;
    m_controller = controller;
    m_pidController = new PIDController(RotateCommandConstants.P, RotateCommandConstants.I, RotateCommandConstants.D);
    m_pidController.setSetpoint(0);
    m_pidController.setTolerance(2.0);

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DetectionList detectionList = m_visionServer.getApriltagDetectionLists()[0];
    Optional<AngleMagVector> vectorToHubOptional = m_aprilTagFilter
        .getFloorVectorToTaggedElement(detectionList, FieldElement.HUB);

    if (vectorToHubOptional.isEmpty())
      return;

    AngleMagVector vectorToHub = vectorToHubOptional.get();

    double pidOutput = m_pidController.calculate(vectorToHub.Angle);
    double launchAngle = ShooterCalculations.getLaunchAngle(vectorToHub.Magnitude,
        ShooterConstants.muzzleToRimHeight_in, ShooterConstants.hubRimToCenter_in, 24);
    double launchSpeed = ShooterCalculations.getLaunchSpeed(launchAngle, vectorToHub.Magnitude,
        ShooterConstants.hubRimToCenter_in, 24);
    
    m_stickCommand.setAlternateRotation(Optional.of(pidOutput));
    m_shooter.setHoodAngle(launchAngle);
    m_shooter.setShooterVelocity(launchSpeed);

    if (m_shooter.isReadyToShoot() && m_pidController.atSetpoint()) {
      m_controller.setRumble(RumbleType.kBothRumble, 0.6);
    } else {
      m_controller.setRumble(RumbleType.kBothRumble, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_stickCommand.setAlternateRotation(Optional.empty());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
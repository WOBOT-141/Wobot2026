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
import frc.robot.subsystems.Vision.Detections.IDetection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtHubCommand extends Command {

  private final StickCommand m_stickCommand;
  private final VisionServer m_visionServer;
  private final Shooter m_shooter;
  private final AprilTagFilter m_aprilTagFilter;
  private PIDController m_pidController;
  private final ShooterCalculations m_ShooterCalculations;
  private CommandXboxController m_controller;

  public AimAtHubCommand(StickCommand stickCommand, VisionServer visionServer, Shooter shooter,
      AprilTagFilter aprilTagFilter, ShooterCalculations shooterCalculations, CommandXboxController controller) {
    m_ShooterCalculations = shooterCalculations;
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
    if (vectorToHubOptional.isEmpty()) {
      return;
    }
    AngleMagVector vectorToHub = vectorToHubOptional.get();
    double pidOutput = 0;
    for (int i = 0; i < detectionList.getSize(); i++) {
      IDetection detection = detectionList.getDetection(i);
      if (m_aprilTagFilter.isCenterTag(detection.getId())) {
        pidOutput = m_pidController.calculate(detection.getBearing() - vectorToHub.Angle);
      }
    }

    m_stickCommand.setAlternateRotation(Optional.of(pidOutput));
    double launchAngle = ShooterCalculations.getLaunchAngle(vectorToHub.Magnitude,
        ShooterConstants.muzzleToRimHeight_in, ShooterConstants.hubRimToCenter_in, 24);
    m_shooter.setHoodAngle(launchAngle);
    double launchSpeed = ShooterCalculations.getLaunchSpeed(launchAngle, vectorToHub.Magnitude,
        ShooterConstants.hubRimToCenter_in, 24);
    m_shooter.setShooterVelocity(launchSpeed);

    if (m_shooter.isReadyToShoot() && m_pidController.atSetpoint()) {
      m_controller.setRumble(RumbleType.kBothRumble, 0.0);
      // TODO Use bearing and subtract the vector to hub angle from it to get pid
      // output, check to make sure the pid output is within tolerance, and then
      // create a keybinding, finished, just test and make sure we are doing everything right
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AngleMagVector;
import frc.robot.Constants;
import frc.robot.Constants.RotateCommandConstants;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.VisionServer;
import frc.robot.subsystems.Vision.AprilTags.AprilTagFilter;
import frc.robot.subsystems.Vision.AprilTags.FieldElement;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtHubCommand extends Command {

  private final StickCommand m_stickCommand;
  private final VisionServer m_visionServer;
  private final Shooter m_shooter;
  private final AprilTagFilter m_aprilTagFilter;
  private PIDController m_pidController;

  public AimAtHubCommand(StickCommand stickCommand, VisionServer visionServer, Shooter shooter, AprilTagFilter aprilTagFilter) {

    m_stickCommand = stickCommand;
    m_shooter = shooter;
    m_visionServer = visionServer;
    m_aprilTagFilter = aprilTagFilter;

  m_pidController = new PIDController(RotateCommandConstants.P, RotateCommandConstants.I, RotateCommandConstants.D);
  m_pidController.setSetpoint(0);
  m_pidController.setTolerance(2.0);
  

  addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<AngleMagVector> vectorToHubOptional = m_aprilTagFilter.getFloorVectorToTaggedElement(m_visionServer.getApriltagDetectionLists()[0], FieldElement.HUB);
    if (vectorToHubOptional.isEmpty()){
      return;
    }
    AngleMagVector vectorToHub = vectorToHubOptional.get();
     double pidOutput = m_pidController.calculate(vectorToHub.Angle);
     m_stickCommand.setAlternateRotation(Optional.of(pidOutput));
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

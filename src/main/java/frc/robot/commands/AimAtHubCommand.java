// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import javax.print.attribute.standard.PrinterIsAcceptingJobs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.ServerVision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtHubCommand extends Command {

  private final StickCommand m_stickCommand;
  private final ServerVision m_visionServer;
  private final Shooter m_shooter;

  public AimAtHubCommand(StickCommand stickCommand, ServerVision visionServer, Shooter shooter) {

    m_stickCommand = stickCommand;
    m_shooter = shooter;
    m_visionServer = visionServer;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationToTarget_perc = 0.0; 
    m_stickCommand.setAlternateRotation(Optional.of(rotationToTarget_perc));
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RotateCommandConstants;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.Vision.PIDProvider;
import frc.robot.subsystems.Vision.VisionServer;
import frc.robot.subsystems.Vision.Detections.Detection;
import frc.robot.subsystems.Vision.Detections.DetectionList;
import frc.robot.subsystems.Vision.Detections.IDetection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MovingRotateToTag extends Command {
private final VisionServer m_visionServer;
private final StickCommand m_stickCommand;

private int m_closestTagId;
private PIDController m_pidController;
private IDetection m_closestDetection;
  /** Creates a new RotateToTag. */
  public MovingRotateToTag(VisionServer visionServer, StickCommand stickCommand) {
    m_visionServer = visionServer; 
    m_stickCommand = stickCommand;
     
    m_pidController = new PIDController(RotateCommandConstants.P, RotateCommandConstants.I, RotateCommandConstants.D);
     m_pidController.setSetpoint(0);
     m_pidController.setTolerance(2.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     m_closestDetection = m_visionServer.getClosestDetection();
     m_closestTagId = (int) m_closestDetection.getId();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IDetection currentDetection = m_visionServer.getDetectionList().getDetectionById(m_closestTagId);
    double pidOutput = -m_pidController.calculate(currentDetection.getAngle());
    m_stickCommand.setAlternateRotation(Optional.of(pidOutput));

    SmartDashboard.putNumber("RotateToTagP", m_pidController.getP());
    SmartDashboard.putNumber("RotateTotagI", m_pidController.getI());
    SmartDashboard.putNumber("RotateToTagD", m_pidController.getD());
    SmartDashboard.putNumber("PIDOutput", pidOutput);
    SmartDashboard.putNumber("ClosestTagAngle", currentDetection.getAngle());
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

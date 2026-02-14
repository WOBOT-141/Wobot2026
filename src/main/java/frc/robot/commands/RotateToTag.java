// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RotateCommandConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.Vision.PIDProvider;
import frc.robot.subsystems.Vision.VisionServer;
import frc.robot.subsystems.Vision.Detections.Detection;
import frc.robot.subsystems.Vision.Detections.DetectionList;
import frc.robot.subsystems.Vision.Detections.IDetection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToTag extends Command {
private final VisionServer m_visionServer;
private final Drivetrain m_drivetrain;
private int m_closestTagId;
private PIDController m_pidController;
private IDetection m_closestDetection;
  /** Creates a new RotateToTag. */
  public RotateToTag(VisionServer visionServer, Drivetrain drivetrain) {
    m_visionServer = visionServer; 
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     m_closestDetection = m_visionServer.getClosestDetection();
     m_closestTagId = (int) m_closestDetection.getId();
     m_pidController = new PIDController(PIDProvider.getDriveOmega_P(), PIDProvider.getDriveOmega_I(), PIDProvider.getDriveOmega_D());
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IDetection currentDetection = m_visionServer.getClosestDetection();
    double pidOutput = m_pidController.calculate(currentDetection.getYaw());
    m_drivetrain.driveSwerve(0, 0, pidOutput);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }
}

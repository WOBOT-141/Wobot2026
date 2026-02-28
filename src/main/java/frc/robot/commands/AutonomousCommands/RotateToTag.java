// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RotateCommandConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision.VisionServer;
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
     m_pidController = new PIDController(RotateCommandConstants.P, RotateCommandConstants.I, RotateCommandConstants.D);
     m_pidController.setSetpoint(0);
     m_pidController.setTolerance(2.0);
     m_drivetrain.driveWithGyro(0, 0, 0); 
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IDetection currentDetection = m_visionServer.getClosestDetection();
    double pidOutput = m_pidController.calculate(currentDetection.getAngle());
    m_drivetrain.driveWithoutGyro(0, 0, -pidOutput);
    SmartDashboard.putNumber("RotateToTagP", m_pidController.getP());
    SmartDashboard.putNumber("RotateTotagI", m_pidController.getI());
    SmartDashboard.putNumber("RotateToTagD", m_pidController.getD());
    SmartDashboard.putNumber("PIDOutput", pidOutput);
    SmartDashboard.putNumber("ClosestTagAngle", currentDetection.getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return m_pidController.atSetpoint();
    return false;
  }


}

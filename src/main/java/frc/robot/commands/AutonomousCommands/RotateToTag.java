// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RotateVisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision.VisionServer;
import frc.robot.subsystems.Vision.Detections.DetectionList;
import frc.robot.subsystems.Vision.Detections.IDetection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToTag extends Command {
  private final VisionServer m_visionServer;
  private final Drivetrain m_drivetrain;
  private int m_tagId;
  private PIDController m_pidController;

  /** Creates a new RotateToTag. */
  public RotateToTag(int tagID, VisionServer visionServer, Drivetrain drivetrain) {
    m_visionServer = visionServer;
    m_drivetrain = drivetrain;
    m_tagId = tagID;
    m_pidController = new PIDController(RotateVisionConstants.P, RotateVisionConstants.I, RotateVisionConstants.D);
    m_pidController.setSetpoint(0);
    m_pidController.setTolerance(3.0);
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.reset();

    // m_drivetrain.driveWithGyro(0, 0, 0);
    System.out.println("Rotate To Tag START");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DetectionList detectionList = m_visionServer.getApriltagDetectionLists()[0];
    System.out.println(detectionList.getSize());
    IDetection currentDetection = detectionList.getDetectionById(m_tagId);
    if (!currentDetection.isValid()){
      System.out.println("NO TAG DETECTED");
      return;
    }
    double pidOutput = m_pidController.calculate(currentDetection.getBearing());
    m_drivetrain.driveWithoutGyro(0, 0, -pidOutput, false);

    SmartDashboard.putNumber("RotateToTagP", m_pidController.getP());
    SmartDashboard.putNumber("RotateTotagI", m_pidController.getI());
    SmartDashboard.putNumber("RotateToTagD", m_pidController.getD());
    SmartDashboard.putNumber("PIDOutput", pidOutput);
    SmartDashboard.putNumber("ClosestTagAngle", currentDetection.getBearing());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Rotate To Tag END");
    m_drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
    // return false;
  }

}

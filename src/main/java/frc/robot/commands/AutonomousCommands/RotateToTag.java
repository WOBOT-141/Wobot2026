package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.RotateCommandConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision.VisionServer;
import frc.robot.subsystems.Vision.Detections.IDetection;

public class RotateToTag extends Command {

  private final VisionServer m_visionServer;
  private final Drivetrain m_drivetrain;

  private PIDController m_pidController;
  private IDetection m_targetDetection;

  public RotateToTag(VisionServer visionServer, Drivetrain drivetrain) {
    m_visionServer = visionServer;
    m_drivetrain = drivetrain;

    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {

    m_targetDetection = m_visionServer.getClosestDetection();

    if (m_targetDetection == null) {
      cancel();
      return;
    }

    m_pidController = new PIDController(
        RotateCommandConstants.P,
        RotateCommandConstants.I,
        RotateCommandConstants.D);

    m_pidController.setSetpoint(0);
    m_pidController.setTolerance(1.0);

    m_pidController.enableContinuousInput(-180, 180);

    m_drivetrain.driveWithoutGyro(0, 0, 0);
  }

  @Override
  public void execute() {

    IDetection currentDetection = m_visionServer.getClosestDetection();

    // Lost target → stop safely
    if (currentDetection == null) {
      m_drivetrain.driveWithoutGyro(0, 0, 0);
      return;
    }

    double bearing = currentDetection.getBearing();

    double pidOutput = m_pidController.calculate(bearing);

    m_drivetrain.driveWithoutGyro(0, 0, -pidOutput);

    SmartDashboard.putNumber("RotateToTag_Bearing", bearing);
    SmartDashboard.putNumber("RotateToTag_Output", pidOutput);
    SmartDashboard.putBoolean("RotateToTag_AtSetpoint", m_pidController.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }
}
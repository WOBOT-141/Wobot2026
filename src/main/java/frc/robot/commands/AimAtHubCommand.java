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
import frc.robot.Constants.RotateVisionConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.ShooterCalculations;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.subsystems.IGyro;
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
  private final IGyro m_gyro;
  private final AprilTagFilter m_aprilTagFilter;
  private final PIDController m_pidController;
  private final CommandXboxController m_controller;

  private AngleMagVector m_lastFoundVector;
  private double m_gyroAtLastDetection_deg = 0;

  public AimAtHubCommand(
      StickCommand stickCommand,
      VisionServer visionServer,
      Shooter shooter,
      IGyro gyro,
      AprilTagFilter aprilTagFilter,
      CommandXboxController controller) {

    m_stickCommand = stickCommand;
    m_shooter = shooter;
    m_gyro = gyro;
    m_visionServer = visionServer;
    m_aprilTagFilter = aprilTagFilter;
    m_controller = controller;
    m_pidController = new PIDController(RotateVisionConstants.P, RotateVisionConstants.I, RotateVisionConstants.D);
    m_pidController.setSetpoint(0);
    m_pidController.setTolerance(2.0);

    m_lastFoundVector = new AngleMagVector(0, 0);

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.reset();
    System.out.println("AimAtHub START");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DetectionList detectionList = m_visionServer.getApriltagDetectionLists()[0];
    Optional<AngleMagVector> m_vectorToHubOptional = m_aprilTagFilter
        .getFloorVectorToTaggedElement(detectionList, FieldElement.HUB);

    System.out.printf("AimAtHubCommand tags:%d  hasVector:%b  lastGyro:%.3f\n", detectionList.getSize(),
        m_vectorToHubOptional.isPresent(), m_gyroAtLastDetection_deg);

    double pidOutput = 0;
    if (m_vectorToHubOptional.isPresent()) {
      m_lastFoundVector = m_vectorToHubOptional.get();
      pidOutput = m_pidController.calculate(-m_lastFoundVector.Angle);

    } else {
      // TODO Double-check the signage in this math
      double gyroAngleDiff_deg = m_gyro.getRawGyroAngle() - m_gyroAtLastDetection_deg;
      double vectorAngleDiff_deg = gyroAngleDiff_deg - m_lastFoundVector.Angle;
      pidOutput = m_pidController.calculate(vectorAngleDiff_deg);
    }

    m_gyroAtLastDetection_deg = m_gyro.getRawGyroAngle();

    double launchAngle = ShooterCalculations.getLaunchAngle(m_lastFoundVector.Magnitude,
        ShooterConstants.muzzleToRimHeight_in, ShooterConstants.hubRimToCenter_in, 24);
    double launchSpeed = ShooterCalculations.getLaunchSpeed(launchAngle, m_lastFoundVector.Magnitude,
        ShooterConstants.hubRimToCenter_in, 24);

    System.out.printf("[vectorAngle:%.3f  vectorDist:%.3f  altRotation:%.3f  launchAngle:%.3f  launchSpeed:%.3f]\n",
        m_lastFoundVector.Angle, m_lastFoundVector.Magnitude, pidOutput, launchAngle, launchSpeed);

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
    System.out.println("AimAtHub END");
    m_stickCommand.setAlternateRotation(Optional.empty());
    m_gyroAtLastDetection_deg = 0;
    m_lastFoundVector = new AngleMagVector(0, 0);
    m_shooter.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
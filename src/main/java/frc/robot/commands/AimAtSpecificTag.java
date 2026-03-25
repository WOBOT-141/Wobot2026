// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.AngleMagVector;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.RotateCommandConstants;
import frc.robot.Constants.RotateVisionConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.DashboardPIDGetter;
import frc.robot.Util.ShooterCalculations;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.Detections.IDetection;
import frc.robot.subsystems.Vision.VisionServer;
import frc.robot.subsystems.Vision.AprilTags.AprilTagFilter;
import frc.robot.subsystems.Vision.AprilTags.FieldElement;
import frc.robot.subsystems.Vision.Detections.DetectionList;
import frc.robot.subsystems.Vision.Detections.NullDetection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtSpecificTag extends Command {

  private final int m_tagId;
  private final StickCommand m_stickCommand;
  private final VisionServer m_visionServer;
  private final Shooter m_shooter;
  private final IGyro m_gyro;
  private final AprilTagFilter m_aprilTagFilter;
  private final PIDController m_pidController;
  private final CommandXboxController m_controller;
  private final DashboardPIDGetter m_getter;

  private IDetection m_lastFoundVector;
  private double m_targetGyroAngle_deg = 0;
  private boolean m_initialVectorFound = false;

  public AimAtSpecificTag(
    int tagId,
      StickCommand stickCommand,
      VisionServer visionServer,
      Shooter shooter,
      IGyro gyro,
      AprilTagFilter aprilTagFilter,
      CommandXboxController controller) {

    m_tagId = tagId;
    m_stickCommand = stickCommand;
    m_shooter = shooter;
    m_gyro = gyro;
    m_visionServer = visionServer;
    m_aprilTagFilter = aprilTagFilter;
    m_controller = controller;
    m_pidController = new PIDController(RotateVisionConstants.P, RotateVisionConstants.I, RotateVisionConstants.D);
    m_getter = new DashboardPIDGetter(m_pidController, "RotateCenterTag", RotateVisionConstants.P, RotateVisionConstants.I, RotateVisionConstants.D, 0);
    m_pidController.setTolerance(2.0);

    addRequirements(shooter);
  }


  @Override
  public void initialize() {
    m_pidController.reset();
    m_pidController.setSetpoint(0);
    m_getter.setPIDValuesFromDashboard();

    m_lastFoundVector = NullDetection.Instance;
    m_targetGyroAngle_deg = 0;
    m_initialVectorFound = false;

    System.out.println("AimAtHub START");
  }


  @Override
  public void execute() {
    //Get data to operate on
    double currentGyroAngle_deg = m_gyro.getRawGyroAngle();
    DetectionList detectionList = m_visionServer.getApriltagDetectionLists()[0];
    IDetection detection = detectionList.getDetectionById(m_tagId);

    //Update our targeting if we have usable tag data
    if (detection.isValid()) {
      m_lastFoundVector = detection;
      m_targetGyroAngle_deg = currentGyroAngle_deg + m_lastFoundVector.getBearing(); //TODO may need to be added instead of subtracted.

      m_pidController.setSetpoint(m_targetGyroAngle_deg);
      m_initialVectorFound = true;
    }
    
    //Don't command anything if we don't have something to work off yet.
    if (!m_initialVectorFound){
      System.out.println("AimAtHub: Initial vector not yet found");
      return;
    }

    //Command output
    double rotationPidOutput = m_pidController.calculate(currentGyroAngle_deg);
    m_stickCommand.setAlternateRotation(Optional.of(rotationPidOutput));

    //Controller Feedback
    double rightRumble = m_shooter.isReadyToShoot() ? ControllerConstants.rumbleFeedback_perc : 0.0;
    double leftRumble = m_pidController.atSetpoint() ? ControllerConstants.rumbleFeedback_perc : 0.0;
    m_controller.setRumble(RumbleType.kLeftRumble, leftRumble);
    m_controller.setRumble(RumbleType.kRightRumble, rightRumble);
   
    //Debug output
    System.out.printf("[AimAtCenterTag1 tags:%d  hasDetection:%b  gyroTarget:%.3f  gyroCurrent:%.3f  pidOutput:%.3f]\n", detectionList.getSize(),
        m_lastFoundVector.isValid(), m_targetGyroAngle_deg, currentGyroAngle_deg, rotationPidOutput);
    SmartDashboard.putNumber("AimAtHub targetAngle", m_targetGyroAngle_deg);
    SmartDashboard.putNumber("AimAtHub currentAngle", currentGyroAngle_deg);
  }


  public IDetection getFacingCenterDetection(DetectionList detectionList) {
    IDetection centerDetection = NullDetection.Instance;

    for (int i=0; i<detectionList.getSize(); i++) {
      IDetection detection = detectionList.getDetection(i);
      if (m_aprilTagFilter.getTagInfoIfRelevantCenter(detection.getId(), FieldElement.HUB).isEmpty())
        continue;

      if (!centerDetection.isValid() || detection.getYaw() < centerDetection.getYaw())
      {
        centerDetection = detection;
        continue;
      }
    }

    return centerDetection;
  }


  @Override
  public void end(boolean interrupted) {
    System.out.println("AimAtHub END");
    m_stickCommand.setAlternateRotation(Optional.empty());
    m_controller.setRumble(RumbleType.kBothRumble, 0);
    m_shooter.cancel();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
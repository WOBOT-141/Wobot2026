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
public class AimAtCenterTagCommand extends Command {

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
  private double m_launchAngle_deg;
  private double m_launchSpeed_ips;
  private boolean m_initialVectorFound = false;

  public AimAtCenterTagCommand(
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
    m_launchSpeed_ips = 0;
    m_launchAngle_deg = 0;
    m_initialVectorFound = false;

    System.out.println("AimAtHub START");
  }


  @Override
  public void execute() {
    //Get data to operate on
    double currentGyroAngle_deg = m_gyro.getRawGyroAngle();
    DetectionList detectionList = m_visionServer.getApriltagDetectionLists()[0];
    IDetection detection = getFacingCenterDetection(detectionList);

    //Update our targeting if we have usable tag data
    if (detection.isValid()) {
      m_lastFoundVector = detection;
      m_targetGyroAngle_deg = currentGyroAngle_deg + m_lastFoundVector.getBearing(); //TODO may need to be added instead of subtracted.

      m_pidController.setSetpoint(m_targetGyroAngle_deg);
      double tagDistance_in =  m_lastFoundVector.getDistance() + ShooterConstants.cameraToMuzzleDist_in;
      double floorDistance_in = AprilTagFilter.FindRightTriangleLeg(ShooterConstants.tagCenterToFloorHeight_in, tagDistance_in);

      SmartDashboard.putNumber("AimAtCenter TagDist", tagDistance_in);
      SmartDashboard.putNumber("AimAtCenter FloorDist", floorDistance_in);

      m_launchAngle_deg = ShooterCalculations.getLaunchAngle(floorDistance_in,
          ShooterConstants.muzzleToRimHeight_in, ShooterConstants.hubRimToCenter_in, 24);
      m_launchSpeed_ips = ShooterCalculations.getLaunchSpeed(m_launchAngle_deg, m_lastFoundVector.getDistance(),
          ShooterConstants.hubRimToCenter_in, 24);

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
    m_shooter.setHoodAngle(m_launchAngle_deg);
    m_shooter.setShooterVelocity(m_launchSpeed_ips);

    //Controller Feedback
    double rightRumble = m_shooter.isReadyToShoot() ? ControllerConstants.rumbleFeedback_perc : 0.0;
    double leftRumble = m_pidController.atSetpoint() ? ControllerConstants.rumbleFeedback_perc : 0.0;
    m_controller.setRumble(RumbleType.kLeftRumble, leftRumble);
    m_controller.setRumble(RumbleType.kRightRumble, rightRumble);
   
    //Debug output
    System.out.printf("[AimAtCenterTag1 tags:%d  hasDetection:%b  gyroTarget:%.3f  gyroCurrent:%.3f  pidOutput:%.3f]\n", detectionList.getSize(),
        m_lastFoundVector.isValid(), m_targetGyroAngle_deg, currentGyroAngle_deg, rotationPidOutput);
    System.out.printf("[AimAtCenterTag2 vectorAngle:%.3f  vectorDist:%.3f  altRotation:%.3f  launchAngle:%.3f  launchSpeed:%.3f]\n",
        m_lastFoundVector.getBearing(), m_lastFoundVector.getDistance(), rotationPidOutput, m_launchAngle_deg, m_launchSpeed_ips);
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
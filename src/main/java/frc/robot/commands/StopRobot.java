// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;

public class StopRobot extends InstantCommand {

  private final Intake m_intake;
  private final MagazineAndFeed m_magazine;
  private final Drivetrain m_drive;
  private final Shooter m_shooter;

  public StopRobot(Intake intake, MagazineAndFeed magazine, Drivetrain drive, Shooter shooter) {

    m_intake = intake;
    m_magazine = magazine;
    m_drive = drive;
    m_shooter = shooter;

    addRequirements(m_intake, m_magazine, m_drive, m_shooter);
  }

  @Override
  public void initialize() {

    m_shooter.cancel();
    m_magazine.cancel();
    m_intake.stopArm();
    m_intake.intakeRollerStop();
    m_drive.stopMotors();

  }
}

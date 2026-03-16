// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.MagazineAndFeed;

public class IntakeFuel extends Command {

  private final MagazineAndFeed m_magazine;
  private final double m_speed;

  private double m_startTime_sec;

  public IntakeFuel(MagazineAndFeed magazine, double speed) {

    m_magazine = magazine;
    m_speed = speed;

    addRequirements(m_magazine);
  }

  @Override
  public void initialize() {

    m_startTime_sec = Timer.getFPGATimestamp();

  }

  @Override
  public void execute() {

    if (Timer.getFPGATimestamp() - m_startTime_sec > FeederConstants.feedStartDelay_sec) {
      m_magazine.setFeedVelocity(m_speed);
    }

  }

  @Override
  public void end(boolean interrupted) {

    m_magazine.setFeedVelocity(0);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

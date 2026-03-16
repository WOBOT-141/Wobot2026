// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.MagazineAndFeed;

public class Feed extends Command {

  private final MagazineAndFeed m_magazineAndFeed;

  public Feed(MagazineAndFeed magazineAndFeed) {

    m_magazineAndFeed = magazineAndFeed;

    addRequirements(m_magazineAndFeed);
  }

  @Override
  public void execute() {

    m_magazineAndFeed.setFeedVelocity(FeederConstants.feedSpeed_perc);

  }

  @Override
  public void end(boolean interrupted) {

    m_magazineAndFeed.setFeedVelocity(0);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

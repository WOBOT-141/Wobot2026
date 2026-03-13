// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MagazineAndFeed;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setFeed extends Command {
  private final MagazineAndFeed m_magazine;
  private final double m_speed;
  /** Creates a new setFeed. */
  public setFeed(MagazineAndFeed magazine) {
    m_magazine = magazine;
    m_speed = .6;//TODO make actual value
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_magazine);
  }
  public setFeed(MagazineAndFeed magazine, double speed) {
    m_magazine = magazine;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_magazine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() > 0.5){
    m_magazine.setFeedVelocity(m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_magazine.setFeedVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

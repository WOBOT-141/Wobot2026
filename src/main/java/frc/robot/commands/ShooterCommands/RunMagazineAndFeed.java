// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Feed;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunMagazineAndFeed extends ParallelCommandGroup {
  Drivetrain m_drivetrain;
  MagazineAndFeed m_magazineAndFeed;
  Command lockWheels;
  Command feed;


    
   
  /** Creates a new FireIntake. */
  public RunMagazineAndFeed(Drivetrain drivetrain, MagazineAndFeed magazineAndFeed) {
    m_drivetrain = drivetrain;
    m_magazineAndFeed = magazineAndFeed;
    lockWheels = Commands.runOnce(() -> m_drivetrain.lockWheels());
    feed = new Feed(m_magazineAndFeed);
    WaitCommand wait = new WaitCommand(ShooterConstants.firingWaitTime_sec);
        //revShooter = new  ShooterCalculationAimCommand(120, ShooterConstants.muzzleToRimHeight_in, ShooterConstants.hubRimToCenter_in, 12, m_shooter);
        // Add your commands in the addCommands() call, e.g.

    addRequirements(m_drivetrain, m_magazineAndFeed);
    addCommands(lockWheels.andThen(feed));

  }
}

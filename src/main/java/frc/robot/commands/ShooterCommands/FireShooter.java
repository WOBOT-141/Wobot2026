// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Feed;
import frc.robot.commands.ShooterCalculationAimCommand;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FireShooter extends ParallelCommandGroup {
  Shooter m_shooter;
  Drivetrain m_drivetrain;
  MagazineAndFeed m_magazineAndFeed;
  Command lockWheels;
  Command feed = new Feed(m_magazineAndFeed);
  Command ShooterCalculationAimCommand = new  ShooterCalculationAimCommand(120, ShooterConstants.muzzleToRimHeight_in, ShooterConstants.hubRimToCenter_in, 12, m_shooter);
    
   
  /** Creates a new FireIntake. */
  public FireShooter(Shooter shooter, Drivetrain drivetrain, MagazineAndFeed magazineAndFeed) {
    m_shooter = shooter;
    m_drivetrain = drivetrain;
    m_magazineAndFeed = magazineAndFeed;
    lockWheels = Commands.run(() -> m_drivetrain.lockWheels());
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(lockWheels.alongWith(ShooterCalculationAimCommand).until(() -> m_shooter.isReadyToShoot()), feed);

  }
}

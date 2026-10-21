// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.IntakeCommands.IntakeFuel;
import frc.robot.commands.IntakeCommands.IntakeJams;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonShooterV2 extends SequentialCommandGroup {
  private final Shooter m_Shooter;
  private final MagazineAndFeed m_magazine;
  
  /** Creates a new AutonShooterV2. */
  public AutonShooterV2(
    Drivetrain drivetrain,
    Shooter shooter,
    Intake intake,
    MagazineAndFeed magazine
  ){

    m_Shooter = shooter;
    m_magazine = magazine;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(m_magazine, m_Shooter);
    addCommands(
        new aimByOdometry(drivetrain, shooter, ShooterConstants.muzzleToRimHeight_in, ShooterConstants.hubRimToCenter_in, 24)
            .alongWith(new WaitCommand(0.6)
                .andThen(new IntakeJams(intake)))
            .andThen(Commands.run(() -> m_Shooter.cancel())));
  }
}

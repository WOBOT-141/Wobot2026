// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.ShooterCommands.SendHoodToHome;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HoodResetter extends SequentialCommandGroup {
  /** Creates a new HoodResetter. */
  public HoodResetter(Shooter shooter, IGyro gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  
    // addCommands(
    //   new SendHoodToHome(shooter).andThen(
    //   new SendHoodToHome(shooter)).andThen(
    //   new SendHoodToHome(shooter)).andThen(
    //     new ResetGyro(gyro))
    // );
  }
}

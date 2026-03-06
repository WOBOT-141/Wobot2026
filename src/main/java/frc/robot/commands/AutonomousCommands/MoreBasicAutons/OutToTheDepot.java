// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MoreBasicAutons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousCommands.GyroResetter;
import frc.robot.commands.AutonomousCommands.StraightLineDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.SwerveModule;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OutToTheDepot extends SequentialCommandGroup {
  /** Creates a new OutToTheDepot. */
  public OutToTheDepot(Drivetrain drivetrain, SwerveModule[] wheels, IGyro gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new StraightLineDrive(drivetrain, -90, 130, wheels, gyro).andThen(
      new StraightLineDrive(drivetrain, 0, 185, wheels, gyro)).andThen(
      new StraightLineDrive(drivetrain, 0, 40, wheels, gyro)).andThen(
      new StraightLineDrive(drivetrain, 105, 90, wheels, gyro)).andThen(
      new StraightLineDrive(drivetrain, 45, -47, wheels, gyro)).andThen(
      new StraightLineDrive(drivetrain, 90, -12, wheels, gyro))
    ); //NOT DONE, some later task
  }
}

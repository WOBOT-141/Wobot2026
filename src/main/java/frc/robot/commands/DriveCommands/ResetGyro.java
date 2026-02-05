package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IGyro;

public class ResetGyro extends InstantCommand {
  IGyro _gyro;

  public ResetGyro(IGyro gyro) {
    _gyro = gyro;
  }

  @Override
  public void initialize() {
    _gyro.ResetGyro();
  }
}
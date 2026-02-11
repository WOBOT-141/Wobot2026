// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PrototypeShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrototypeShooterCMD extends InstantCommand {
private PrototypeShooter m_shooter;
  /** Creates a new DriveInDirection. */
  public void PrototypeShooterCMD(PrototypeShooter shooter) {
    m_shooter = shooter;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double flywheelP = SmartDashboard.getNumber("flywheelP", 0.0);
    double flywheelI = SmartDashboard.getNumber("flywheelI", 0.0);
    double flywheelD = SmartDashboard.getNumber("flywheelD", 0.0);
    double feedMotorP = SmartDashboard.getNumber("feedMotorP", 0.0);
    double feedMotorI = SmartDashboard.getNumber("feedMotorI", 0.0);
    double feedMotorD = SmartDashboard.getNumber("feedMotorD", 0.0);
    
    m_shooter.CMDmethod(flywheelP, flywheelI, flywheelD, feedMotorP, feedMotorI, feedMotorD);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

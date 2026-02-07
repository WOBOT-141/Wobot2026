// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToAngle extends Command {
  private final Drivetrain m_Drivetrain;
  private final double m_rotationAmount_deg;
  private double m_desiredBotAngle_deg;

  IGyro gyro;
  /** Creates a new RotateToAngle. */

  public RotateToAngle(Drivetrain drivetrain, double rotationAmount_deg, IGyro gyro) {
    m_Drivetrain = drivetrain;
    m_rotationAmount_deg = rotationAmount_deg;


    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   m_desiredBotAngle_deg = (gyro.getYaw() + m_rotationAmount_deg);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double missingAngle_deg = (gyro.getYaw() - m_desiredBotAngle_deg);
    
    if ( ( (missingAngle_deg / m_rotationAmount_deg) % 180 ) >= 180) {
      

    } else {
      
    }

  }

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

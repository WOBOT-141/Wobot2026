// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StraightLineDrive extends Command {
  private final Drivetrain m_drivetrain;
  private final double m_distance_in;
  private double distanceTraveled;
  private SwerveModule[] m_wheels;
  private double[] StartingEncoderPos_in;
  double desiredBotDistance;
  double DistanceLeftOver_perc;

  /** Creates a new StraightLineDrive. */
  public StraightLineDrive(Drivetrain drivetrain, double distance, SwerveModule wheels[]) {
    m_drivetrain = drivetrain;
    m_distance_in = distance;
    m_wheels = wheels;
    StartingEncoderPos_in = new double[m_wheels.length];
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (int i = 0; i < m_wheels.length; i++) {
      StartingEncoderPos_in[i] = m_wheels[i].getEncoderDistance();
      SmartDashboard.putNumber("StartingEncoderPos_" + i, StartingEncoderPos_in[i]);
    }
    desiredBotDistance = m_distance_in + distanceTraveled;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double[] CurrentEncoderPos_in = new double[m_wheels.length];
    double DistanceFound_perc = 1;
    double TotalSubstractedDistance = 0;
    for (int i = 0; i < m_wheels.length; i++) {
      CurrentEncoderPos_in[i] = m_wheels[i].getEncoderDistance();
      SmartDashboard.putNumber("CurrentEncoderPos_" + i, CurrentEncoderPos_in[i]);
      TotalSubstractedDistance += StartingEncoderPos_in[i] - CurrentEncoderPos_in[i];
    }

    double AvgSubstractedDistance = TotalSubstractedDistance / m_wheels.length;
    DistanceLeftOver_perc = Math.abs(AvgSubstractedDistance / desiredBotDistance);


    SmartDashboard.putNumber("distanceLeftOver_perc", DistanceLeftOver_perc);
    SmartDashboard.putNumber("TotalSubtractedDistance", AvgSubstractedDistance);
    SmartDashboard.putNumber("DistanceFound_perc", DistanceFound_perc);
    SmartDashboard.putNumber("m_distance_in", desiredBotDistance);

    double slowSpot = 0.3;
    double driveSpeed = 0.05;

    if (CurrentEncoderPos_in[1] != StartingEncoderPos_in[1]) {
      if (DistanceLeftOver_perc > 0.95 || 0.05 > DistanceLeftOver_perc) {
        driveSpeed = 0.05;
      } else if (DistanceFound_perc < slowSpot) {
        driveSpeed = 0.2;
      } else {
        driveSpeed = 0.5;
      }
    }

    m_drivetrain.driveWithoutGyro(0, driveSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveWithoutGyro(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DistanceLeftOver_perc > 0.97;
  }
}

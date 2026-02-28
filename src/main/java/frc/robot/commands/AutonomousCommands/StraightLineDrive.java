// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.SwerveModule;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StraightLineDrive extends Command {
  private final Drivetrain m_drivetrain;
  private final double m_distance_in;
  private final SwerveModule[] m_wheels;
  private final IGyro m_gyro;

  private double m_distanceTraveled_in;
  private double[] m_startingEncoderPos_in;
  private double m_desiredBotDistance_in;

  private double distanceLeftOver_perc = 0.01;

  private double driveSpeed;

  /** Creates a new StraightLineDrive. */
  public StraightLineDrive(Drivetrain drivetrain, double distance, SwerveModule wheels[], IGyro gyro) {
    m_drivetrain = drivetrain;
    m_distance_in = distance;
    m_wheels = wheels;
    m_gyro = gyro;
    m_startingEncoderPos_in = new double[m_wheels.length];
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gyro.ResetGyro();
    for (int i = 0; i < m_wheels.length; i++) {
      m_wheels[i].resetEncoder();
      m_startingEncoderPos_in[i] = m_wheels[i].getEncoderDistance();
      SmartDashboard.putNumber("StartingEncoderPos_" + i, m_startingEncoderPos_in[i]);
    }
    m_desiredBotDistance_in = m_distance_in + m_distanceTraveled_in;
    driveSpeed = 0.5;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double[] currentEncoderPos_in = new double[m_wheels.length];
    double totalSubstractedDistance_in = 0;
    for (int i = 0; i < m_wheels.length; i++) {
      currentEncoderPos_in[i] = m_wheels[i].getEncoderDistance();
      SmartDashboard.putNumber("CurrentEncoderPos_" + i, currentEncoderPos_in[i]);
      totalSubstractedDistance_in += Math.abs(m_startingEncoderPos_in[i] - currentEncoderPos_in[i]);
    }

    double m_avgSubstractedDistance_in = totalSubstractedDistance_in / m_wheels.length;
    distanceLeftOver_perc = m_avgSubstractedDistance_in / m_desiredBotDistance_in;

    SmartDashboard.putNumber("distanceLeftOver_perc", distanceLeftOver_perc);
    SmartDashboard.putNumber("m_AvgSubtractedDistance", m_avgSubstractedDistance_in);
    SmartDashboard.putNumber("m_distance_in", m_desiredBotDistance_in);
    SmartDashboard.putNumber("driveSpeed", driveSpeed);

    // if (m_distance_in <= -1) {
    //   distanceLeftOver_perc *= Math.abs(distanceLeftOver_perc);
    //   driveSpeed *= -1;
    // }

    // if (currentEncoderPos_in[1] != m_startingEncoderPos_in[1]) {
      // if (0.05 < distanceLeftOver_perc && 0.25 > distanceLeftOver_perc) {
      //   driveSpeed = (distanceLeftOver_perc * 3.25) - 0.1125;
      // } else if (0.1 <= distanceLeftOver_perc && distanceLeftOver_perc <= 0.9) {
      //   driveSpeed = 0.7;
      // } else if (0.6 < distanceLeftOver_perc && 0.95 >= distanceLeftOver_perc) {
      //   driveSpeed = (distanceLeftOver_perc * -2.16667) + 2.10833;
      // } else if (distanceLeftOver_perc > 0.95) {
      //   driveSpeed = 0.05;
      // }
      // driveSpeed = driveSpeed * 0.00714286 - 0.309524;
   // }

    m_drivetrain.driveWithGyro(0, driveSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveWithGyro(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(m_avgSubstractedDistance_in - m_distance_in) < 1;
    return distanceLeftOver_perc >= 1 || distanceLeftOver_perc < -0;
  }
}

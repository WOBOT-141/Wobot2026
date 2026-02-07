// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IBotSelector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.lib.SpektrumFlightController;

public class ConsolidatedStickCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drive;
  private final IGyro m_gyro;

  private final SpektrumFlightController m_flightController;
  double lastRightDirectional = 0.0;
  public SlewRateLimiter rampRate;
  public boolean lockdown = false;
  public boolean isRotateCenter = false;
  IBotSelector selectedBot;


  /**
   *
   * @param drive
   *          The subsystem used by this command.
   * @param gyro
   *          TODO
   */
  public ConsolidatedStickCommand(Drivetrain drive, IGyro gyro, SpektrumFlightController m_flightController) {
    m_drive = drive;
    m_gyro = gyro;
    addRequirements(drive);

    this.m_flightController = m_flightController;
    rampRate = new SlewRateLimiter(1);
    // selectedBot = IBotSelector.getSelected();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    stickDebug();

    // Calculate the magnitudes and directions
    double rightMagniude = Math
        .sqrt(Math.pow(m_flightController.getDriveX(), 2) + Math.pow(m_flightController.getDriveY(), 2));
    double rightDirectional = Math
        .toDegrees(Math.atan2(m_flightController.getDriveX(), m_flightController.getDriveY()));
    double leftDirectional = Math
        .toDegrees(Math.atan2(m_flightController.getRotate(), m_flightController.getThrottle()));
    double throttleValue = (((m_flightController.getThrottle() + 1) / 2) + .1);

    // Clamp Angle
    rightDirectional = (rightDirectional + 360) % 360;
    leftDirectional = (leftDirectional + 360) % 360;

    // subtract gyro from rightDirectional
    rightDirectional = (rightDirectional - m_gyro.getRealGyroAngle());

    // Magnitude for the movement of the bot on the right stick
    double finalLinearMagnitude = rightMagniude * throttleValue;
    if (finalLinearMagnitude > 1) {
      finalLinearMagnitude = 1;
    } else if (finalLinearMagnitude < -1) {
      finalLinearMagnitude = -1;
    }

    double rampedMagnitude = rampRate.calculate(finalLinearMagnitude);
    double finalRotationalMagnitude = m_flightController.getRotate() * throttleValue;
    if (finalRotationalMagnitude > 1) {
      finalRotationalMagnitude = 1;
    } else if (finalRotationalMagnitude < -1) {
      finalLinearMagnitude = -1;
    }

    if (m_flightController.getSelectPressButton()) {
      lockdown = true;
    }

    // Locks the wheels
    if (lockdown) {
      m_drive.lockWheels();
      if (finalLinearMagnitude != 0 || finalRotationalMagnitude != 0) {
        lockdown = false;
        System.out.println("lockdown");
      }
      System.out.println("working");
    } else {
      // TODO add this to its own method maybe like stick debug is?
      // SmartDashboard.putNumber("Final Linear Magnitude", finalLinearMagnitude);
      // SmartDashboard.putNumber("Ramped Magnitude: ", rampedMagnitude);
      // SmartDashboard.putNumber("Final Rotational Magnitude: ",
      // finalRotationalMagnitude);
      // SmartDashboard.putNumber("Right Directional: ", rightDirectional);
      // SmartDashboard.putNumber("Last Directional", lastRightDirectional);
      if (finalLinearMagnitude != 0 || finalRotationalMagnitude != 0) {

        double yDist = isRotateCenter ? selectedBot.getDistToFront() : 0;

        m_drive.driveWithGyro(rightDirectional, rampedMagnitude, finalRotationalMagnitude, 0, yDist);
        lastRightDirectional = rightDirectional;
      } else {
        rampRate.reset(0);
        m_drive.driveWithGyro(lastRightDirectional, 0, 0);
      }
    }

  }
  public void isRotateCenter(boolean bool) {
    isRotateCenter = bool;
  }
  public void stickDebug() {
    SmartDashboard.putNumber("STICK X: ", m_flightController.getDriveX());
    SmartDashboard.putNumber("STICK Y: ", m_flightController.getDriveY());
    SmartDashboard.putNumber("STICK ROT: ", m_flightController.getRotate());
    SmartDashboard.putNumber("STICK THROT: ", m_flightController.getThrottle());
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
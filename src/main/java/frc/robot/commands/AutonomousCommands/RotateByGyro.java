// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;

public class RotateByGyro extends Command {
  Drivetrain drive;
  IGyro gyro;

  double wantedAngle;
  double speed;
  boolean botOriented;
  double currentAngle;
  double deltaCurrent;
  double finalAngle;
  double smallestDelta;
  PIDController turnPID;

  public RotateByGyro(Drivetrain drive, IGyro gyro, double speed, double wantedAngle, boolean botOriented) {
    this.drive = drive;
    this.gyro = gyro;
    this.wantedAngle = wantedAngle;
    this.speed = speed;
    this.botOriented = botOriented;

    turnPID = new PIDController(Constants.RotateCommandConstants.P, Constants.RotateCommandConstants.I, Constants.RotateCommandConstants.D);
    turnPID.setTolerance(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnPID.reset();
    currentAngle = gyro.getRawGyroAngle();
    finalAngle = currentAngle + wantedAngle;
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("current", currentAngle);
    SmartDashboard.putNumber("final", finalAngle);
    currentAngle = gyro.getRawGyroAngle();
    double rotationalOutput = turnPID.calculate(currentAngle, finalAngle) /90d;
    if(rotationalOutput > 1){
      rotationalOutput = 1;
    }else if(rotationalOutput < -1){
      rotationalOutput = -1;
    }
    rotationalOutput *= speed;
    SmartDashboard.putNumber("Rotational output: ", rotationalOutput);
    System.out.println("PID output----------- " + rotationalOutput);

    drive.driveWithoutGyro(0,0, rotationalOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double degreeDifference = Math.abs(wantedAngle - currentAngle) % 360;
    if(degreeDifference < 3.5){
      return true;
    }else{
      return false;
    }
  }
}
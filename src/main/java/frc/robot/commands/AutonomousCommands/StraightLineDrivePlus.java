// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveCommandConstants;
import frc.robot.Constants.RotateVisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.SwerveModule;

public class StraightLineDrivePlus extends Command {

  private final Drivetrain m_drivetrain;
  private final double m_translationAngle;
  private final double m_distance_in;
  private final double m_rotationAmount_deg;
  private final SwerveModule[] m_wheels;
  private final IGyro m_gyro;

  private double[] m_startingEncoderPos_in;

  private double m_distanceTraveled_in;
  private double m_targetHeading_deg;

  /* Motion profile */

  private TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_goal;
  private TrapezoidProfile.State m_setpoint;

  private double MAX_VELOCITY;//5.8
  private double MAX_ACCEL;//11.6

  /* Distance PID */

  private PIDController m_distancePID;

  private static final double kDistanceP = DriveCommandConstants.P; //0.015
  private static final double kDistanceI = DriveCommandConstants.I;
  private static final double kDistanceD = DriveCommandConstants.D; //0.001

  /* Heading PID */

  private PIDController m_headingPID;

  private static final double kHeadingP = RotateVisionConstants.P;
  private static final double kHeadingI = RotateVisionConstants.I;
  private static final double kHeadingD = RotateVisionConstants.D;

  /* Timer for accurate loop timing */

  private final Timer m_timer = new Timer();
  private double m_lastTime;

  public StraightLineDrivePlus(Drivetrain drivetrain, double translationAngle, double distance, double rotationAmount_deg, SwerveModule wheels[], IGyro gyro, double maxSpeed) {

    m_drivetrain = drivetrain;
    m_translationAngle = translationAngle;
    m_distance_in = distance;
    m_wheels = wheels;
    m_gyro = gyro;
    MAX_VELOCITY = maxSpeed;
    MAX_ACCEL = maxSpeed + 1;
    m_rotationAmount_deg = rotationAmount_deg;
    m_startingEncoderPos_in = new double[m_wheels.length];

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    System.out.println("START /////////////////");
    for (int i = 0; i < m_wheels.length; i++) {
      m_startingEncoderPos_in[i] = m_wheels[i].getEncoderDistance();
    }

    m_targetHeading_deg = m_gyro.getYaw() + m_rotationAmount_deg; //m_targetHeading_deg = m_gyro.getYaw();

    /* Motion profile setup */

    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL);
    System.out.printf("Max Velocity:", MAX_VELOCITY);
    System.out.printf("Max Acceleration:", MAX_ACCEL);

    m_goal = new TrapezoidProfile.State(Math.abs(m_distance_in), 0);
    m_setpoint = new TrapezoidProfile.State(0, 0);
    m_profile = new TrapezoidProfile(constraints);

    m_distancePID = new PIDController(kDistanceP, kDistanceI, kDistanceD);
    m_distancePID.setTolerance(1);

    m_headingPID = new PIDController(kHeadingP, kHeadingI, kHeadingD);
    m_headingPID.enableContinuousInput(-180, 180);
    m_headingPID.setTolerance(0.5);

    /* Start timer */
    m_timer.reset();
    m_timer.start();
    m_lastTime = m_timer.get();
  }

  @Override
  public void execute() {
    System.out.printf("Max Velocity:%.3f\n", MAX_VELOCITY);
    System.out.printf("Max Acceleration:%.3f\n", MAX_ACCEL);

    double totalDistance = 0;

    for (int i = 0; i < m_wheels.length; i++) {

      double current = m_wheels[i].getEncoderDistance();
      double diff = Math.abs(m_startingEncoderPos_in[i] - current);

      totalDistance += diff;
    }

    m_distanceTraveled_in = totalDistance / m_wheels.length;

    /* Calculate real loop time */

    double currentTime = m_timer.get();
    double dt = currentTime - m_lastTime;
    m_lastTime = currentTime;

    /* Motion profile */

    m_setpoint = m_profile.calculate(dt, m_setpoint, m_goal);

    double profileVelocity = m_setpoint.velocity;
    double distanceCorrection = m_distancePID.calculate(m_distanceTraveled_in, m_goal.position); //m_goal.position
    double headingCorrection = m_headingPID.calculate(m_gyro.getYaw(), m_targetHeading_deg);

    double speed = distanceCorrection + profileVelocity;
    System.out.printf("profileVelocity:%.3f\n", profileVelocity);
    System.out.printf("distanceCorrection:%.3f\n", distanceCorrection);
    System.out.printf("Speed:%.3f\n", speed);

    SmartDashboard.putNumber("Distance Traveled", m_distanceTraveled_in);
    SmartDashboard.putNumber("Profile Velocity", profileVelocity);
    SmartDashboard.putNumber("Distance Correction", distanceCorrection);
    SmartDashboard.putNumber("Heading Correction", headingCorrection);
    SmartDashboard.putNumber("Final Speed", speed);

    m_drivetrain.driveWithGyro(m_translationAngle, profileVelocity, headingCorrection,true); //speed
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("DONE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    m_drivetrain.driveWithGyro(0, 0, 0,true);
  }

  @Override
  public boolean isFinished() {

    // return  m_headingPID.atSetpoint();
    return m_distancePID.atSetpoint();
  }
}
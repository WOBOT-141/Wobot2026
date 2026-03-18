package frc.robot.commands.DriveCommands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.IBotSelector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;

public class StickCommand extends Command {

  private final Drivetrain m_drive;
  private final IBotSelector m_selectedBot;

  private final DoubleSupplier m_driveXSupplier;
  private final DoubleSupplier m_driveYSupplier;
  private final DoubleSupplier m_rotateSupplier;
  private final DoubleSupplier m_throttleSupplier;

  private final SlewRateLimiter m_rampRate = new SlewRateLimiter(1.0);

  private double m_lastRightDirectional = 0.0;
  private boolean m_isRotateCenter = false;
  private boolean m_lockdown = false;
  private Optional<Double> m_altCommandedRotation_perc = Optional.empty();

  private int m_dashboardCounter = 0;
  private static final int DASHBOARD_DIVIDER = 10;

  public StickCommand(
      Drivetrain drive,
      IGyro gyro,
      IBotSelector selectedBot,
      DoubleSupplier driveXSupplier,
      DoubleSupplier driveYSupplier,
      DoubleSupplier rotateSupplier,
      DoubleSupplier throttleSupplier) {

    m_drive = drive;
    m_selectedBot = selectedBot;

    m_driveXSupplier = driveXSupplier;
    m_driveYSupplier = driveYSupplier;
    m_rotateSupplier = rotateSupplier;
    m_throttleSupplier = throttleSupplier;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_rampRate.reset(0);
    m_lockdown = false;
  }

  @Override
  public void execute() {

    if (++m_dashboardCounter % DASHBOARD_DIVIDER == 0) {
      stickDebug();
    }

    double stickX_perc = applyDeadband(m_driveXSupplier.getAsDouble(), 0.05);
    double stickY_perc = applyDeadband(m_driveYSupplier.getAsDouble(), 0.05);
    double stickRotate_perc = applyDeadband(m_rotateSupplier.getAsDouble(), 0.05);
    double stickThrottle_perc = m_throttleSupplier.getAsDouble();

    double throttle_perc = calcThrottle_perc(stickThrottle_perc);

    boolean isStopped =
        Math.abs(stickX_perc) < 0.01 &&
        Math.abs(stickY_perc) < 0.01 &&
        Math.abs(stickRotate_perc) < 0.01;

    if (isStopped) {
      m_drive.driveWithGyro(m_lastRightDirectional, 0.0, 0.0);
      return;
    }
   
    double translationMagnitude_perc = throttle_perc * calcTranslationMagnitude_perc(stickX_perc, stickY_perc);
    double translationAngle_deg = calcTranslationAngle_deg(stickX_perc, stickY_perc);
    double rotationalMagnitude_perc = throttle_perc * stickRotate_perc;

    translationMagnitude_perc = m_rampRate.calculate(translationMagnitude_perc);
    translationMagnitude_perc = Math.max(-1.0, Math.min(1.0, translationMagnitude_perc));
    rotationalMagnitude_perc = Math.max(-1.0, Math.min(1.0, rotationalMagnitude_perc));

   if (translationMagnitude_perc != 0 || rotationalMagnitude_perc != 0) {
      m_lockdown = false;
    }

    if (m_lockdown) {
      m_drive.lockWheels();
      return;
    }

    if (rotationalMagnitude_perc == 0 && m_altCommandedRotation_perc.isPresent()) {
      rotationalMagnitude_perc = m_altCommandedRotation_perc.get();
    }

   double yDist = m_isRotateCenter ? m_selectedBot.getDistToFront() : 0.0;

    m_drive.driveWithGyro(
        translationAngle_deg,
        translationMagnitude_perc,
        rotationalMagnitude_perc,
        0.0,
        yDist);

    if (translationMagnitude_perc > 0.05) {
      m_lastRightDirectional = translationAngle_deg;
    }
  }

  private double applyDeadband(double value, double deadband) {
    return Math.abs(value) < deadband ? 0.0 : value;
  }

  private double calcThrottle_perc(double stickThrottle_perc) {
    double throttle = ((stickThrottle_perc + 1) / 2.0) + 0.1;

    if (m_altCommandedRotation_perc.isPresent()) {
      throttle *= 0.5;
    }

    return Math.min(throttle, 1.0); // clamp
  }

  private double calcTranslationMagnitude_perc(double stickX_perc, double stickY_perc) {
    double xPrime = stickX_perc * Math.sqrt(1 - (Math.pow(stickY_perc, 2) / 2));
    double yPrime = stickY_perc * Math.sqrt(1 - (Math.pow(stickX_perc, 2) / 2));

    return Math.sqrt(xPrime * xPrime + yPrime * yPrime);
  }

  private double calcTranslationAngle_deg(double stickX_perc, double stickY_perc) {
    double angle = Math.toDegrees(Math.atan2(stickX_perc, stickY_perc));
    return (angle + 360.0) % 360.0;
  }

  public void setFrontDrive(boolean isTrue) {
    m_isRotateCenter = isTrue;
  }

  public void setAlternateRotation(Optional<Double> altRotation_perc) {
    m_altCommandedRotation_perc = altRotation_perc;
  }

  public void lockWheels() {
    m_lockdown = true;
  }

  private void stickDebug() {
    SmartDashboard.putNumber("STICK X", m_driveXSupplier.getAsDouble());
    SmartDashboard.putNumber("STICK Y", m_driveYSupplier.getAsDouble());
    SmartDashboard.putNumber("STICK ROT", m_rotateSupplier.getAsDouble());
    SmartDashboard.putNumber("STICK THROTTLE", m_throttleSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.driveWithGyro(m_lastRightDirectional, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooterIO.ShooterIO;
import frc.robot.subsystems.shooterIO.ShooterIOInputsAutoLogged;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double m_targetVelocity_ips = 0.0;
  private double m_hoodTargetPosition_deg = ShooterConstants.hoodLowerLimit_deg;
  private boolean m_hoodAtHome = false;

  // Shot detection
  private double m_lastAvgRPM_rpm = 0.0;
  private boolean m_shotDetected = false;
  private double m_lastShotTime_sec = -1.0;

  private String m_hoodPosition_SD = "SETHOODPOS!!"; // SmartDashboard key for hood position

  public Shooter(ShooterIO io) {
    this.io = io;

    SmartDashboard.putNumber("hoodP", ShooterConstants.hoodP);
    SmartDashboard.putNumber("hoodI", ShooterConstants.hoodI);
    SmartDashboard.putNumber("hoodD", ShooterConstants.hoodD);
    SmartDashboard.putNumber(m_hoodPosition_SD, m_hoodTargetPosition_deg);
  }

  public void homeHood() {
    stopHood();
    m_hoodAtHome = true;
  }

  public boolean isHoodAtHome() {
    SmartDashboard.putNumber("homePos", inputs.hoodAngleDeg);
    return m_hoodAtHome;
  }

  public void setHoodAtHome(boolean atHome) {
    m_hoodAtHome = atHome;
  }

  public void setHoodAngle(double targetPosition_deg) {
    targetPosition_deg = Math.max(targetPosition_deg, ShooterConstants.hoodLowerLimit_deg);
    targetPosition_deg = Math.min(targetPosition_deg, ShooterConstants.hoodUpperLimit_deg);
    m_hoodTargetPosition_deg = targetPosition_deg;

    io.setHoodAngle(m_hoodTargetPosition_deg);
  }

  public void setHoodPositionFromDashboard() {
    double targetPosition_deg = SmartDashboard.getNumber(m_hoodPosition_SD, m_hoodTargetPosition_deg);
    
    setHoodAngle(targetPosition_deg);
  }
  
  public void setPIDValues(double hoodP, double hoodI, double hoodD) {
  }

  public void setShooterVelocity(double targetVelocity_ips) {
    if (targetVelocity_ips == m_targetVelocity_ips)
      return;

    m_targetVelocity_ips =  targetVelocity_ips*ShooterConstants.flywheelSlipFactor;

    io.setFlywheelVelocityIps(m_targetVelocity_ips);
  }

  // Speed checks
  public boolean rightAtSpeed() {
    return Math.abs(inputs.rightVelocityIps - m_targetVelocity_ips) < ShooterConstants.flywheelVelocityTolerance_ips;
  }

  public boolean middleAtSpeed() {
    return Math.abs(inputs.middleVelocityIps - m_targetVelocity_ips) < ShooterConstants.flywheelVelocityTolerance_ips;
  }

  public boolean leftAtSpeed() {
    return Math.abs(inputs.leftVelocityIps - m_targetVelocity_ips) < ShooterConstants.flywheelVelocityTolerance_ips;
  }

  public boolean allAtSpeed() {
    return rightAtSpeed() && middleAtSpeed() && leftAtSpeed();
  }

  public boolean hoodAtPosition(){
    return Math.abs(inputs.hoodAngleDeg - m_hoodTargetPosition_deg) < ShooterConstants.hoodPositionTolerance_deg;
  }

  public boolean isReadyToShoot() {
    return allAtSpeed() && hoodAtPosition();
  }
  
  public double getAverageVelocity_ips() {
    return (inputs.rightVelocityIps + inputs.middleVelocityIps + inputs.leftVelocityIps) / 3.0 ;
  }

  // Shot detection via RPM dip
  public boolean updateShotDetection() {
    // TODO convert to ips
    double currentRPM_rpm = getAverageVelocity_ips();
    double now_sec = Timer.getFPGATimestamp();

    m_shotDetected = false;

    if ((m_lastAvgRPM_rpm - currentRPM_rpm) > ShooterConstants.shotDipThreshold_rpm) {

      if (m_lastShotTime_sec < 0 ||
          (now_sec - m_lastShotTime_sec) > ShooterConstants.shotMinSeparation_sec) {

        m_shotDetected = true;
        m_lastShotTime_sec = now_sec;
      }
    }

    m_lastAvgRPM_rpm = currentRPM_rpm;
    return m_shotDetected;
  }

  public double getLastShotTime_sec() {
    return m_lastShotTime_sec;
  }

  public void cancel() {
    io.stop();
    m_targetVelocity_ips = 0;
    m_hoodTargetPosition_deg = ShooterConstants.hoodLowerLimit_deg;
  }

  public void stopHood() {
    io.setHoodAngle(inputs.hoodAngleDeg);
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    
    Logger.processInputs("Shooter", inputs);

    updateShotDetection();
    
    SmartDashboard.putNumber("Shooter Commanded IPS", m_targetVelocity_ips);
    SmartDashboard.putNumber("Shooter Actual IPS", getAverageVelocity_ips());
    SmartDashboard.putBoolean("ShooterHomed", m_hoodAtHome);

    SmartDashboard.putNumber("Hood Commanded Position", m_hoodTargetPosition_deg);
    SmartDashboard.putNumber("Hood Actual Position", inputs.hoodAngleDeg);
    SmartDashboard.putNumber("Hood Current", inputs.hoodCurrentAmps);
    SmartDashboard.putNumber("Shooter Avg RPM", getAverageVelocity_ips());
    SmartDashboard.putBoolean("Shot Detected", m_shotDetected);

    SmartDashboard.putNumber("Right motor RPM", inputs.rightVelocityIps);
    SmartDashboard.putNumber("Middle motor RPM", inputs.middleVelocityIps);
    SmartDashboard.putNumber("Left motor RPM", inputs.leftVelocityIps);
  }
}
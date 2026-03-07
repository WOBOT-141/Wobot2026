package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class SetPIDShooter extends InstantCommand {

  private final Shooter m_shooter;
  private double m_flywheelP0 = 0;
  private double m_flywheelI0 = 0;
  private double m_flywheelD0 = 0;


  public SetPIDShooter(Shooter shooter) {
    m_shooter = shooter;

    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    double m_flywheelP = SmartDashboard.getNumber("Shooter P", m_flywheelP0);
    double m_flywheelI = SmartDashboard.getNumber("Shooter I", m_flywheelI0);
    double m_flywheelD = SmartDashboard.getNumber("Shooter D", m_flywheelD0);
    System.out.printf("SetPidShooter start FlyWheelP: %f, FlyWheelI: %f, FlyWheelD: %f\n", m_flywheelP, m_flywheelI, m_flywheelD);
    
    m_shooter.setPIDValues(m_flywheelP, m_flywheelI, m_flywheelD);
  } 

    @Override
    public void end(boolean interrupted) {
    System.out.println("SetPidShooter End");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}



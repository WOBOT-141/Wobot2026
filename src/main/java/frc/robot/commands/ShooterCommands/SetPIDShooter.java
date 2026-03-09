package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class SetPIDShooter extends InstantCommand {

  private final Shooter m_shooter;
  private double m_hoodD0 = 0;
  private double m_hoodI0 = 0;
  private double m_hoodP0 = 0;


  public SetPIDShooter(Shooter shooter) {
    m_shooter = shooter;

    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    double m_hoodP = SmartDashboard.getNumber("Shooter P", m_hoodP0);
    double m_hoodI = SmartDashboard.getNumber("Shooter I", m_hoodI0);
    double m_hoodD = SmartDashboard.getNumber("Shooter D", m_hoodD0);
    System.out.printf("SetPidShooter start HoodP: %f, HoodI: %f, HoodD: %f\n", m_hoodP, m_hoodI, m_hoodD);
    
    m_shooter.setPIDValues(m_hoodP, m_hoodI, m_hoodD);
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



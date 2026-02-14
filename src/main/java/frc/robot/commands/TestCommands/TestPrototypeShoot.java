// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PrototypeShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestPrototypeShoot extends InstantCommand {
  private PrototypeShooter m_testShooter;
    private double m_shooterRPM;
    
    /** Creates a new DriveInDirection. */
    public TestPrototypeShoot(PrototypeShooter shooter, double shootRPM ) {
      m_shooterRPM = shootRPM;
      m_testShooter = shooter;
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      System.out.println("TestPrototypeShoot has Started");
    }
      
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      m_testShooter.setShooterRPM(m_shooterRPM, m_shooterRPM);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_testShooter.cancel();
    System.out.println("TestPrototypeShoot has ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class RotateToHub extends Command {
    private final Drivetrain m_drivetrain;
    private final Double m_translationX;
    private final Double m_translationY;
    
    private final PIDController m_thetaController = new PIDController(Constants.RotateCommandConstants.P,Constants.RotateCommandConstants.I,Constants.RotateCommandConstants.D);

    public RotateToHub(Drivetrain drivetrain, Double translationX, Double translationY) {
        m_drivetrain = drivetrain;
        m_translationX = translationX;
        m_translationY = translationY;
        
        m_thetaController.enableContinuousInput(-180, 180);
        
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        double dx = m_drivetrain.getHubX() - m_drivetrain.getPoseX();
        double dy = m_drivetrain.getHubY() - m_drivetrain.getPoseY();
        
        Rotation2d targetRotation = new Rotation2d(dx, dy);
        double targetAngle = targetRotation.getDegrees();

        double currentAngle = m_drivetrain.getPoseRotation().getDegrees();
        double rotationOutput = m_thetaController.calculate(currentAngle, targetAngle);

        m_drivetrain.driveWithGyro(
            m_translationX, 
            m_translationY, 
            rotationOutput
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
    }
}
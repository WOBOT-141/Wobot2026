package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.SpektrumFlightController;
import frc.robot.subsystems.Drivetrain;

// Note: right now, controls are backwards when using this command.  Prefer StickCommand for now.
// We will have to integrate the normalization into StickCommand.
public class EllipticalStickNormalization extends Command {
    private final SpektrumFlightController m_flightController;
    private final Drivetrain m_drivetrain;
    
    public EllipticalStickNormalization(SpektrumFlightController flightController, Drivetrain drivetrain) {
        m_flightController = flightController;
       m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double xAxis = m_flightController.getDriveX();
        double yAxis = m_flightController.getDriveY();
        double rotationSpeed = m_flightController.getRotate();
        double throttlePercentage = m_flightController.getThrottle();
        int exponent = 2;
        double xPrime = xAxis*Math.sqrt(1-(Math.pow(yAxis, exponent)/2));
        double yPrime = yAxis*Math.sqrt(1-(Math.pow(xAxis, exponent)/2));

        throttlePercentage = throttlePercentage*0.5+0.5;
        // throttlePercentage = Math.max(throttlePercentage*0.5+0.5, 0);
        double linearSpeed = Math.sqrt(Math.pow(xPrime, exponent)+Math.pow(yPrime, exponent))*throttlePercentage;
        double linearAngle = Math.atan2(xPrime, yPrime)*180/Math.PI;
        linearAngle = (linearAngle+360)%360;
        rotationSpeed = rotationSpeed*throttlePercentage;
        
        m_drivetrain.driveWithGyro(linearAngle, linearSpeed, rotationSpeed);
        /* TO DO LIST:
         * 1. Anything else we need to do that we find out in the middle of coding/other stuff
         */
    }
}

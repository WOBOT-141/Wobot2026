package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.SpektrumFlightController;
import frc.robot.subsystems.Drivetrain;

// Note: right now, controls are backwards when using this command.  Prefer StickCommand for now.
// We will have to integrate the normalization into StickCommand.
public class NormalizedStickCommand extends Command {
    private final SpektrumFlightController _flightController;
    private final Drivetrain _driveSubsystem;
    
    public NormalizedStickCommand(SpektrumFlightController flightController, Drivetrain driveSubsystem) {
        _flightController = flightController;
        _driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double xAxis = _flightController.getDriveX();
        double yAxis = _flightController.getDriveY();
        double rotationSpeed = _flightController.getRotate();
        double throttlePercentage = _flightController.getThrottle();
        int exponent = 2;
        double xPrime = xAxis*Math.sqrt(1-(Math.pow(yAxis, exponent)/2));
        double yPrime = yAxis*Math.sqrt(1-(Math.pow(xAxis, exponent)/2));

        throttlePercentage = throttlePercentage*0.5+0.5;
        // throttlePercentage = Math.max(throttlePercentage*0.5+0.5, 0);
        double linearSpeed = Math.sqrt(Math.pow(xPrime, exponent)+Math.pow(yPrime, exponent))*throttlePercentage;
        double linearAngle = Math.atan2(xPrime, yPrime)*180/Math.PI;
        linearAngle = (linearAngle+360)%360;
        rotationSpeed = rotationSpeed*throttlePercentage;
        
        _driveSubsystem.driveSwerve(linearAngle, linearSpeed, rotationSpeed);
        /* TO DO LIST:
         * 1. Anything else we need to do that we find out in the middle of coding/other stuff
         */
    }
}

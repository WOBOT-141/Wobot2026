package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IGyro;

/**
 * A one-shot command that resets/zeros the gyro heading (yaw).
 */
public class ResetGyro extends Command {

    private final IGyro gyro;

    public ResetGyro(IGyro gyro) {
        this.gyro = gyro;
        // Optional: addRequirements() if you want to require the gyro subsystem
        // addRequirements(gyro);  // if IGyro extends SubsystemBase
    }

    @Override
    public void initialize() {
        gyro.zeroYaw(); // use this to set the zero for field orientented operations
    }

    @Override
    public boolean isFinished() {
        return true;        // one-shot command — finishes immediately after initialize()
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;        // Allow resetting gyro even when robot is disabled
    }
}
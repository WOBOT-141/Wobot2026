// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode,
 * as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    public Robot() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // set up subsystems, and configure default commands.
        m_robotContainer = new RobotContainer();
    }

    /**
     * This function is called every 20 ms, no matter the mode.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler — responsible for command execution
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // Optional: reset or zero things if needed
    }

    @Override
    public void disabledPeriodic() {
        // wheelSetup() is no longer needed — offsets are set in constructors
        // You can remove this call or leave it empty
    }

    /**
     * This autonomous runs the command selected by your RobotContainer.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command if it exists
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        // usually empty — commands handle periodic behavior
    }

    @Override
    public void teleopInit() {
        // Cancel any leftover autonomous command
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // The default command (StickCommand) is already set in RobotContainer
        // No need to call stickCommandInit() anymore

        // Optional: log controller connection if you still want to check it
        if (RobotContainer.flightController != null && RobotContainer.flightController.isConnected()) {
            System.out.println("Flight controller is connected");
        } else {
            System.out.println("Warning: Flight controller not detected");
        }
    }

    @Override
    public void teleopPeriodic() {
        // usually empty — default command handles teleop driving
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
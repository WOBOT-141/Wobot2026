// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.SpektrumFlightController;
import frc.robot.Constants.VisionServerConstants;
import frc.robot.commands.DriveCommands.DriveToPose;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.IStickParameters;
import frc.robot.commands.StickParameters.StickBParameters;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Vision.AprilTags.RebuiltAprilTags;
import frc.robot.subsystems.Vision.VisionServer;

public class RobotContainer {

    private final SwerveModule[] m_wheels;
    private final Drivetrain m_drive;
    private final IGyro m_gyro = new NavxGyro();

    private final SpektrumFlightController m_flightController0 = new SpektrumFlightController(0);
    private final CommandXboxController m_xboxController0 = new CommandXboxController(1);
    // private final CommandJoystick m_joystick = new CommandJoystick(2);
    private final StickCommand m_stickCommand;

    private final IBotSelector m_selectedBot = new MainBot();

        private final VisionServer m_visionServer = new VisionServer(VisionServerConstants.NumberOfCameras,
            RebuiltAprilTags.Tags.length);
    // private final IBotSelector m_selectedBot = new SummerBot();

    // Merging AdvantageKit Logged Chooser with the populated routines from main
    private final LoggedDashboardChooser<Command> m_autonChooser = new LoggedDashboardChooser<>("Auto Routine");

    public RobotContainer() {

        m_wheels = new SwerveModule[] {

                new SwerveModule(
                        Constants.CANIdConstants.frontLeftSteerMotorID,
                        Constants.CANIdConstants.frontLeftDriveMotorID,
                        Constants.CANIdConstants.frontLeftEncoderid,
                        m_selectedBot.getfrontLeftOffset(),
                        m_selectedBot.getfrontLeftLocationX(),
                        m_selectedBot.getfrontLeftLocationY(),
                        m_selectedBot.rotationsPerInch_rot())
        };

        m_drive = new Drivetrain(m_wheels, m_gyro, m_visionServer);

        configureFlightControllerParameters();

        m_stickCommand = new StickCommand(
                m_drive,
                m_gyro,
                m_selectedBot,
                () -> m_flightController0.getDriveX(),
                () -> m_flightController0.getDriveY(),
                () -> m_flightController0.getRotate(),
                () -> m_flightController0.getThrottle());

        m_drive.setDefaultCommand(m_stickCommand);

        configureBindings();
    }

    /**
     * Restore flight controller scaling, offsets, and deadbands.
     */
    private void configureFlightControllerParameters() {

        IStickParameters p = new StickBParameters();

        m_flightController0.setRotateFactor(p.get_RotateFactor());
        m_flightController0.setDriveXFactor(p.get_DriveXFactor());
        m_flightController0.setDriveYFactor(p.get_DriveYFactor());
        m_flightController0.setThrottleFactor(p.get_ThrottleFactor());

        m_flightController0.setRotateOffset(p.get_RotateOffset());
        m_flightController0.setDriveXOffset(p.get_DriveXOffset());
        m_flightController0.setDriveYOffset(p.get_DriveYOffset());
        m_flightController0.setThrottleOffset(p.get_ThrottleOffset());

        m_flightController0.setKnobOffset(p.get_knobOffset());
        m_flightController0.setxDeadBand(p.get_deadbandX());
        m_flightController0.setyDeadBand(p.get_deadbandY());
        m_flightController0.setRotationDeadBand(p.get_deadbandRot());
    }

    /**
     * Re-zero swerve modules while disabled.
     */
    public void wheelSetup() {

        m_wheels[0].setEncoderOffset(m_selectedBot.getfrontLeftOffset());
        m_wheels[0].set_xPos(m_selectedBot.getfrontLeftLocationX());
        m_wheels[0].set_yPos(m_selectedBot.getfrontLeftLocationY());

        m_wheels[1].setEncoderOffset(m_selectedBot.getbackLeftOffset());
        m_wheels[1].set_xPos(m_selectedBot.getbackLeftLocationX());
        m_wheels[1].set_yPos(m_selectedBot.getbackLeftLocationY());

        m_wheels[2].setEncoderOffset(m_selectedBot.getfrontRightOffset());
        m_wheels[2].set_xPos(m_selectedBot.getfrontRightLocationX());
        m_wheels[2].set_yPos(m_selectedBot.getfrontRightLocationY());

        m_wheels[3].setEncoderOffset(m_selectedBot.getbackRightOffset());
        m_wheels[3].set_xPos(m_selectedBot.getbackRightLocationX());
        m_wheels[3].set_yPos(m_selectedBot.getbackRightLocationY());

        for (int i = 0; i < 4; i++) {
            m_wheels[i].settoZero();
        }
    }

    private void configureBindings() {
        configureFlightController();
    }


    private void configureFlightController() {
        new Trigger(() -> m_flightController0.getRawButton(SpektrumFlightController.Button.kReset.value))
                .onTrue(new ResetGyro(m_gyro));
        new Trigger(() -> m_flightController0.getRawButton(SpektrumFlightController.Button.kSelectPress.value))
                .onTrue(Commands.runOnce(() -> m_stickCommand.lockWheels()));

        // Preserving HEAD's implementation of the IButton Pose reset, but retaining
        // main's comments just in case
        new Trigger(() -> m_flightController0.getIButton())
                // .onTrue(Commands.runOnce(() -> m_drive.resetPose(new Pose2d())));
                .whileTrue(Commands.runOnce(() -> new DriveToPose(m_drive, () -> new Pose2d(0, 0, new Rotation2d(0)))));
        // new Trigger(() -> m_flightController0.getIButton())
        // .whileTrue(m_intake.extendIntakeCommand(-IntakeConstants.intakeExtendSpeed_perc));
        // .whileTrue(new AimAtHubCommand(m_stickCommand, m_visionServer, m_shooter,
        // m_gyro, m_aprilTagFilter, m_xboxController0));
        // .whileTrue(new MovingRotateToTag(m_visionServer, m_stickCommand));
    }

    public Command getAutonomousCommand() {
        // LoggedDashboardChooser uses .get() instead of WPILib's getSelected()
        Command selectedCommand = m_autonChooser.get();

        if (selectedCommand != null) {
            System.out.println("Selected auton: " + selectedCommand.getName());
            return selectedCommand;
        }

        System.out.println("Selected auton: <null>");
        return Commands.none();
    }

}
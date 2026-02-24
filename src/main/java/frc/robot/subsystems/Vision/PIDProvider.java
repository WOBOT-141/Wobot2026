package frc.robot.subsystems.Vision;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.





import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
/** Add your docs here. */
public class PIDProvider {
    public static String MAX_DRIVE = "MAX_DRIVE";
    public static String MAX_ROTATE = "MAX_ROTATE";

    public static String DRIVE_Y_P = "Drive_Y_P";
    public static String DRIVE_Y_I = "Drive_Y_I";
    public static String DRIVE_Y_D = "Drive_Y_D";

    public static String DRIVE_X_P = "Drive_X_P";
    public static String DRIVE_X_I = "Drive_X_I";
    public static String DRIVE_X_D = "Drive_X_D";

    public static String DRIVE_OMEGA_P = "Drive_OMEGA_P";
    public static String DRIVE_OMEGA_I = "Drive_OMEGA_I";
    public static String DRIVE_OMEGA_D = "Drive_OMEGA_D";

    public static String ROTATE_P = "Rotate_P";
    public static String ROTATE_I = "Rotate_I";
    public static String ROTATE_D = "Rotate_D";

    public static String DRIVE_P = "Drive_P";
    public static String DRIVE_I = "Drive_I";
    public static String DRIVE_D = "Drive_D";

    public static void Initialize() {
        SmartDashboard.putNumber(MAX_DRIVE, 0.5);
        SmartDashboard.putNumber(MAX_ROTATE, 0.3);

        // We should adjust to using 6:3:2 (Y:X:OMEGA)

        SmartDashboard.putNumber(DRIVE_Y_P, 0.008);
        SmartDashboard.putNumber(DRIVE_Y_I, 0);
        SmartDashboard.putNumber(DRIVE_Y_D, 0);

        SmartDashboard.putNumber(DRIVE_X_P, 0.001333);
        SmartDashboard.putNumber(DRIVE_X_I, 0.0001);
        SmartDashboard.putNumber(DRIVE_X_D, 0);

        SmartDashboard.putNumber(DRIVE_OMEGA_P, 0.0032);
        SmartDashboard.putNumber(DRIVE_OMEGA_I, 0.00003);
        SmartDashboard.putNumber(DRIVE_OMEGA_D, 0);

        SmartDashboard.putNumber(ROTATE_P, Constants.RotateCommandConstants.P);
        SmartDashboard.putNumber(ROTATE_I, Constants.RotateCommandConstants.I);
        SmartDashboard.putNumber(ROTATE_D, Constants.RotateCommandConstants.D);

        SmartDashboard.putNumber(DRIVE_P, Constants.DriveCommandConstants.P);
        SmartDashboard.putNumber(DRIVE_I, Constants.DriveCommandConstants.I);
        SmartDashboard.putNumber(DRIVE_D, Constants.DriveCommandConstants.D); 
    }


    public static double getMaxDrive() {
        return SmartDashboard.getNumber(MAX_DRIVE, 0.5);
    }

    public static double getMaxRotate() {
        return SmartDashboard.getNumber(MAX_ROTATE, 0.3);
    }

    public static double getDriveY_P() {
        return SmartDashboard.getNumber(DRIVE_Y_P, 0);
    }

    public static double getDriveY_I() {
        return SmartDashboard.getNumber(DRIVE_Y_I, 0);
    }

    public static double getDriveY_D() {
        return SmartDashboard.getNumber(DRIVE_Y_D, 0);
    }

    public static double getDriveX_P() {
        return SmartDashboard.getNumber(DRIVE_X_P, 0);
    }

    public static double getDriveX_I() {
        return SmartDashboard.getNumber(DRIVE_X_I, 0);
    }

    public static double getDriveX_D() {
        return SmartDashboard.getNumber(DRIVE_X_D, 0);
    }

    public static double getDriveOmega_P() {
        return SmartDashboard.getNumber(DRIVE_OMEGA_P, 0);
    }

    public static double getDriveOmega_I() {
        return SmartDashboard.getNumber(DRIVE_OMEGA_I, 0);
    }

    public static double getDriveOmega_D() {
        return SmartDashboard.getNumber(DRIVE_OMEGA_D, 0);
    }

    public static double getRotate_P() {
        return SmartDashboard.getNumber(ROTATE_P, 0);
    }

    public static double getRotate_I() {
        return SmartDashboard.getNumber(ROTATE_I, 0);
    }

    public static double getRotate_D() {
        return SmartDashboard.getNumber(ROTATE_D, 0);
    }

    public static double getDrive_P() {
        return SmartDashboard.getNumber(DRIVE_P, 0);
    }

    public static double getDrive_I() {
        return SmartDashboard.getNumber(DRIVE_I, 0);
    }

    public static double getDrive_D() {
        return SmartDashboard.getNumber(DRIVE_D, 0);
    }

}


package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {

    private final Wheel[] wheels;
    private final SwerveDriveKinematics kinematics;

    private boolean isTipping = false;
    private boolean climbing = false;
    private double tipMagnitude;
    private double tipDirection;
    private final double tipThreshold = 10; // degrees

    /**
     * Constructor – only takes the array of Wheel modules
     */
    public Drive(Wheel[] wheels) {
        this.wheels = wheels;

        // Build kinematics using wheel positions
        // Your coordinate system: right = +x, forward = +y
        // WPILib: forward = +x, left = +y → we convert accordingly
        Translation2d[] modulePositions = new Translation2d[wheels.length];
        for (int i = 0; i < wheels.length; i++) {
            double wpilibX = wheels[i].get_yPos();     // forward → x
            double wpilibY = -wheels[i].get_xPos();    // right → -y (left positive)
            modulePositions[i] = new Translation2d(wpilibX, wpilibY);
        }

        this.kinematics = new SwerveDriveKinematics(modulePositions);
    }

    public void setClimbing(boolean climb) {
        climbing = climb;
    }

    public void driveSwerve(double linearAngle_deg, double linearSpeed_mps, double rotationalSpeed_dps) {
        driveSwerve(linearAngle_deg, linearSpeed_mps, rotationalSpeed_dps, 0, 0);
    }

    public void driveSwerve(double linearAngle_deg, double linearSpeed_mps,
                             double rotationalSpeed_dps, double rotX, double rotY) {

        if (isTipping && !climbing) {
            linearAngle_deg = tipDirection;
            linearSpeed_mps = tipMagnitude / (tipThreshold * 2);
            rotationalSpeed_dps = 0;
            rotX = 0;
            rotY = 0;
        }

        SmartDashboard.putNumber("Linear Speed (m/s)", linearSpeed_mps);
        SmartDashboard.putNumber("Linear Angle (deg)", linearAngle_deg);
        SmartDashboard.putNumber("Rotational Speed (deg/s)", rotationalSpeed_dps);

        if (Math.abs(linearSpeed_mps) < 1e-6 && Math.abs(rotationalSpeed_dps) < 1e-6) {
            for (Wheel wheel : wheels) {
                wheel.cancelMovement();
            }
            return;
        }

        // Convert inputs to chassis speeds (robot coordinate frame)
        double vx = linearSpeed_mps * Math.sin(Math.toRadians(linearAngle_deg));   // right
        double vy = linearSpeed_mps * Math.cos(Math.toRadians(linearAngle_deg));   // forward
        double omegaRad = Math.toRadians(rotationalSpeed_dps);

        // Adjust for rotation center offset (if rotX, rotY are used)
        vx += omegaRad * rotY;   // right component from rotation
        vy -= omegaRad * rotX;   // forward component from rotation

        // WPILib coordinate system: +x forward, +y left, +omega CCW
        double wpilibVx = vy;             // forward
        double wpilibVy = -vx;            // left = -right
        double wpilibOmega = omegaRad;    // CCW positive

        ChassisSpeeds speeds = new ChassisSpeeds(wpilibVx, wpilibVy, wpilibOmega);

        // Calculate module states
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        // Desaturate (normalize) if any wheel exceeds max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DriveConstants.topSpeed_mps);

        // Send to each module with optimization
        for (int i = 0; i < wheels.length; i++) {
            SwerveModuleState optimized = wheels[i].bestAngle(states[i]);
            wheels[i].setAngle(optimized.angle.getDegrees());
            wheels[i].setSpeed(optimized.speedMetersPerSecond);
        }
    }

    public void lockWheels() {
        // X-lock pattern (most common for defense)
        // Adjust angles based on your wheel order: FL, FR, BL, BR
        double[] lockAngles = {45, -45, -45, 45};
        for (int i = 0; i < wheels.length; i++) {
            wheels[i].setAngle(lockAngles[i]);
            wheels[i].setSpeed(0);
        }
    }

    @Override
    public void periodic() {
        // Update tipping state here if you have gyro access
        // Example (uncomment when you add gyro back to commands):
        // if (gyro != null) {
        //     double pitch = gyro.getPitch();
        //     isTipping = Math.abs(pitch) > tipThreshold;
        //     tipMagnitude = Math.abs(pitch);
        //     // tipDirection = calculateRecoveryAngle(pitch);
        // }
    }

    public Wheel[] getWheels() {
        return wheels;
    }
}
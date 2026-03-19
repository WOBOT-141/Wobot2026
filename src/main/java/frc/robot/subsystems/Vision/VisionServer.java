package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.Detections.DetectionList;
import frc.robot.subsystems.Vision.Detections.IDetection;
import frc.robot.subsystems.Vision.Detections.ObjectType;

public class VisionServer extends SubsystemBase {

    private AprilTagFieldLayout m_aprilTagFieldLayout;

    private final boolean DEBUG_LOG = false;

    NetworkTableInstance m_nt_Instance;
    NetworkTable m_nt_table;
    NetworkTableEntry aprilTagEntry;
    StringTopic aprilTagStringTopic;
    StringSubscriber subscriber;

    ByteBuffer m_buffer_from_table;

    public VisionServer() {
        m_buffer_from_table = ByteBuffer.allocate(8192);
        m_nt_Instance = NetworkTableInstance.getDefault();
        m_nt_table = m_nt_Instance.getTable("Vision");
        aprilTagEntry = m_nt_table.getEntry("Apriltag");
        // aprilTagStringTopic = m_nt_Instance.getStringTopic("/Vision/Apriltag");
        // subscriber = aprilTagStringTopic.subscribe("skippyskippyskippy",
        // PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10));

        try {
            m_aprilTagFieldLayout = new AprilTagFieldLayout(
                    Filesystem.getDeployDirectory().toPath().resolve("2026-official.json")
            );
        } catch (IOException e) {

        }

        SmartDashboard.setDefaultBoolean("SimVision/HasTarget", true);
        SmartDashboard.setDefaultNumber("SimVision/TagID", 3);
        SmartDashboard.setDefaultNumber("SimVision/Distance_in", 72.0);
        SmartDashboard.setDefaultNumber("SimVision/Angle_deg", 30.0);
        SmartDashboard.setDefaultNumber("SimVision/RobotBearing_deg", 10.0);

    }

    public DetectionList getDetectionList() {
        DetectionList _detectlist = null; // TODO? maybe shouldn't be null?
        if (aprilTagEntry.exists()) {
            m_buffer_from_table.clear();
            m_buffer_from_table.put(aprilTagEntry.getRaw(null));
            m_buffer_from_table.flip();

            int number_of_tags = m_buffer_from_table.getInt();
            int number_of_fields = m_buffer_from_table.getInt();
            if (DEBUG_LOG) {
                System.out.println("num tags " + number_of_tags + " num fields " + number_of_fields);
            }

            _detectlist = new DetectionList(ObjectType.APRILTAG, number_of_tags);

            for (int i = 0; i < number_of_tags; i++) {
                double distance = m_buffer_from_table.getDouble();
                double angle = m_buffer_from_table.getDouble();
                int id = (int) m_buffer_from_table.getDouble();
                double yaw = m_buffer_from_table.getDouble();
                double pitch = m_buffer_from_table.getDouble();
                double roll = m_buffer_from_table.getDouble();
                double cameraNum = m_buffer_from_table.getDouble();

                _detectlist.addDetection(
                        ObjectType.APRILTAG,
                        distance,
                        angle,
                        id,
                        yaw,
                        pitch,
                        roll,
                        cameraNum);
            }

            for (int i = 0; i < _detectlist.getSize(); i++) {
                if (DEBUG_LOG) {
                    System.out.println(_detectlist.getDetection(i).toString());
                }
            }

        } else {
            if (DEBUG_LOG) {
                System.out.println("No dice!");
            }
        }
        return _detectlist;
    }

    public IDetection getClosestDetection() {
        DetectionList detectionList = getDetectionList();
        if(detectionList == null){
            return null;
        }
        IDetection closestDetection = detectionList.getDetection(0);
        for (int i = 0; i < detectionList.getSize(); i++) {
            IDetection currentDetection = detectionList.getDetection(i);
            if (currentDetection.getDistance() < closestDetection.getDistance()) {
                closestDetection = currentDetection;
            }
        }
        return closestDetection;
    }

    public Pose2d getTagPose() {
        IDetection detection = getClosestDetection();
        if (detection == null) {
            return null;
        }

        return getTagPose(detection.getId());
    }

    public Pose2d getTagPose(int tagID) {
        Optional<Pose3d> pose
                = m_aprilTagFieldLayout.getTagPose(tagID);

        if (pose.isEmpty()) {
            return null;
        }

        Pose3d p3 = pose.get();
        Pose2d p2 = p3.toPose2d();
        Logger.recordOutput("VisionPose", p2);
        return p2;
    }

    public Pose2d getEstimatedRobotPose() {
        IDetection detection = getClosestDetection();
        if (detection == null) {
            return null;
        }

        Optional<Pose3d> tagPoseOptional = m_aprilTagFieldLayout.getTagPose(getClosestDetection().getId()); //Get the current tag from the apriltagFieldLayout
        if (tagPoseOptional.isEmpty()) {
            return null; // null detection should probably change 
        }

        Pose2d tagPose = tagPoseOptional.get().toPose2d(); // getTagPose returns a pose3d, this turns it into a pose3d. for using odom in shooting we need that 3d axis

        double distanceMeters = Units.inchesToMeters(detection.getDistance());//change our inches to meters as the pose excpets meters 
        double bearingRadians = Units.degreesToRadians(-detection.getAngle());//same here but for degrees

        double tagYawRadians = Units.degreesToRadians(detection.getDistance());//same again 

        Rotation2d relativeRotation = new Rotation2d(Math.PI + tagYawRadians);// our system of getting the angle from the gyro is different from the WPI coordinate system. this flips the angle to match

        Transform2d cameraToTag = new Transform2d(
                distanceMeters * Math.cos(bearingRadians), // X offset (Forward)
                distanceMeters * Math.sin(bearingRadians), // Y offset (Sideways)
                relativeRotation // The combined rotation
        );

        return tagPose.transformBy(cameraToTag.inverse());
    }

    public Pose2d getSimulatedRobotPose() {
        int tagToFind = (int) SmartDashboard.getNumber("SimVision/TagID", 3); //This is what powers the Testing UI 
        double distFromTag = SmartDashboard.getNumber("SimVision/Distance_in", 72.0);
        double angleFromTag = SmartDashboard.getNumber("SimVision/Angle_deg", 30.0);
        double robotBearing = SmartDashboard.getNumber("SimVision/RobotBearing_deg", 10.0);

        Optional<Pose3d> tagPoseOptional = m_aprilTagFieldLayout.getTagPose(tagToFind); //Get the current tag from the apriltagFieldLayout
        if (tagPoseOptional.isEmpty()) {
            return null; // null detection should probably change 
        }

        Pose2d tagPose = tagPoseOptional.get().toPose2d(); // getTagPose returns a pose3d, this turns it into a pose3d. for using odom in shooting we need that 3d axis

        double distanceMeters = Units.inchesToMeters(distFromTag);//change our inches to meters as the pose excpets meters 
        double bearingRadians = Units.degreesToRadians(-angleFromTag);//same here but for degrees

        double tagYawRadians = Units.degreesToRadians(robotBearing);//same again 

        Rotation2d relativeRotation = new Rotation2d(Math.PI + tagYawRadians);// our system of getting the angle from the gyro is different from the WPI coordinate system. this flips the angle to match

        Transform2d cameraToTag = new Transform2d(
                distanceMeters * Math.cos(bearingRadians), // X offset (Forward)
                distanceMeters * Math.sin(bearingRadians), // Y offset (Sideways)
                relativeRotation // The combined rotation
        );

        return tagPose.transformBy(cameraToTag.inverse());
    }

    public boolean hasTargets() {
        if (getClosestDetection() == null) {
            return true;
        } else {
            return false;
        }
    }

    public double angleYaw() {
        return m_nt_table.getEntry("yaw").getDouble(0);
    }

    @Override
    public void periodic() {
        getDetectionList();
    }

 
}

package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.nio.BufferUnderflowException;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.Detections.DetectionList;
import frc.robot.subsystems.Vision.Detections.IDetection;
import frc.robot.subsystems.Vision.Detections.NullDetection;
import frc.robot.subsystems.Vision.Detections.ObjectType;

public class VisionServer extends SubsystemBase {

  private final boolean DEBUG_LOG = false;

  NetworkTableInstance m_nt_Instance;
  NetworkTable m_nt_table;
  NetworkTableEntry aprilTagEntry;
  NetworkTableEntry[] aprilTagEntries;
  StringTopic aprilTagStringTopic;
  private AprilTagFieldLayout m_aprilTagFieldLayout;

  ByteBuffer m_buffer_from_table;
  int m_number_of_cameras;
  int m_number_of_Apriltags;

  DetectionList[] m_detections_Apriltag;

  public VisionServer(int ANumberOfCameras, int AnumberOfTags) {

    if (DEBUG_LOG) DriverStation.silenceJoystickConnectionWarning(true);

    m_number_of_cameras = ANumberOfCameras;
    m_number_of_Apriltags = AnumberOfTags;

    m_buffer_from_table = ByteBuffer.allocate(8192);
    m_nt_Instance = NetworkTableInstance.getDefault();
    m_nt_table = m_nt_Instance.getTable("Vision");

    aprilTagEntries = new NetworkTableEntry[ANumberOfCameras];
    for (int i = 0; i < aprilTagEntries.length; ++i) {
      aprilTagEntries[i] = m_nt_table.getEntry("Camid"+i+"/Apriltag");
    }

    m_detections_Apriltag = new DetectionList[ANumberOfCameras];
    for (int i = 0; i < m_detections_Apriltag.length; ++i) {
      m_detections_Apriltag[i] = new DetectionList(ObjectType.APRILTAG, AnumberOfTags);
    }

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

  public void updateDetectionLists() {
    for (int cameraIndex = 0; cameraIndex < m_number_of_cameras; ++cameraIndex) {

      if (DEBUG_LOG) System.out.println("Cam" + cameraIndex + "-----");

      // Clear out old data since we might not be seeing the same things anymore
      m_detections_Apriltag[cameraIndex].clear();

      if (aprilTagEntries[cameraIndex].exists()) {
        // Empty the buffer so it is fresh and then pull everything from the NT table in to it
        m_buffer_from_table.clear();
        m_buffer_from_table.put(aprilTagEntries[cameraIndex].getRaw(null));
        m_buffer_from_table.flip(); // TODO zdw - should this be used?

        int number_of_tags = 0;
        int number_of_fields = 0;

        try {
          number_of_tags = m_buffer_from_table.getInt();
          number_of_fields = m_buffer_from_table.getInt();
          if (DEBUG_LOG) {
            System.out.println("num tags " + number_of_tags + " num fields " + number_of_fields + " " + m_buffer_from_table.remaining());
          }

            while (number_of_tags > 0) {
            --number_of_tags;

            double distance = m_buffer_from_table.getDouble();
            double angle = m_buffer_from_table.getDouble();
            int tag_id = (int) m_buffer_from_table.getDouble();
            double yaw = m_buffer_from_table.getDouble();
            double pitch = m_buffer_from_table.getDouble();
            double roll = m_buffer_from_table.getDouble();
            double cameraNum = m_buffer_from_table.getDouble();

            if (tag_id != 0) {
              m_detections_Apriltag[cameraIndex].addDetection(
              ObjectType.APRILTAG,
              distance,
              angle,
              tag_id,
              yaw,
              pitch,
              roll,
              cameraNum
            );
            }
            else System.out.println("Tag id 0");

            if (DEBUG_LOG) System.out.println("Detection: " + m_detections_Apriltag[cameraIndex].getDetectionById(tag_id).toString());

          };
        } catch (BufferUnderflowException exception) {
          // nothing seen so incase we get the underflow log out then continue to the next camera
          if (DEBUG_LOG) System.out.println("VisionServer BufferUnderflowException camindex " + cameraIndex);
          continue;
        }

      } else {
        if (DEBUG_LOG) System.out.println("No dice! " + aprilTagEntries[cameraIndex].getName());
      }
    }
  }

  public boolean hasTargets() {
    if (getClosestDetection() == null) {
        return true;
    } else {
        return false;
    }
  }

  public Pose2d getTagPose() {
    IDetection detection = getClosestDetection();
    if (detection == null) {
        return null;
    }

    return getTagPose(detection.getId());
  }

  public Pose2d getTagPose(int tagID) {
    Optional<Pose3d> pose = m_aprilTagFieldLayout.getTagPose(tagID);

    if (pose.isEmpty()) {
      return null;
    }

    Pose3d p3 = pose.get();
    Pose2d p2 = p3.toPose2d();
    Logger.recordOutput("VisionPose", p2);
    return p2;
  }

  public DetectionList[] getApriltagDetectionLists() {
    return m_detections_Apriltag;
  }

  public IDetection getClosestDetection() {
    IDetection closestDetection = NullDetection.Instance;
    for (DetectionList detection_list : m_detections_Apriltag) {
      for (int i = 0; i < detection_list.getSize(); i++){
        IDetection currentDetection = detection_list.getDetection(i);
        if (currentDetection.getDistance() < closestDetection.getDistance())
        {
          closestDetection = currentDetection;
        }
      }
    }

    return closestDetection;
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
    double bearingRadians = Units.degreesToRadians(-detection.getBearing());//same here but for degrees // zane, this was getAngle()

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

  @Override
  public void periodic() {
    updateDetectionLists();
  }
}

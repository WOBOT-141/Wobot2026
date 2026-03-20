package frc.robot.subsystems.Vision;

import java.nio.BufferUnderflowException;
import java.nio.ByteBuffer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
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
  StringSubscriber subscriber;

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
            else if (DEBUG_LOG) System.out.println("Tag id 0");

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

  @Override
  public void periodic() {
    updateDetectionLists();
  }
}



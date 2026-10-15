package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;



public class PhotonVisionServer extends SubsystemBase implements IVision {

    PhotonCamera camera;
    int tagID;

    String taginfomationTosmartDashboard; // 
    

    public  PhotonVisionServer() {
        // Initialize the PhotonVision server here
        camera = new PhotonCamera("PC_Camera"); 


    }

    public PhotonTrackedTarget getTagID(int tagID, PhotonPipelineResult result ) { // IDetection type in old // Look for photonUtils
        // looks through targtes litds
        for (int i = 0; i < result.getTargets().size(); i++) {
            if(result.getTargets().get(i).getFiducialId() == tagID) {
                return result.getTargets().get(i);
            }
        }
        return null;
        // double distance; //testing
        // camea = d.calculateDistanceToTargetMeters(); // testing

        // Calculate robot's field relative pose
       
    }

    public void getBestTarget(PhotonPipelineResult result) {
        if (result.hasTargets()) {
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            tagID = bestTarget.getFiducialId();
            taginfomationTosmartDashboard = bestTarget.toString();
            // You can also get other information about the target here
        }
    }
    public void calculatePosition(PhotonTrackedTarget target) {
        Transform3d cameraToRobot;// place holder //TODO remove place holder
        // AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.);
        //  if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        //     Pose3d rofbotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), CAMERATOROBOT );
        // }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        PhotonPipelineResult results = camera.getLatestResult();
        boolean hasTargets = results.hasTargets(); // returns none if no targets 
        SmartDashboard.putBoolean("Has Targets vision photon", hasTargets);
        if (hasTargets) {
            PhotonTrackedTarget bestTarget = results.getBestTarget();
            tagID = bestTarget.getFiducialId();
            taginfomationTosmartDashboard = bestTarget.toString();
        // // if (!hasTargets) {
        // //     return;
            SmartDashboard.putNumber("atagID Photon", tagID);
            System.out.println(taginfomationTosmartDashboard);
        }    
            


        
        // }
        // results.getTargets().forEach(target -> {
        //     tagID = target.getFiducialId();
        //     target.getBestCameraToTarget().getTranslation().getMeasureX();
        //     target.getBestCameraToTarget().getRotation(); // returns the translation vector from the camera to the target
        //     // You can also get other information about the target here
        // });

    }

}

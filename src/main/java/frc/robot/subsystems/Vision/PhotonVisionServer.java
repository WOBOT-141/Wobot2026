package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;




public class PhotonVisionServer extends SubsystemBase implements IVision {

    PhotonCamera camera;
    int tagID;
    

    public PhotonVisionServer() {
        // Initialize the PhotonVision server here
        camera = new PhotonCamera("photonvision"); 
        

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        PhotonPipelineResult results = camera.getLatestResult();
        boolean hasTargets = results.hasTargets(); // returns none if no targets 
        if (!hasTargets) {
            return;
        }
        // results.getTargets().forEach(target -> {
        //     tagID = target.getFiducialId();
        //     target.getBestCameraToTarget().getTranslation().getMeasureX();
        //     target.getBestCameraToTarget().getRotation(); // returns the translation vector from the camera to the target
        //     // You can also get other information about the target here
        // });

    }

}

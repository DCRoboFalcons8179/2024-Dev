package frc.robot.subsystems;

import java.util.Objects;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {

    private PhotonCamera camera = new PhotonCamera("photonvision");
    private boolean seesObject = false;
    private PhotonTrackedTarget object;

    @Override
    public void periodic() {
        var toSet = camera.getLatestResult().getBestTarget();
        if (Objects.isNull(toSet)) {
            seesObject = true;
            object = toSet;
        } else {
            seesObject = false;
        }
    }

    public boolean seesObject() {
        return seesObject;
    }

    /**
     * should probably check if you see an object
     */
    public double getRingOffsetX() {
        return object.getYaw();
    }

    /**
     * should probably check if you see an object
     */
    public double getRingOffsetY() {
        return object.getPitch();
    }

}

package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

/**
 * Simple static field to store the pose between OpModes.
 * Auto writes to this, Teleop reads from this.
 */
public class PoseStorage {
    // Default to 0,0,0 if no auto was run
    public static Pose currentPose = new Pose(56.000, 8.000, Math.toRadians(90));
}
package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.opModes.BaseShooterOpMode;

public class Drive extends SubsystemBase {

    private final Follower mFollower;
    public boolean aligned = false;

    public Drive(Follower follower) {
        mFollower = follower;
    }

    public void joystickDrive(double forward, double strafe, double turn) {
        mFollower.setTeleOpDrive(forward, strafe, turn, false);
    }

    public void resetHeading() {
        mFollower.startTeleOpDrive();
        mFollower.setPose(new Pose(57.000, 9.000, Math.toRadians(90.0)));
        mFollower.setTeleOpDrive(0.0, 0.0, 0.0, false);
    }

    public void startTeleopDrive() {
        mFollower.startTeleOpDrive();
    }

    public Follower getFollower() {
        return mFollower;
    }

    public boolean isAligned() {
        return aligned;
    }
}

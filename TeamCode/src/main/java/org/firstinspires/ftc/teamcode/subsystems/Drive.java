package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Drive extends SubsystemBase {

    private final Follower mFollower;
    public boolean aligned = false;

    public Drive(Follower follower) {
        mFollower = follower;
    }
    @Override
    public void periodic() {
    }

    public void joystickDrive(double forward, double strafe, double turn) {
        mFollower.setTeleOpDrive(forward, strafe, turn, false);
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

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
        // 1. Get the current heading in radians
        double heading = mFollower.getPose().getHeading();

        // 2. Calculate Field-Oriented Vectors manually
        // Since 90 degrees (North) is "Forward", we treat:
        // 'forward' input as Field Y
        // 'strafe' input as Field X

        // We rotate these vectors by the negative of the robot's heading to convert
        // from Field Frame to Robot Frame.

        // Math:
        // RobotX = FieldX * cos(-heading) - FieldY * sin(-heading)
        // RobotY = FieldX * sin(-heading) + FieldY * cos(-heading)

        // Subbing in strafe for FieldX and forward for FieldY:
        double rotForward = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
        double rotStrafe  = strafe * Math.cos(-heading) - forward * Math.sin(-heading);

        // 3. Send to follower as ROBOT CENTRIC (true)
        // We pass true because we have already handled the field-centric rotation math above.
        // Note: Check if your drive setup expects Y to be Left or Right.
        // Standard Pedro/Roadrunner is X=Forward, Y=Left.
        mFollower.setTeleOpDrive(rotForward, rotStrafe, turn, true);
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



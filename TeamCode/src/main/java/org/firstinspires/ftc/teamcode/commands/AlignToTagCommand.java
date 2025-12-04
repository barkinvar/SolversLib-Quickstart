package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.opModes.BaseShooterOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import java.util.function.DoubleSupplier;

public class AlignToTagCommand extends CommandBase {

    private final Vision vision;
    private final Drive drive;
    private final DoubleSupplier xAxis, yAxis;
    private final double targetHeadingRadians;

    // --- Tuning Constants ---
    private static final double Kp = 0.025;
    private static final double Kd = 0.00125;

    private static final double MAX_VISION_SPEED = 1.0;

    private static final double SCAN_SPEED = 1.0;

    // State Variables
    private double previousTx = 0;

    // 0 = Not scanning (or undecided), 1 = Left, -1 = Right
    private double lockedScanDirection = 0;
    private BaseShooterOpMode.Alliance alliance;

    public AlignToTagCommand(Vision vision, Drive drive, DoubleSupplier xAxis, DoubleSupplier yAxis, double targetHeadingDegrees, BaseShooterOpMode.Alliance alliance) {
        this.vision = vision;
        this.drive = drive;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.targetHeadingRadians = Math.toRadians(targetHeadingDegrees);
        this.alliance = alliance;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        previousTx = 0;
        lockedScanDirection = 0; // Reset on start
        drive.aligned = false;

        drive.startTeleopDrive();
    }

    @Override
    public void execute() {
        double turnPower;

        if (vision.hasValidTarget()) {
            // ===========================================
            // TARGET FOUND: UNLOCK AND TRACK
            // ===========================================

            // We see the tag, so we don't need our "blind guess" anymore.
            // Reset this to 0 so next time we lose the tag, we recalculate fresh.
            lockedScanDirection = 0;

            double tx = vision.getSteeringError();

            if(vision.getDistance() > 2.65) {
                tx += alliance == BaseShooterOpMode.Alliance.BLUE ? -3.0 : 3.0;
            }

            double derivative = tx - previousTx;
            previousTx = tx;

            double rawTurn = -(Kp * tx + Kd * derivative);
            turnPower = Math.max(-MAX_VISION_SPEED, Math.min(MAX_VISION_SPEED, rawTurn));

            drive.aligned = Math.abs(tx) < 2.65;

        } else {
            // ===========================================
            // TARGET LOST: COMMIT TO A DIRECTION
            // ===========================================

            // Only calculate the direction IF we haven't picked one yet.
            if (lockedScanDirection == 0) {
                double currentHeading = drive.getFollower().getPose().getHeading();
                double angleDifference = getShortestAngleDiff(targetHeadingRadians, currentHeading);

                // Determine direction (-1 or 1)
                // If angleDifference is 0 (extremely rare), default to 1
                lockedScanDirection = (angleDifference >= 0) ? 1.0 : -1.0;
            }

            // Just apply the locked speed.
            // We ignore the current heading now. We trust the turn.
            turnPower = lockedScanDirection * SCAN_SPEED;

            previousTx = 0;
            drive.aligned = false;
        }

        drive.joystickDrive(xAxis.getAsDouble(), yAxis.getAsDouble(), turnPower);
    }

    @Override
    public void end(boolean interrupted) {
        drive.joystickDrive(0, 0, 0);
        drive.aligned = false;
    }

    private double getShortestAngleDiff(double target, double current) {
        double diff = target - current;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}
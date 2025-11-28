package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * AlignToTagCommand
 * * Automatically aligns the robot to face a specific AprilTag.
 * Uses a simple P-Controller to minimize the "tx" (horizontal error).
 */
public class AlignToTagCommand extends CommandBase {

    private final Vision vision;
    private final Follower follower;
    private final int targetTagId;

    // PID Constants
    // Kp: Proportional gain. Adjust this if the robot oscillates (too high) or aligns too slowly (too low).
    private static final double Kp = 0.02;

    // Max turning power to prevent the robot from spinning too wildly
    private static final double MAX_TURN_SPEED = 1.0;

    // Deadzone: If error is within this many degrees, consider it aligned (stops jitter)
    private static final double ALIGNMENT_TOLERANCE_DEGREES = 1.0;

    private DoubleSupplier xAxis, yAxis;

    public AlignToTagCommand(Vision vision, Follower follower, int targetTagId, DoubleSupplier xAxis, DoubleSupplier yAxis) {
        this.vision = vision;
        this.follower = follower;
        this.targetTagId = targetTagId;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
    }

    @Override
    public void initialize() {
        // Tell Vision subsystem to specifically look for our desired tag
        vision.setTargetTagId(targetTagId);
    }

    @Override
    public void execute() {
        double turnPower = 0.0;

        if (vision.hasValidTarget()) {
            double tx = vision.getSteeringError();

            if (Math.abs(tx) > ALIGNMENT_TOLERANCE_DEGREES) {
                // Calculate P-Loop output
                // tx is positive when target is to the Right.
                // We want to turn Right.
                // Based on your input "-gamepad1.right_stick_x", a negative value turns Right.
                // So we negate the result: - (Kp * tx)
                double rawTurn = -(Kp * tx);

                // Clamp the output to max speed
                turnPower = Math.max(-MAX_TURN_SPEED, Math.min(MAX_TURN_SPEED, rawTurn));
            }
        } else {
            // Optional: If target is lost, you might want to spin slowly to find it?
            // For now, we stay still for safety.
            turnPower = 0.0;
        }

        // Apply to drivetrain
        // x=0, y=0, turn=turnPower, fieldCentric=false
        follower.setTeleOpDrive(xAxis.getAsDouble(), yAxis.getAsDouble(), turnPower, false);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        follower.setTeleOpDrive(0, 0, 0, false);

        // Reset Vision to wildcard mode (optional, remove if you want to keep the filter)
    }

    @Override
    public boolean isFinished() {
        // Return false to keep aligning while the button is held.
        // If you want this to auto-finish when aligned, return (Math.abs(vision.getSteeringError()) < TOLERANCE);
        return false;
    }
}
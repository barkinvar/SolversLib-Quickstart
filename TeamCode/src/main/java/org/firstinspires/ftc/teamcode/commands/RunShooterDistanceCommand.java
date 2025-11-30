package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * Runs the shooter at a velocity calculated based on Vision distance.
 * * Logic:
 * 1. No Target -> Fallback RPM
 * 2. Distance > 3.25 -> Long Range RPM
 * 3. Standard Range -> Interpolate between Min/Max distance and RPM
 */
public class RunShooterDistanceCommand extends CommandBase {

    private final Shooter shooter;
    private final Vision vision;

    // Tuning Constants
    private static final double MIN_DIST = 1.29;
    private static final double MAX_DIST = 2.55;

    private static final double MIN_RPM = 2650.0;
    private static final double MAX_RPM = 2900.0;

    private static final double LONG_RANGE_CUTOFF = 3.25;
    private static final double LONG_RANGE_RPM = 3300.0;
    private static final double FALLBACK_RPM = 2500.0;

    public RunShooterDistanceCommand(Shooter shooter, Vision vision) {
        this.shooter = shooter;
        this.vision = vision;

        // We require the Shooter, but usually NOT vision (read-only),
        // allowing other commands to read vision simultaneously if needed.
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Optional: Reset internal PID state if necessary
    }

    @Override
    public void execute() {
        double targetRPM;

        // 1. Safety Check: No vision target
        if (!vision.hasValidTarget()) {
            targetRPM = FALLBACK_RPM;
        }
        // 2. Long Range Check
        else if (vision.getDistance() > LONG_RANGE_CUTOFF) {
            targetRPM = LONG_RANGE_RPM;
        }
        // 3. Interpolation Logic
        else {
            double currentDist = vision.getDistance();

            // Clamp distance to range [1.29, 2.55]
            // If dist is 3.0 (the gap between 2.55 and 3.25), it stays clamped at 2.55 (MAX_RPM)
            // If dist is 0.5 (too close), it stays clamped at 1.29 (MIN_RPM)
            double clampedDist = Math.max(MIN_DIST, Math.min(currentDist, MAX_DIST));

            // Calculate percentage (0.0 to 1.0)
            double percent = (clampedDist - MIN_DIST) / (MAX_DIST - MIN_DIST);

            // Linear Interpolate
            targetRPM = MIN_RPM + (percent * (MAX_RPM - MIN_RPM));
        }

        shooter.setTargetVelocity(targetRPM);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the shooter when the command ends (e.g. button released)
        shooter.setTargetVelocity(0.0);
    }

    @Override
    public boolean isFinished() {
        // Run continuously until interrupted
        return false;
    }
}
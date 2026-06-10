package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * Runs the shooter at a velocity calculated based on Vision distance.
 * * Logic:
 * 1. No Target -> Fallback RPM
 * 2. Long Range (dist > cutoff) -> Interpolate [2.75 m, 3.5 m] onto [3100, 3300] RPM
 * 3. Standard Range -> Interpolate between Min/Max distance and RPM
 */
public class RunShooterDistanceCommand extends CommandBase {

    private final Shooter shooter;
    private final Vision vision;

    // Tuning Constants
    private static final double MIN_DIST = 1.29;
    private static final double MAX_DIST = 2.45;

    private static final double MIN_RPM = 2650.0;
    private static final double MAX_RPM = 2800.0;


    private static final double LONG_RANGE_CUTOFF = 2.65;

    // Long-range shots interpolate distance [2.75 m, 3.5 m] onto RPM [3100, 3300].
    private static final double LONG_MIN_DIST = 2.7;
    private static final double LONG_MAX_DIST = 3.5;
    private static final double LONG_MIN_RPM = 3075.0;
    private static final double LONG_MAX_RPM = 3350.0;

    private static final double FALLBACK_RPM = 2600.0;

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
        // 2. Long Range Check: interpolate RPM with distance instead of a flat value.
        else if (vision.getDistance() > LONG_RANGE_CUTOFF) {
            // Clamp distance to [2.75, 3.5]; closer than 2.75 -> 3100, farther than 3.5 -> 3300.
            double clampedDist = Math.max(LONG_MIN_DIST, Math.min(vision.getDistance(), LONG_MAX_DIST));
            double percent = (clampedDist - LONG_MIN_DIST) / (LONG_MAX_DIST - LONG_MIN_DIST);
            targetRPM = LONG_MIN_RPM + (percent * (LONG_MAX_RPM - LONG_MIN_RPM));
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
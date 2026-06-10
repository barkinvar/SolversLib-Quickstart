package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Drive extends SubsystemBase {

    private final Follower mFollower;
    public boolean aligned = false;

    // --- Localization / Pinpoint health guarding ------------------------------------------------
    // The Pinpoint can drop off the I2C bus on a hard hit or ESD event. When that happens the
    // pose read can come back NaN, or (after the device reboots) it resets its internal pose+IMU
    // to 0. Either case corrupts field-oriented drive and flings autos. We detect that and:
    //   1. freeze the heading used for field-centric math at the last good value, and
    //   2. re-seat the follower pose to the last good pose once the device recovers.
    private GoBildaPinpointDriver mPinpoint; // may be null if the localizer isn't a Pinpoint
    private Pose mLastGoodPose = new Pose(0, 0, 0);
    private double mLastGoodHeading = 0.0;
    private boolean mHasGoodPose = false;
    private boolean mLocalizationHealthy = true;
    private boolean mPendingResync = false; // device rebooted/lost pods -> re-seat pose on recovery
    private String mLocalizationStatus = "INIT";

    // Diagnostics so the driver can tell on the hub whether the Pinpoint misbehaved this match.
    private int mFaultCount = 0;   // number of distinct glitch events (healthy -> unhealthy edges)
    private int mResyncCount = 0;  // number of times we re-seated the pose after a reboot/fault
    private boolean mPrevHealthy = true;

    public Drive(Follower follower) {
        mFollower = follower;
        mPinpoint = resolvePinpoint(follower);
    }

    /** Walk the follower's pose tracker to grab the raw Pinpoint driver, if there is one. */
    private static GoBildaPinpointDriver resolvePinpoint(Follower follower) {
        try {
            Localizer localizer = follower.getPoseTracker().getLocalizer();
            if (localizer instanceof PinpointLocalizer) {
                return ((PinpointLocalizer) localizer).getPinpoint();
            }
        } catch (Exception ignored) {
            // Fall through: we simply run without device-status awareness.
        }
        return null;
    }

    public void joystickDrive(double forward, double strafe, double turn) {
        // Use the guarded heading (last good value while the Pinpoint is glitching) so the field
        // frame never snaps to a NaN/0 reading mid-match.
        double heading = getSafeHeading();

        // Rotate the field-centric input into the robot frame by -heading.
        // NOTE: If the robot spins and "forward" drifts, flip the negative sign on heading below.
        double rotX = forward * Math.cos(-heading) - strafe * Math.sin(-heading);
        double rotY = forward * Math.sin(-heading) + strafe * Math.cos(-heading);

        // Final safety net: never push NaN/Inf into the motors.
        if (!isFinite(rotX) || !isFinite(rotY) || !isFinite(turn)) {
            rotX = 0.0;
            rotY = 0.0;
            turn = 0.0;
        }

        // PedroPathing/RoadRunner Standard: X is Forward, Y is Left/Strafe.
        mFollower.setTeleOpDrive(rotX, rotY, turn, true);
    }

    /**
     * Poll the Pinpoint health and keep our last-good pose/heading fresh. Call this once per loop,
     * BEFORE {@code follower.update()}, so that any recovery re-sync takes effect the same loop.
     */
    public void updateLocalization() {
        boolean healthy;
        boolean hardFault; // a reboot / pod loss -> absolute pose is lost, must re-seat on recovery

        GoBildaPinpointDriver.DeviceStatus status = null;
        try {
            if (mPinpoint != null) {
                status = mPinpoint.getDeviceStatus();
            }
        } catch (Exception e) {
            status = null; // treat an I2C exception as "device gone"
        }

        boolean nan = mFollower.isLocalizationNAN();

        if (mPinpoint == null) {
            // No device-status info available: fall back to a pure NaN check.
            healthy = !nan;
            hardFault = false;
            mLocalizationStatus = healthy ? "OK" : "NAN";
        } else if (status == null) {
            healthy = false;
            hardFault = true; // exception reading the bus -> assume it dropped
            mLocalizationStatus = "DISCONNECTED";
        } else {
            mLocalizationStatus = status.toString();
            switch (status) {
                case READY:
                    healthy = !nan;
                    hardFault = false;
                    break;
                case FAULT_BAD_READ:
                    // Single corrupted (CRC-caught) frame. Pose is intact; just freeze for a loop.
                    healthy = false;
                    hardFault = false;
                    break;
                case NOT_READY:
                case CALIBRATING:
                case FAULT_NO_PODS_DETECTED:
                case FAULT_X_POD_NOT_DETECTED:
                case FAULT_Y_POD_NOT_DETECTED:
                case FAULT_IMU_RUNAWAY:
                default:
                    // Device rebooted / lost a pod / IMU diverged -> its absolute pose is no longer
                    // trustworthy, so we must re-seat it from our last good pose once it returns.
                    healthy = false;
                    hardFault = true;
                    break;
            }
            if (nan) {
                healthy = false;
            }
        }

        // Count a new disturbance only on the healthy -> unhealthy edge (not every glitch loop).
        if (mPrevHealthy && !healthy) {
            mFaultCount++;
        }
        mPrevHealthy = healthy;

        if (hardFault) {
            mPendingResync = true;
        }

        if (healthy) {
            Pose pose = mFollower.getPose();
            if (pose != null && isFinite(pose.getX()) && isFinite(pose.getY()) && isFinite(pose.getHeading())) {
                if (mPendingResync && mHasGoodPose) {
                    // Device just came back after a reboot/fault: restore continuity so field-
                    // oriented drive and any running path don't see a teleport to (0,0,0).
                    mFollower.setPose(mLastGoodPose);
                    mPendingResync = false;
                    mResyncCount++;
                } else {
                    mLastGoodPose = pose;
                    mLastGoodHeading = pose.getHeading();
                    mHasGoodPose = true;
                    mPendingResync = false;
                }
            }
        }

        mLocalizationHealthy = healthy;
    }

    /** Heading for field-centric math: live value when healthy, last good value while glitching. */
    public double getSafeHeading() {
        if (mLocalizationHealthy) {
            double h = mFollower.getPose().getHeading();
            if (isFinite(h)) {
                return h;
            }
        }
        return mLastGoodHeading;
    }

    public boolean isLocalizationHealthy() {
        return mLocalizationHealthy;
    }

    public String getLocalizationStatus() {
        return mLocalizationStatus;
    }

    /** Number of distinct localization glitches detected this OpMode. */
    public int getLocalizationFaultCount() {
        return mFaultCount;
    }

    /** Number of times the pose was re-seated after a Pinpoint reboot/fault this OpMode. */
    public int getLocalizationResyncCount() {
        return mResyncCount;
    }

    /** Pinpoint sample frequency in Hz (drops when the bus is struggling); -1 if unavailable. */
    public double getPinpointFrequency() {
        if (mPinpoint == null) {
            return -1;
        }
        try {
            return mPinpoint.getFrequency();
        } catch (Exception e) {
            return -1;
        }
    }

    public void resetHeading() {
        mFollower.startTeleOpDrive();
        Pose reset = new Pose(57.000, 9.000, Math.toRadians(90.0));
        mFollower.setPose(reset);
        // Keep the guard in sync so it doesn't immediately overwrite this manual reset.
        mLastGoodPose = reset;
        mLastGoodHeading = reset.getHeading();
        mHasGoodPose = true;
        mPendingResync = false;
        mFollower.setTeleOpDrive(0.0, 0.0, 0.0, true);
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

    private static boolean isFinite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }
}

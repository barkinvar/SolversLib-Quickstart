package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Vision extends SubsystemBase {

    private final Limelight3A limelight;
    private final TelemetryData telemetry;

    // --- Configuration Constants ---
    // NOTE: User provided units in Meters (29.25 * 0.0254)
    private static final double CAMERA_LENS_HEIGHT = 0.433; // Meters
    private static final double TARGET_HEIGHT = 29.25 * 0.0254; // Meters
    private static final double CAMERA_MOUNT_ANGLE_DEGREES = 20.0;

    // Low Pass Filter factor (0.0 - 1.0)
    private static final double SMOOTHING_FACTOR = 0.2;

    // --- State Variables ---
    private double currentDistance = 0.0;
    private double currentTx = 0.0;
    private double currentTy = 0.0;
    private double currentTa = 0.0;
    private boolean hasTarget = false;

    // Filter: ID of the AprilTag to track. -1 means "Track Any"
    private int targetTagId = -1;

    // Neural Network / Classification Results
    private String detectedLabel = "None";
    private double detectionConfidence = 0.0;

    public Vision(HardwareMap hardwareMap, TelemetryData telemetry) {
        this.telemetry = telemetry;
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        telemetry.addData("Limelight", "Initialized & Started");
        telemetry.update();
    }

    @Override
    public void periodic() {
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            boolean validTargetFound = false;
            double rawTx = 0;
            double rawTy = 0;
            double rawTa = 0;

            // --- 1. AprilTag Filtering Logic ---
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (targetTagId != -1) {
                // strict mode: Only look for the specific ID
                for (LLResultTypes.FiducialResult tag : fiducials) {
                    if (tag.getFiducialId() == targetTagId) {
                        // FIX: Use getTxnc() and getTync() for individual fiducials
                        rawTx = tag.getTargetXDegrees();
                        rawTy = tag.getTargetYDegrees();
                        rawTa = tag.getTargetArea();
                        validTargetFound = true;
                        break; // Stop looking once found
                    }
                }
            } else {
                // Wildcard mode: Take the primary result (closest/largest)
                // This could be an AprilTag OR a Neural Network detection depending on pipeline
                rawTx = result.getTx();
                rawTy = result.getTy();
                rawTa = result.getTa();
                validTargetFound = true;
            }

            // --- 2. Neural Network Logic (Secondary) ---
            // If we didn't find a specific tag, but we are in wildcard mode,
            // check for game pieces (Detector Pipeline)
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
            if (!detectorResults.isEmpty()) {
                LLResultTypes.DetectorResult bestDetection = detectorResults.get(0);
                detectedLabel = bestDetection.getClassName();
                detectionConfidence = bestDetection.getConfidence();

                // If we aren't hunting a specific AprilTag, and we see a game piece, track it
                if (targetTagId == -1 && !fiducials.isEmpty() == false) {
                    // Note: logic implies if we see NO tags, but we SEE a sample, we use sample data
                    // Limelight 'result.getTx()' usually prioritizes the main target anyway.
                }
            } else {
                detectedLabel = "None";
                detectionConfidence = 0.0;
            }

            // --- 3. Update State if Valid ---
            if (validTargetFound) {
                hasTarget = true;

                // Distance Calculation (Tangent)
                // d = (h2 - h1) / tan(a1 + a2)
                double angleToGoalDegrees = CAMERA_MOUNT_ANGLE_DEGREES + rawTy;
                double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

                double calculatedDistance = (TARGET_HEIGHT - CAMERA_LENS_HEIGHT) / Math.tan(angleToGoalRadians);

                // Apply Smoothing
                currentDistance = (SMOOTHING_FACTOR * calculatedDistance) + ((1 - SMOOTHING_FACTOR) * currentDistance);

                currentTx = rawTx;
                currentTy = rawTy;
                currentTa = rawTa;
            } else {
                hasTarget = false;
            }

        } else {
            hasTarget = false;
        }

        // --- Telemetry ---
        telemetry.addData("Limelight", status.getName());
        if (hasTarget) {
            telemetry.addData("Target", "FOUND");
            telemetry.addData("Target ID Filter", targetTagId == -1 ? "ANY" : targetTagId);
            telemetry.addData("tx", currentTx);
            telemetry.addData("ty", currentTy);
            telemetry.addData("Dist", currentDistance);
        } else {
            telemetry.addData("Target", "SEARCHING...");
        }
    }

    // --- Setters & Getters ---

    /**
     * Sets the AprilTag ID to look for.
     *
     * @param id The ID to track (e.g., 1, 2, 11, etc.). Set to -1 to track ANY target.
     */
    public void setTargetTagId(int id) {
        this.targetTagId = id;
    }

    public double getSteeringError() {
        return currentTx;
    }

    public double getDistance() {
        return currentDistance;
    }

    public boolean hasValidTarget() {
        return hasTarget;
    }

    public String getDetectedLabel() {
        return detectedLabel;
    }

    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    public void stop() {
        limelight.stop();
    }
}
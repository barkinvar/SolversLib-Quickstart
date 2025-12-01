package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Led extends SubsystemBase {

    private final Servo ledController;
    private RobotState targetState = RobotState.OFF;

    // For animations
    private final ElapsedTime timer = new ElapsedTime();
    private boolean isAnimating = false;

    /**
     * Enum mapping robot states to goBILDA Servo PWM values.
     * Mappings based on goBILDA 3110-0002-0001 Product Insight #4.
     */
    public enum RobotState {
        OFF(0.0),             // Black

        // Solid Colors (from chart)
        SHOOTER_IDLE(0.28),   // Red
        SHOOTER_WARMING_UP(0.611), // Blue (Chart says Green is 0.5, Blue is 0.611. Adjusted below)
        SHOOTER_READY(0.480),  // Green (Chart says Sage 0.444, Green 0.5)

        // Animation States
        RGB_CYCLE(0.0);

        public final double pwm;

        RobotState(double pwm) {
            this.pwm = pwm;
        }
    }

    public Led(HardwareMap hardwareMap) {
        ledController = hardwareMap.get(Servo.class, "led");
        setState(RobotState.RGB_CYCLE);
    }

    public void setState(RobotState state) {
        // Only reset timer if we are switching INTO an animation from a non-animation
        boolean newIsAnimating = (state == RobotState.RGB_CYCLE);

        if (newIsAnimating && !isAnimating) {
            timer.reset();
        }

        this.targetState = state;
        this.isAnimating = newIsAnimating;
    }

    @Override
    public void periodic() {
        if (isAnimating) {
            handleAnimation();
        } else {
            // Standard static color logic
            writeHardware(targetState.pwm);
        }
    }

    /**
     * Handles dynamic LED updates for animation states.
     */
    private void handleAnimation() {
        double time = timer.seconds();

        if (targetState == RobotState.RGB_CYCLE) {
            // Based on the chart, the spectrum runs from Red (0.277) to Violet (0.722).
            // We linearly ramp between these two values to access the full gradient.

            double minPwm = 0.277; // Red
            double maxPwm = 0.722; // Violet
            double cycleTime = 2.0; // Seconds for one full rainbow loop

            // Calculate progress from 0.0 to 1.0 that repeats every cycleTime
            double progress = (time % cycleTime) / cycleTime;

            // Map progress to the PWM range
            double targetPwm = minPwm + (progress * (maxPwm - minPwm));

            writeHardware(targetPwm);
        }
    }

    /**
     * Internal helper to write to the servo only if the value has changed.
     */
    private void writeHardware(double pwm) {
        ledController.setPosition(pwm);
    }
}
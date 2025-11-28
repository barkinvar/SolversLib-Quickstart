package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler; // Import Scheduler
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.AlignToTagCommand;
import org.firstinspires.ftc.teamcode.commands.RunFeederCommand;
import org.firstinspires.ftc.teamcode.commands.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.RunShooterCommand;
import org.firstinspires.ftc.teamcode.commands.shootFeedCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@TeleOp
public class ShooterOpMode extends CommandOpMode {

    // We can use TelemetryData for our own OpMode logs,
    // but the Subsystem usually expects standard Telemetry.
    TelemetryData telemetryData;
    Follower follower;

    Shooter mShooter;
    Intake mIntake;
    Feeder mFeeder;
    Vision mVision;
    GamepadEx controller;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);

        // 1. RESET FIRST
        // If you reset after binding buttons, you delete the bindings!
        CommandScheduler.getInstance().reset();
        follower.startTeleopDrive();

        telemetryData = new TelemetryData(telemetry);

        // 2. Initialize Subsystem
        // Pass the standard 'telemetry' object to match the Shooter class
        mShooter = new Shooter(hardwareMap, telemetryData);
        mIntake = new Intake(hardwareMap);
        mFeeder = new Feeder(hardwareMap);
        mVision = new Vision(hardwareMap, telemetryData);

        register(mShooter,mIntake,mFeeder,mVision);

        // 4. Initialize Controller
        controller = new GamepadEx(gamepad1);

        // 5. Bind Buttons
        bindButtons();
    }

    @Override
    public void run() {
        // 1. CRITICAL: Read Controller Inputs
        // Without this, the code doesn't know 'A' is being held.
        controller.readButtons();
        if(!gamepad1.x) {

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false); }

        follower.update();

        // 2. Run Scheduler (Handled by super.run() in CommandOpMode)
        super.run();

        // 3. Update Telemetry
        telemetryData.update();
        // Since Shooter uses standard telemetry, we also need standard update
        // (Unless TelemetryData wraps it in a way that handles both)
        telemetry.update();

    }

    private void bindButtons() {
        // When 'A' is held, run the shooter at 3000 RPM (or TPS)
        controller.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new RunShooterCommand(mShooter, 2900.0).alongWith(new shootFeedCommand(mFeeder, mIntake, mShooter)));

        controller.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new RunShooterCommand(mShooter, 3400.0).alongWith(new shootFeedCommand(mFeeder, mIntake, mShooter)));

        controller.getGamepadButton(GamepadKeys.Button.X).whileHeld(new AlignToTagCommand(mVision, follower, 20, () -> -gamepad1.left_stick_y, () -> -gamepad1.left_stick_x));

        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new RunFeederCommand(mFeeder));

        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new RunIntakeCommand(mIntake));

    }
}
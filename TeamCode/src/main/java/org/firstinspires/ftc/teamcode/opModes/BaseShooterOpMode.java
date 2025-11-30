package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.AlignToTagCommand;
import org.firstinspires.ftc.teamcode.commands.RunFeederCommand;
import org.firstinspires.ftc.teamcode.commands.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.RunShooterCommand;
import org.firstinspires.ftc.teamcode.commands.RunShooterDistanceCommand;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.commands.ShootFeedCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * BaseShooterOpMode
 * Refactored to use a Constructor for Alliance configuration.
 */
public abstract class BaseShooterOpMode extends CommandOpMode {

    // Define the Enum directly here or in a separate file
    public enum Alliance {
        RED,
        BLUE
    }

    // This field stores which alliance we are running
    protected final Alliance alliance;

    // Constructor required by Child classes
    public BaseShooterOpMode(Alliance alliance) {
        this.alliance = alliance;
    }

    protected TelemetryData telemetryData;
    protected Follower follower;
    protected Shooter mShooter;
    protected Intake mIntake;
    protected Feeder mFeeder;
    protected Vision mVision;
    protected  Drive mDrive;
    protected GamepadEx controller;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);

        // 1. RESET FIRST
        CommandScheduler.getInstance().reset();

        telemetryData = new TelemetryData(telemetry);

        // 2. Initialize Subsystem
        mShooter = new Shooter(hardwareMap, telemetryData);
        mIntake = new Intake(hardwareMap);
        mFeeder = new Feeder(hardwareMap);
        mVision = new Vision(hardwareMap, telemetryData);
        mDrive = new Drive(follower);

        // LOGIC: Determine Tag ID based on the alliance variable
        int targetTagId = (alliance == Alliance.BLUE) ? 20 : 24;
        mVision.setTargetTagId(targetTagId);

        register(mShooter, mIntake, mFeeder, mVision, mDrive);

        // 4. Initialize Controller
        controller = new GamepadEx(gamepad1);

        // 5. Bind Buttons
        bindButtons();
    }

    @Override
    public void run() {
        // 1. CRITICAL: Read Controller Inputs
        controller.readButtons();

        // Only manual drive if NOT aligning (holding X)
        // Note: Command-based usually handles this via 'requirements', but this manual check is safe.
        if (!gamepad1.x) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        }

        // 2. Run Scheduler
        super.run();

        // 3. Update Telemetry
        telemetryData.update();
        telemetry.update();
    }

    private void bindButtons() {
        mDrive.setDefaultCommand(new TeleopDriveCommand(mDrive, () -> -gamepad1.left_stick_y, () -> -gamepad1.left_stick_x, () -> -gamepad1.right_stick_x));

        controller.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new AlignToTagCommand(
                        mVision,
                        mDrive,
                        () -> -gamepad1.left_stick_y,
                        () -> -gamepad1.left_stick_x,
                        (alliance == Alliance.BLUE) ? 130.0 : 50.0, alliance
                ).alongWith(new RunShooterDistanceCommand(mShooter, mVision), new ShootFeedCommand(mFeeder, mIntake, mShooter, mDrive::isAligned)));

        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new RunFeederCommand(mFeeder));

        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new RunIntakeCommand(mIntake));
    }
}
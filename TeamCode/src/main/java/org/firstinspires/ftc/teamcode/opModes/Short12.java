package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.subsystems.Led.RobotState.SHOOTER_IDLE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.commands.AlignToTagCommand;
import org.firstinspires.ftc.teamcode.commands.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.RunShooterDistanceCommand;
import org.firstinspires.ftc.teamcode.commands.ShootFeedCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

// Using the Alliance Enum from your BaseShooterOpMode
import org.firstinspires.ftc.teamcode.opModes.BaseShooterOpMode.Alliance;

/**
 * Short12
 * Contains all auto logic for Blue and Red.
 * Extend this class and pass the Alliance in the constructor to create the OpMode.
 */
public abstract class Short12 extends CommandOpMode {

    protected final Alliance alliance;

    private Follower follower;
    private TelemetryData telemetryData;

    // Subsystems
    private Shooter mShooter;
    private Intake mIntake;
    private Feeder mFeeder;
    private Vision mVision;
    private Drive mDrive;
    private Led mLed;

    // Path chains
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

    public Short12(Alliance alliance) {
        this.alliance = alliance;
    }

    // ---------------------------------------------------------------------------------------------
    // MIRRORING HELPERS (Moved from BaseAuto)
    // ---------------------------------------------------------------------------------------------

    /**
     * Creates a Pose based on the alliance.
     * If RED, it flips the X coordinate (144 - x) and mirrors the heading (180 - heading).
     * Input heading is expected in DEGREES.
     */
    public Pose pose(double x, double y, double headingDeg) {
        if (alliance == Alliance.RED) {
            double mirroredX = 144.0 - x;
            double mirroredHeading = 180.0 - headingDeg;
            return new Pose(mirroredX, y, Math.toRadians(mirroredHeading));
        }
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    /**
     * Creates a Point (Pose without heading) for Bezier Control Points.
     * If RED, it flips the X coordinate.
     */
    public Pose point(double x, double y) {
        if (alliance == Alliance.RED) {
            return new Pose(144.0 - x, y);
        }
        return new Pose(x, y);
    }

    /**
     * Converts degrees to radians, mirroring the angle if Alliance is RED.
     */
    public double radians(double degrees) {
        if (alliance == Alliance.RED) {
            return Math.toRadians(180.0 - degrees);
        }
        return Math.toRadians(degrees);
    }

    /**
     * Returns the heading in DEGREES, mirrored if RED.
     */
    public double degrees(double degrees) {
        if (alliance == Alliance.RED) {
            return 180.0 - degrees;
        }
        return degrees;
    }

    // ---------------------------------------------------------------------------------------------
    // PATH BUILDING
    // ---------------------------------------------------------------------------------------------

    public void buildPaths() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(26.341, 131.049), new Pose(58.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(133))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(58.000, 84.000), new Pose(16.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.000, 84.000), new Pose(15.000, 76.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15.000, 76.000), new Pose(58.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(133))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(58.000, 84.000),
                                new Pose(63.000, 53.000),
                                new Pose(12.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(12.000, 60.000), new Pose(58.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(58.000, 84.000),
                                new Pose(63.000, 28.000),
                                new Pose(12.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(12.000, 36.000), new Pose(58.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(131))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(58.000, 84.000), new Pose(32.000, 70.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(131), Math.toRadians(180))
                .build();
    }

    // ---------------------------------------------------------------------------------------------
    // INITIALIZATION & RUN
    // ---------------------------------------------------------------------------------------------

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);

        CommandScheduler.getInstance().reset();
        telemetryData = new TelemetryData(telemetry);

        mShooter = new Shooter(hardwareMap, telemetryData);
        mIntake = new Intake(hardwareMap);
        mFeeder = new Feeder(hardwareMap);
        mVision = new Vision(hardwareMap, telemetryData);
        mDrive = new Drive(follower);
        mLed = new Led(hardwareMap);

        register(mShooter, mIntake, mFeeder, mVision, mDrive, mLed);

        buildPaths();

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        // Initialize Follower with the mirrored start pose
        follower.setPose(pose(26.341, 131.049, 144));

        schedule(
                new InstantCommand(() -> mLed.setState(SHOOTER_IDLE)),
                new RunCommand(() -> follower.update()),
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, Path1).alongWith(new WaitCommand(1000).andThen(new InstantCommand(() -> mShooter.setTargetVelocity(2700.0)))),
                        alignAndShoot().withTimeout(2750),
                        new RunIntakeCommand(mIntake).raceWith(new FollowPathCommand(follower, Path2)),
                        new FollowPathCommand(follower, Path3),
                        new WaitCommand(750),
                        new FollowPathCommand(follower, Path4).alongWith(new InstantCommand(() -> mShooter.setTargetVelocity(2700.0))),
                        alignAndShoot().withTimeout(2750),
                        new RunIntakeCommand(mIntake).raceWith(new FollowPathCommand(follower, Path5)),
                        new FollowPathCommand(follower, Path6).alongWith(new WaitCommand(500).andThen(new InstantCommand(() -> mShooter.setTargetVelocity(2700.0)))),
                        alignAndShoot().withTimeout(2750),
                        new RunIntakeCommand(mIntake).raceWith(new FollowPathCommand(follower, Path6)),
                        new FollowPathCommand(follower, Path8).alongWith(new WaitCommand(1000).andThen(new InstantCommand(() -> mShooter.setTargetVelocity(2700.0)))),
                        alignAndShoot().withTimeout(2750),
                        new FollowPathCommand(follower, Path9)
                )
        );
    }

    @Override
    public void initialize_loop() {
        mLed.periodic();
    }

    @Override
    public void run() {
        super.run();
        PoseStorage.currentPose = follower.getPose();
        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryData.update();
    }

    public Command alignAndShoot() {
        return new AlignToTagCommand(
                mVision,
                mDrive,
                () -> 0.0,
                () -> 0.0,
                degrees(130.0),
                alliance
        ).alongWith(new RunShooterDistanceCommand(mShooter, mVision), new ShootFeedCommand(mFeeder, mIntake, mShooter, mLed, mDrive::isAligned));
    }
}
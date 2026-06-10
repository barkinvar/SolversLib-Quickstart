package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.function.BooleanSupplier;

/**
 * Runs the feeder + intake to shoot while the shooter is at speed and aligned.
 * Uses tolerance hysteresis: a tight tolerance is required to START feeding,
 * then a looser tolerance keeps feeding so a brief velocity dip from a ball
 * passing through doesn't kick us back into warmup mid-burst.
 */
public class ShootFeedCommand extends CommandBase {

    private Shooter shooter;
    private Feeder feeder;
    private Intake intake;
    private Led led;
    private BooleanSupplier isAligned;

    // True once we've begun feeding; widens the tolerance window.
    private boolean isFeeding = false;

    public ShootFeedCommand(Feeder feeder, Intake intake, Shooter shooter, Led led, BooleanSupplier isAligned) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.intake = intake;
        this.isAligned = isAligned;
        this.led = led;
        // Declare subsystem dependencies so the scheduler knows
        // this command requires the Feeder and Intake subsystems.
        addRequirements(feeder, intake);
    }

    @Override
    public void initialize() {
        feeder.setSpeed(0.0);
        intake.setSpeed(0.0);
        isFeeding = false;
        led.setState(Led.RobotState.SHOOTER_WARMING_UP);
    }

    @Override
    public void execute() {
        boolean highGoal = shooter.getTargetVelocity() > 3000.0;

        // Tight tolerance to START, loose tolerance to KEEP feeding.
        double startTol = highGoal ? 175.0 : 325.0;
        double keepTol  = highGoal ? 500.0 : 510.0;
        double tol = isFeeding ? keepTol : startTol;

        if (Math.abs(shooter.getError()) < tol && isAligned.getAsBoolean()) {
            feeder.setSpeed(1.0);
            intake.setSpeed(1.0);
            isFeeding = true;
            led.setState(Led.RobotState.SHOOTER_READY);
        } else {
            feeder.setSpeed(0.0);
            intake.setSpeed(0.0);
            isFeeding = false;
            led.setState(Led.RobotState.SHOOTER_WARMING_UP);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends (button released).
        feeder.setSpeed(0.0);
        intake.setSpeed(0.0);
        isFeeding = false;
        led.setState(Led.RobotState.SHOOTER_IDLE);
    }

    @Override
    public boolean isFinished() {
        // Run until explicitly interrupted (button released).
        return false;
    }
}
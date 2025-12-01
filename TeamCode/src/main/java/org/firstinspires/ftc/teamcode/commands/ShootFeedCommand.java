package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.function.BooleanSupplier;

/**
 * A simple command that runs the shooter at a specific velocity while active,
 * and stops the shooter when it ends.
 */
public class ShootFeedCommand extends CommandBase {

    private Shooter shooter;
    private Feeder feeder;
    private Intake intake;
    private Led led;
    private  BooleanSupplier isAligned;


    public ShootFeedCommand(Feeder feeder, Intake intake, Shooter shooter, Led led,BooleanSupplier isAligned) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.intake = intake;
        this.isAligned = isAligned;
        this.led = led;
        // Declare subsystem dependency so the scheduler knows
        // this command requires the Shooter subsystem.
        addRequirements(feeder, intake);
    }

    @Override
    public void initialize() {
        feeder.setSpeed(0.0);
        intake.setSpeed(0.0);
        led.setState(Led.RobotState.SHOOTER_WARMING_UP);
    }

    @Override
    public void execute() {
        if (shooter.getTargetVelocity() < 3000.0 && Math.abs(shooter.getError()) < 175.0 && isAligned.getAsBoolean()) {
            feeder.setSpeed(1.0);
            intake.setSpeed(1.0);
            led.setState(Led.RobotState.SHOOTER_READY);
        } else if (shooter.getTargetVelocity() > 3000.0 && Math.abs(shooter.getError()) < 125.0 && isAligned.getAsBoolean()) {
            feeder.setSpeed(1.0);
            intake.setSpeed(1.0);
            led.setState(Led.RobotState.SHOOTER_READY);
        } else {
            feeder.setSpeed(0.0);
            intake.setSpeed(0.0);
            led.setState(Led.RobotState.SHOOTER_WARMING_UP);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends (button released)
        feeder.setSpeed(0.0);
        intake.setSpeed(0.0);
        led.setState(Led.RobotState.SHOOTER_IDLE);
    }

    @Override
    public boolean isFinished() {
        // Return false so the command runs until it is explicitly interrupted
        return false;
    }
}
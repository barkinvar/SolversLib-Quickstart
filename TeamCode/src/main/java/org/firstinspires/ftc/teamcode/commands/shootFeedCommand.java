package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * A simple command that runs the shooter at a specific velocity while active,
 * and stops the shooter when it ends.
 */
public class shootFeedCommand extends CommandBase {

    private Shooter shooter;
    private Feeder feeder;
    private Intake intake;
    private int wait = 0;

    public shootFeedCommand(Feeder feeder, Intake intake, Shooter shooter) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.intake = intake;

        // Declare subsystem dependency so the scheduler knows
        // this command requires the Shooter subsystem.
        addRequirements(feeder, intake);
    }

    @Override
    public void initialize() {
        feeder.setSpeed(0.0);
        intake.setSpeed(0.0);
        wait = 0;
    }

    @Override
    public void execute() {
        if (wait < 2) {
            wait++;
        }

        if (Math.abs(shooter.getError()) < 175.0 && wait >= 2) {
            feeder.setSpeed(1.0);
            intake.setSpeed(1.0);
        } else {
            feeder.setSpeed(0.0);
            intake.setSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends (button released)
        feeder.setSpeed(0.0);
        intake.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        // Return false so the command runs until it is explicitly interrupted
        return false;
    }
}
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
public class EmergencyCommand extends CommandBase {

    private Shooter shooter;
    private Feeder feeder;
    private Intake intake;


    public EmergencyCommand(Feeder feeder, Intake intake, Shooter shooter) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.intake = intake;
        // Declare subsystem dependency so the scheduler knows
        // this command requires the Shooter subsystem.
        addRequirements(feeder, intake, shooter);
    }

    @Override
    public void initialize() {
        feeder.setSpeed(-1.0);
        intake.setSpeed(-1.0);
        shooter.setTargetVelocity(4000.0);
    }

    @Override
    public void execute() {
        feeder.setSpeed(-1.0);
        intake.setSpeed(-1.0);
        shooter.setTargetVelocity(4000.0);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setSpeed(0.0);
        intake.setSpeed(0.0);
        shooter.setTargetVelocity(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
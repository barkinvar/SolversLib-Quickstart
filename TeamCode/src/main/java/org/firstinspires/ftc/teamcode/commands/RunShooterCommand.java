package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.seattlesolvers.solverslib.command.CommandBase;

/**
 * A simple command that runs the shooter at a specific velocity while active,
 * and stops the shooter when it ends.
 */
public class RunShooterCommand extends CommandBase {

    private final Shooter shooter;
    private final double targetVelocity;

    /**
     * @param shooter        The subsystem instance
     * @param targetVelocity The speed in Ticks Per Second to run the shooter at
     */
    public RunShooterCommand(Shooter shooter, double targetVelocity) {
        this.shooter = shooter;
        this.targetVelocity = targetVelocity;
        // Declare subsystem dependency so the scheduler knows
        // this command requires the Shooter subsystem.
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetVelocity(targetVelocity);
    }

    @Override
    public void execute() {
        shooter.setTargetVelocity(targetVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends (button released)
        shooter.setTargetVelocity(0.0);
    }

    @Override
    public boolean isFinished() {
        // Return false so the command runs until it is explicitly interrupted
        return false;
    }
}
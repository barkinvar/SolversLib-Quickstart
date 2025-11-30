package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends CommandBase {

    private final Drive m_drive;
    private final DoubleSupplier m_forwardInput;
    private final DoubleSupplier m_strafeInput;
    private final DoubleSupplier m_turnInput;

    /**
     * Standard Teleop Drive Command.
     *
     * @param drive       The drive subsystem
     * @param forward     Supplier for forward/backward movement (usually -gamepad.left_stick_y)
     * @param strafe      Supplier for left/right movement (usually gamepad.left_stick_x)
     * @param turn        Supplier for turning (usually gamepad.right_stick_x)
     */
    public TeleopDriveCommand(Drive drive, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        m_drive = drive;
        m_forwardInput = forward;
        m_strafeInput = strafe;
        m_turnInput = turn;

        // Register the subsystem so the scheduler knows this command requires the Drive
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        // PedroPathing specific: Ensure the follower is in TeleOp mode
        m_drive.startTeleopDrive();
    }

    @Override
    public void execute() {
        // Get the current values from the gamepads
        double forward = m_forwardInput.getAsDouble();
        double strafe = m_strafeInput.getAsDouble();
        double turn = m_turnInput.getAsDouble();

        // Pass values to the subsystem
        m_drive.joystickDrive(forward, strafe, turn);
    }

    @Override
    public boolean isFinished() {
        // Default commands should never finish on their own
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Optional: Stop the robot when the command ends
        m_drive.joystickDrive(0, 0, 0);
    }
}
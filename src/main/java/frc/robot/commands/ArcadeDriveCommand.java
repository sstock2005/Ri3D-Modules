package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ArcadeDriveCommand extends Command
{
    private final DrivetrainSubsystem m_drivetrain;
    
    private double m_forward;
    private double m_rotation;

    public ArcadeDriveCommand(DrivetrainSubsystem drivetrain, double forward, double rotation) {
        m_drivetrain = drivetrain;
        m_forward = forward;
        m_rotation = rotation;
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drivetrain.driveArcade(m_forward, m_rotation);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.driveArcade(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
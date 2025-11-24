package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase
{
    SparkMax leftLeader;
    SparkMax leftFollower;
    SparkMax rightLeader;
    SparkMax rightFollower;

    public DrivetrainSubsystem()
    {
        // Initialize the SPARKs
        leftLeader = new SparkMax(Constants.DrivetrainConstants.kLeftLeaderPort, MotorType.kBrushless);
        leftFollower = new SparkMax(Constants.DrivetrainConstants.kLeftFollowerPort, MotorType.kBrushless);
        rightLeader = new SparkMax(Constants.DrivetrainConstants.kRightLeaderPort, MotorType.kBrushless);
        rightFollower = new SparkMax(Constants.DrivetrainConstants.kLeftFollowerPort, MotorType.kBrushless);

        // Create new SPARK MAX configuration objects.
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
        SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

        // Set default parameters and set left leader config
        globalConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);
        
        // Apply global config and invert since it is on the opposite side
        rightLeaderConfig
            .apply(globalConfig)
            .inverted(true);
        
        // Apply global config and set leader for follower mode
        leftFollowerConfig
            .apply(globalConfig)
            .follow(leftLeader);

        // Apply the global config and set the leader for follower mode
        rightFollowerConfig
            .apply(globalConfig)
            .follow(rightLeader);


        // Apply configuration
        // kResetSafeParameters is used to get to a known state.
        // kPersistParameters is used to ensure config
        leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Left Out", leftLeader.getAppliedOutput());
        SmartDashboard.putNumber("Right Out", rightLeader.getAppliedOutput());
    }

    public void driveArcade(double forward, double rotation)
    {
        leftLeader.set(forward + rotation);
        rightLeader.set(forward - rotation);
    }
}
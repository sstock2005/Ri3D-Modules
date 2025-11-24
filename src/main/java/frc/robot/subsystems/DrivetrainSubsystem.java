package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase
{
    private SparkMax m_leftMotorLeader;
    private SparkMax m_rightMotorLeader;
    private SparkMax m_leftMotorFollower;
    private SparkMax m_rightMotorFollower;
    private DifferentialDrive m_robotDrive;

    //Outreach drive speed limit
    //IMPORTANT: Change the number below to change the drive speed limit percentage
    public double driveSpeedLimit = 0.35; //0.5 means 50% speed
    /* To deploy code
     * 1. Ensure you are connected to the robot Wifi Ri3D_9999
     * 2. Press ctrl-shift-p at the same time 
     * 3. Type deploy (if there isn't a > symbol in the search bar, >deploy)
     * 4. Select: WPILib: Deploy Robot Code
     * 5. If it fails, repeat step 1
     */

    /* To pull code from github
     * 1. Connect to Wifi with internet 
     * 2. Select the branch thing on the left side in VS Code
     * 3. Select the 3 dots above the message box (next to Source Control)
     * 4. Select pull
     * 5. If a pop up asks if you want to build, select yes
     */

    //if the speed limit is removed, put the /2 back for the x-axis in line 27 of RobotContainer

    public DrivetrainSubsystem()
    {
        m_leftMotorLeader = new SparkMax(Constants.DrivetrainConstants.LEFT_MOTOR_PORT, MotorType.kBrushless);
        m_rightMotorLeader = new SparkMax(Constants.DrivetrainConstants.RIGHT_MOTOR_PORT, MotorType.kBrushless);
        m_rightMotorFollower = new SparkMax(Constants.DrivetrainConstants.RIGHT_FOLLOWER_PORT, MotorType.kBrushless);
        m_leftMotorFollower = new SparkMax(Constants.DrivetrainConstants.LEFT_FOLLOWER_PORT, MotorType.kBrushless);
        
        SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
        rightLeaderConfig.inverted(true);
        m_rightMotorLeader.configure(rightLeaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
        rightFollowerConfig.follow(m_rightMotorLeader);
        m_rightMotorFollower.configure(rightFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
        leftFollowerConfig.follow(m_leftMotorLeader);
        m_leftMotorFollower.configure(leftFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_robotDrive = new DifferentialDrive(m_leftMotorLeader::set, m_rightMotorLeader::set);
        SendableRegistry.addChild(m_robotDrive, m_leftMotorLeader);
        SendableRegistry.addChild(m_robotDrive, m_rightMotorLeader);
    }

    @Override
    public void periodic()
    {
        // SmartDashboard.putNumber("Left Motor", m_leftMotorLeader.get());
        // SmartDashboard.putNumber("Right Motor", m_rightMotorLeader.get());
    }

    public void driveArcade(double left, double right)
    {
        m_robotDrive.arcadeDrive(left*driveSpeedLimit, right*driveSpeedLimit);
    }
}
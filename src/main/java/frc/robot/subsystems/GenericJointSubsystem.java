package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.kCanBusName;
import static frc.robot.Constants.JointConstants.kJointMotorPort;
import static frc.robot.Constants.JointConstants.kJointMotorCanPort;
import static frc.robot.Constants.JointConstants.kP;
import static frc.robot.Constants.JointConstants.kRotorToSensorRatio;
import static frc.robot.Constants.JointConstants.kS;
import static frc.robot.Constants.JointConstants.kTolerance;
import static frc.robot.Constants.JointConstants.kI;
import static frc.robot.Constants.JointConstants.kA;
import static frc.robot.Constants.JointConstants.kD;
import static frc.robot.Constants.JointConstants.kG;
import static frc.robot.Constants.JointConstants.kV;
import static frc.robot.Constants.JointConstants.kJointMotorStatorCurrentLimit;
import static frc.robot.Constants.JointConstants.kJointMotorSupplyCurrentLimit;
import static frc.robot.Constants.JointConstants.kLowerLimit;
import static frc.robot.Constants.JointConstants.kMotionMagicAcceleration;
import static frc.robot.Constants.JointConstants.kMotionMagicCruiseVel;
import static frc.robot.Constants.JointConstants.kUpperLimit;

/*
 * Generic Joint Subsystem
 * 
 * The subsystem for a generic PID controlled drive to position joint with limits.
 * Modify as needed for use case.
 */
public class GenericJointSubsystem extends SubsystemBase 
{
    private TalonFX jointMotor;
    private TalonFXConfiguration jointMotorConfig;
    private CANcoder jointEncoder;
    private final MotionMagicVoltage mRequest;
    private double lastSetRotation = 0;

    public GenericJointSubsystem()
    {
        /*
         * Initilize items
         */
        jointMotor = new TalonFX(kJointMotorPort, kCanBusName);
        jointEncoder = new CANcoder(kJointMotorCanPort, kCanBusName);
        mRequest = new MotionMagicVoltage(0);

        /*
         * Configure items
         */
        jointMotorConfig = new TalonFXConfiguration()
            // Encoder config
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackRemoteSensorID(jointEncoder.getDeviceID())
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withRotorToSensorRatio(kRotorToSensorRatio)
            )
            // Config current limits
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(kJointMotorStatorCurrentLimit)
                    .withSupplyCurrentLimit(kJointMotorSupplyCurrentLimit)
            )
            // Config motor software limits
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(kUpperLimit) // in encorder rotations
                    .withReverseSoftLimitThreshold(kLowerLimit) // in encorder rotations
            )
            // Config neutral mode and inverted settings
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive) // not inverted
            )
            // Closed loop control config
            .withClosedLoopGeneral(
                new ClosedLoopGeneralConfigs()
                    .withContinuousWrap(false)
            )
            // Closed loop control PID and feedback config
            .withSlot0(
                new Slot0Configs()
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKP(kP)
                    .withKI(kI)
                    .withKD(kD)
                    .withKG(kG)
                    .withKA(kA)
                    .withKS(kS)
                    .withKV(kV)
            )
            // Closed loop motion magic config
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(kMotionMagicAcceleration)
                    .withMotionMagicCruiseVelocity(kMotionMagicCruiseVel)
            );

        /*
         * Apply Configurations
         */
        jointMotor.getConfigurator().apply(jointMotorConfig);
    }

    /*
     * Go to position using the built in CTR motion magic. 
     * Uses motors integrated PID and feed forward values
     */
    public void setToPosition(double encoderRotations)
    {
        // Ensure set within tolerance, if not, force in tolerance and throw error
        if (encoderRotations > kUpperLimit) {
            DriverStation.reportError("Tried to set joint above upper limit: " + encoderRotations, false);
            encoderRotations = kUpperLimit;
        }
        else if (encoderRotations < kLowerLimit) {
            DriverStation.reportError("Tried to set joint under lower limit: " + encoderRotations, false);
            encoderRotations = kLowerLimit;
        }

        // Update last set rotation
        lastSetRotation = encoderRotations;

        // Command go to position
        jointMotor.setControl(mRequest.withPosition(encoderRotations));
    }

    @Override
    public void periodic() 
    {
        // Get current wrist error
        double currentPosition = jointEncoder.getPosition().getValueAsDouble();
        double error = currentPosition - lastSetRotation;

        // Verify if joint is in tolerance, if not, set position.
        if (error > kTolerance || error < -kTolerance) {
            setToPosition(lastSetRotation);
        }
    }
}

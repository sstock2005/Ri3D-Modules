package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;

public class Constants 
{
    public static final int kDriverControllerPort = 0;

    public static final String kCanBusName = "rio";

    public static final class DrivetrainConstants
    {
        public static final int kLeftLeaderPort = 2;
        public static final int kLeftFollowerPort = 3;
        public static final int kRightLeaderPort = 9;
        public static final int kRightFollowerPort = 4;
        public static final double kStickDeadband = 0.1;
    }

    public static final class JointConstants 
    {
        // Motor & Can
        public static final int kJointMotorPort = 0;
        public static final int kJointMotorCanPort = 0;

        // PID & Feed forward
        public static final double kP = 0; // Proportional Gain
        public static final double kI = 0; // Integral Gain
        public static final double kD = 0; // Derivative Gain
        public static final double kG = 0; // Gravity Feedforward/Feedback Gain
        public static final double kS = 0; // Static Feedforward Gain
        public static final double kA = 0; // Acceleration Feedforward Gain
        public static final double kV = 0; // Velocity Feedforward Gain

        // Motion magic targets
        public static final double kMotionMagicAcceleration = 0; // Target acceleration Motion Magic based control modes are allowed to use
        public static final double kMotionMagicCruiseVel = 0; // Maximum velocity Motion Magic based control modes are allowed to use

        // Gear ratio accounting
        public static final double kRotorToSensorRatio = 1 / 5; // Set to gear ratio, example is 5:1 reduction

        // Tolerance as encoder rotations
        public static final int kTolerance = 0;

        // Limits in encoder ticks
        public static final int kUpperLimit = 0;
        public static final int kLowerLimit = 0;

        // Current limits
        public static final Current kJointMotorStatorCurrentLimit = Amps.of(0);
        public static final Current kJointMotorSupplyCurrentLimit = Amps.of(0);
    }
}
package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Drivebase {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot

        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final SwerveModuleConstants Mod0 = //TODO: This must be tuned to specific robot
            new SwerveModuleConstants(
                1, /* Front Left Drive Motor ID */
                2, /* Front Left Angle Motor ID */
                1, /* Front Left Encoder ID */
                Rotation2d.fromDegrees(0.0)); /* Front Left Angle Offset */

        /* Front Right Module - Module 1 */
        public static final SwerveModuleConstants Mod1 = //TODO: This must be tuned to specific robot
            new SwerveModuleConstants(
                3, /* Front Right Drive Motor ID */
                4, /* Front Right Angle Motor ID */
                2, /* Front Right Encoder ID */
                Rotation2d.fromDegrees(0.0)); /* Front Right Angle Offset */

        /* Back Left Module - Module 2 */
        public static final SwerveModuleConstants Mod2 = //TODO: This must be tuned to specific robot
            new SwerveModuleConstants(
                5, /* Back Left Drive Motor ID */
                6, /* Back Left Angle Motor ID */
                3, /* Back Left Encoder ID */
                Rotation2d.fromDegrees(0.0)); /* Back Left Angle Offset */

        /* Back Right Module - Module 3 */
        public static final SwerveModuleConstants Mod3 = //TODO: This must be tuned to specific robot
            new SwerveModuleConstants(
                7, /* Back Right Drive Motor ID */
                8, /* Back Right Angle Motor ID */
                4, /* Back Right Encoder ID */
                Rotation2d.fromDegrees(0.0)); /* Back Right Angle Offset */
    }

    public static final class Auto { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}

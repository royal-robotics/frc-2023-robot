package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final double stickRotationDeadband = 0.4;

    public static final double speedMultiplier = 1;
    public static final double slowMode = 0.25;

    public static final class Container {
        public static final int translationAxis = 1;
        public static final int strafeAxis = 0;
        public static final int rotationAxis = 2; 
        public static final int intakeTranslationAxis = 1; //up and down on left joystick
        public static final int armTranslationAxis = 5; //up and down on right joystick
    }

    public static final class Drivebase {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = 0.495;
        public static final double wheelBase = 0.495;

        public static final double frontLeftAngle = 328.9581298828125;
        public static final double frontRightAngle = 31.146240234375;
        public static final double backLeftAngle = 122.42889404296875;
        public static final double backRightAngle = 123.39019775390626;

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
                2, /* Front Left Drive Motor ID */
                0, /* Front Left Angle Motor ID */
                3, /* Front Left Encoder ID */
                Rotation2d.fromDegrees(frontLeftAngle)); /* Front Left Angle Offset */

        /* Front Right Module - Module 1 */
        public static final SwerveModuleConstants Mod1 = //TODO: This must be tuned to specific robot
            new SwerveModuleConstants(
                1, /* Front Right Drive Motor ID */
                7, /* Front Right Angle Motor ID */
                2, /* Front Right Encoder ID */
                Rotation2d.fromDegrees(frontRightAngle)); /* Front Right Angle Offset */

        /* Back Left Module - Module 2 */
        public static final SwerveModuleConstants Mod2 = //TODO: This must be tuned to specific robot
            new SwerveModuleConstants(
                3, /* Back Left Drive Motor ID */
                4, /* Back Left Angle Motor ID */
                0, /* Back Left Encoder ID */
                Rotation2d.fromDegrees(backLeftAngle)); /* Back Left Angle Offset */

        /* Back Right Module - Module 3 */
        public static final SwerveModuleConstants Mod3 = //TODO: This must be tuned to specific robot
            new SwerveModuleConstants(
                6, /* Back Right Drive Motor ID */
                5, /* Back Right Angle Motor ID */
                1, /* Back Right Encoder ID */
                Rotation2d.fromDegrees(backRightAngle)); /* Back Right Angle Offset */
    }

    public static final class Auto { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 12;
        public static final double kPYController = 12;
        public static final double kPThetaController = 11;
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}

package src.test.java;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.beans.Transient;
import java.util.concurrent.TimeUnit;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

class PIDDriveTest {
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);

    private final ProfiledPIDController pitchController = new ProfiledPIDController(0.8, 0, 0, OMEGA_CONSTRAINTS);
    private final ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRAINTS);

    @BeforeEach // this method will run before each test
    void setup() {
        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Test
    @Timeout(value = 42, unit = TimeUnit.MILLISECONDS)
    void testPIDDrive() {
        Pose2d goalPose = new Pose2d(4.20939, 27.43, new Rotation2d(Units.degreesToRadians(230)));
        Pose2d currentPose = new Pose2d(-2.2, 14, new Rotation2d(Units.degreesToRadians(199)));

        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());


        System.out.println(currentPose.getX());
        System.out.println(currentPose.getY());
        System.out.println(currentPose.getRotation().getDegrees());
        System.out.println("---------");

        for (int i = 1; i <= 1000; i++) {
            double xError = xController.calculate(currentPose.getX());
            double yError = yController.calculate(currentPose.getY());
            double omegaError = omegaController.calculate(currentPose.getRotation().getRadians());

            // System.out.println(xError);
            // System.out.println(yError);
            // System.out.println(omegaError);
            // System.out.println("---------");

            currentPose = new Pose2d(currentPose.getX()+xError, 
                                        currentPose.getY()+yError, 
                                        new Rotation2d(currentPose.getRotation().getRadians()+omegaError));
        }
        System.out.println(currentPose.getX());
        System.out.println(currentPose.getY());
        System.out.println(currentPose.getRotation().getDegrees());
        assertEquals(goalPose, currentPose);
    }


}

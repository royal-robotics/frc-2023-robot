package frc.robot.sensors;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;


// Raw camera stream: 10.25.22.11:5800
// Dashboard: 10.25.22.11:5801
public class Limelight {
    private NetworkTable _table = NetworkTableInstance.getDefault().getTable("limelight");

    // We should start with the visible pipeline
    static {
        var table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("pipeline").setNumber(0);
    }

    public boolean onTarget() {
        return _table.getEntry("tv").getDouble(0) != 0;
    }

    public double targetX() {
        return _table.getEntry("tx").getDouble(0);
    }

    public double targetY() {
        return _table.getEntry("ty").getDouble(0);
    }

    public double targetArea() {
        return _table.getEntry("ta").getDouble(0);
    }

    public void setPipeline(double pipeline) {
        _table.getEntry("pipeline").setNumber(pipeline);
    }

    public double hasTarget() {
        return _table.getEntry("tv").getDouble(-1);
    }

    public Pose3d getPoseBlue() {
        if (onTarget()) {
            double[] botpose = _table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            // Make sure array length is correct
            if (botpose.length == 6) {
                return new Pose3d(
                    new Translation3d(botpose[0], botpose[1], botpose[2]),
                    new Rotation3d(
                        Units.degreesToRadians(botpose[3]),
                        Units.degreesToRadians(botpose[4]),
                        Units.degreesToRadians(botpose[5]))
                );
            }
        }
        return new Pose3d();
    }

    public Pose3d getPoseRed() {
        if (onTarget()) {
            double[] botpose = _table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
            // Make sure array length is correct
            if (botpose.length == 6) {
                return new Pose3d(
                    new Translation3d(botpose[0], botpose[1], botpose[2]),
                    new Rotation3d(
                        Units.degreesToRadians(botpose[3]),
                        Units.degreesToRadians(botpose[4]),
                        Units.degreesToRadians(botpose[5]))
                );
            }
        }
        return new Pose3d();
    }

    public Pose3d botPoseTargetSpace() {
        if (onTarget()) {
            double[] botpose = _table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
            // Make sure array length is correct
            if (botpose.length == 6) {
                return new Pose3d(
                    new Translation3d(botpose[0], botpose[1], botpose[2]),
                    new Rotation3d(
                        Units.degreesToRadians(botpose[3]),
                        Units.degreesToRadians(botpose[4]),
                        Units.degreesToRadians(botpose[5]))
                );
            }
        }
        return new Pose3d();
    }

    public Pose3d targetPoseRobotSpace() {
        if (onTarget()) {
            double[] targetpose = _table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
            // Make sure array length is correct
            if (targetpose.length == 6) {
                return new Pose3d(
                    new Translation3d(targetpose[0], targetpose[1], targetpose[2]),
                    new Rotation3d(
                        Units.degreesToRadians(targetpose[3]),
                        Units.degreesToRadians(targetpose[4]),
                        Units.degreesToRadians(targetpose[5]))
                );
            }
        }
        return new Pose3d();
    }
}
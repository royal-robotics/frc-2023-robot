package frc.robot.sensors;

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

    public double[] getPoseBlue() {
        if (onTarget()) {
            double[] poseBlue = _table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            if (poseBlue.length < 6) {
                return new double[6];
            }
            return poseBlue;
        }
        return new double[6];
    }

    public double[] getPoseRed() {
        if (onTarget()) {
            double[] poseRed = _table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
            if (poseRed.length < 6) {
                return new double[6];
            }
            return poseRed;
        }
        return new double[6];
    }

    public double[] botPoseTargetSpace() {
        if (onTarget()) {
            double[] botPose = _table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
            if (botPose.length < 6) {
                return new double[6];
            }
            return botPose;
        }
        return new double[6];
    }

    public double[] targetPoseRobotSpace() {
        if (onTarget()) {
            double[] targetPose = _table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
            if (targetPose.length < 6) {
                return new double[6];
            }
            return targetPose;
        }   
        return new double[6];
    }
}
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
        double[] botpose = _table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        // Make sure array length is correct
        if (botpose.length == 6){
            return botpose;
        }
        return new double[6];
    }

    public double[] getPoseRed() {
        double[] botpose = _table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        // Make sure array length is correct
        if (botpose.length == 6){
            return botpose;
        }
        return new double[6];
    }

    public double[] botPoseTargetSpace() {
        double[] botpose = _table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        // Make sure array length is correct
        if (botpose.length == 6){
            return botpose;
        }
        return new double[6];
    }

    public double[] targetPoseRobotSpace() {
        double[] botpose = _table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        // Make sure array length is correct
        if (botpose.length == 6){
            return botpose;
        }
        return new double[6];
    }
}
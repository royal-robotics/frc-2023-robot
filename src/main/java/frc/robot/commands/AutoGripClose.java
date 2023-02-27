package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Arm;

public class AutoGripClose extends GripOpen{
    private Timer s_time;
    private double s_goalTime;
    public AutoGripClose(Arm arm, double goalTime) {
        super(arm);
        s_time = new Timer();
        s_goalTime = goalTime;
    }

    @Override
    public void initialize () {
        s_time.start();
    }
    @Override
    public boolean isFinished() {
        return s_time.hasElapsed(s_goalTime);
    }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class AutoGripUp extends GripUp {
    private Timer s_time;
    private double s_goalTime;
    public AutoGripUp(Arm arm, Intake intake, double goalTime) {
        super(arm, intake);
        s_time = new Timer();
        s_goalTime = goalTime;
    }

    @Override
    public void initialize () {
        s_time.reset();
        s_time.start();
    }
    @Override
    public boolean isFinished() {
        return s_time.hasElapsed(s_goalTime);
    }
}

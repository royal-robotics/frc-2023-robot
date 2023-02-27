package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class AutoExtendIntake extends ExtendIntake{
    private Timer s_time;
    private double goalTime;

    public AutoExtendIntake(Arm arm, Intake intake, double wheelSpeed, double goalTime) {
        super(arm, intake, wheelSpeed);
        s_time = new Timer();
        this.goalTime = goalTime;
    }
    
    @Override
    public void initialize () {
        s_time.start();
    }
    @Override
    public boolean isFinished() {
        return s_time.hasElapsed(goalTime);
    }
}

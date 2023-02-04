package frc.robot.autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.autonomous.autoModes.*;
import java.util.function.Supplier;

public class AutoModeSelector {
    private final SendableChooser<Supplier<SequentialCommandGroup>> _chooser;

    public AutoModeSelector(RobotContainer robotContainer) {
        _chooser = new SendableChooser<Supplier<SequentialCommandGroup>>();
        _chooser.addOption("autoMode", () -> new AutoMode(robotContainer));
        _chooser.addOption("autoMode2", () -> new AutoMode2(robotContainer));
        Shuffleboard.getTab("Competition").add("Auto Mode", _chooser).withPosition(0, 0).withSize(2, 1);
    }
    public SequentialCommandGroup getAutoMode() {
        return _chooser.getSelected().get();
    }
}

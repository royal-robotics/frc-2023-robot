package frc.robot.autonomous;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.RobotContainer;
import java.util.function.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.autoModes.*;

public class AutoModeSelector {
    private final SendableChooser<Supplier<SequentialCommandGroup>> _chooser;

    public AutoModeSelector(RobotContainer robotContainer) {
        _chooser = new SendableChooser<Supplier<SequentialCommandGroup>>();
        _chooser.addOption("autoMode", () -> new autoMode(robotContainer));
        Shuffleboard.getTab("Competition").add("Auto Mode", _chooser).withPosition(0, 0).withSize(2, 1);
    }
    public SequentialCommandGroup getAutoMode() {
        return _chooser.getSelected().get();
    }
}

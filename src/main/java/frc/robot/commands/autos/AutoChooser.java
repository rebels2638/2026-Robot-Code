package frc.robot.commands.autos;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.util.sendable.SendableRegistry;

public class AutoChooser {
    private static AutoChooser instance;

    public static AutoChooser getInstance() {
        if (instance == null) {
            instance = new AutoChooser();
        }
        return instance;
    }

    private final SendableChooser<Command> chooser = new SendableChooser<>();
    private final StringSubscriber customPathSub;
    private static final String NT_TABLE = "Auto";

    private AutoChooser() {
        registerAutos();

        var table = NetworkTableInstance.getDefault().getTable(NT_TABLE);

        SendableRegistry.add(chooser, NT_TABLE, "Chooser");
        var builder = new SendableBuilderImpl();
        builder.setTable(table.getSubTable("Chooser"));
        chooser.initSendable(builder);
        builder.startListeners();

        table.getEntry("CustomPath").setString("");
        customPathSub = table.getStringTopic("CustomPath").subscribe("");
    }

    private void registerAutos() {
        chooser.setDefaultOption("Do Nothing", Commands.none());

        for (String path : getAvailablePaths()) {
            chooser.addOption("Path: " + path, AutoBuilder.path(path));
        }

        chooser.addOption("Tower Left",
                AutoBuilder.withShooting(AutoBuilder.path("middle_start_to_tower_left")));

        chooser.addOption("Tower Middle",
                AutoBuilder.withShooting(AutoBuilder.path("middle_start_to_tower_middle")));

        chooser.addOption("Tower Right",
                AutoBuilder.withShooting(AutoBuilder.path("middle_start_to_tower_right")));

        chooser.addOption("Depot Shoot and Lob",
                AutoBuilder.withShooting(
                        AutoBuilder.paths("middle_start_to_depot_shoot", "outpost_to_neutral_zone_bottom_left_lob")));
    }

    public Command getSelectedAuto() {
        String customPath = customPathSub.get();

        if (customPath != null && !customPath.isEmpty()) {
            Logger.recordOutput("Auto/Selected", "Custom: " + customPath);
            return AutoBuilder.path(customPath);
        }

        Command selected = chooser.getSelected();
        Logger.recordOutput("Auto/Selected", selected != null ? selected.getName() : "None");

        return selected != null ? selected : Commands.none();
    }

    private List<String> getAvailablePaths() {
        List<String> paths = new ArrayList<>();
        File dir = new File(Filesystem.getDeployDirectory(), "autos/paths");

        if (dir.exists() && dir.isDirectory()) {
            File[] files = dir.listFiles((d, n) -> n.endsWith(".json"));
            if (files != null) {
                for (File f : files) {
                    paths.add(f.getName().replace(".json", ""));
                }
            }
        }
        return paths;
    }
}

package frc.robot.demo;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonParseException;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;

/**
 * Reads and writes calibration points as JSON.
 *
 * <p>On the roboRIO the live file is {@code /home/lvuser/demo-calibration.json}, outside the deploy
 * folder so redeploys keep it. If it is missing, a copy committed at
 * {@code src/main/deploy/demo-calibration.json} is used as the baseline.
 */
public final class CalibrationStore {
    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().create();
    private static final int FILE_VERSION = 1;

    public record LoadResult(List<CalibrationPoint> points, String source, String error) {}

    private static final class FileContents {
        int version = FILE_VERSION;
        List<PointJson> points = new ArrayList<>();
    }

    private static final class PointJson {
        double distanceMeters;
        double targetHeightMeters;
        double hoodDegrees;
        double flywheelRps;
        String savedAt;
    }

    private final Path primaryPath;
    private final Path fallbackPath;

    public CalibrationStore(Path primaryPath, Path fallbackPath) {
        this.primaryPath = primaryPath;
        this.fallbackPath = fallbackPath;
    }

    public static CalibrationStore forCurrentPlatform() {
        Path primaryDirectory = RobotBase.isReal()
            ? Path.of(DemoConstants.ROBORIO_CALIBRATION_DIRECTORY)
            : Filesystem.getOperatingDirectory().toPath();
        return new CalibrationStore(
            primaryDirectory.resolve(DemoConstants.CALIBRATION_FILE_NAME),
            Filesystem.getDeployDirectory().toPath().resolve(DemoConstants.CALIBRATION_FILE_NAME)
        );
    }

    public Path primaryPath() {
        return primaryPath;
    }

    public LoadResult load() {
        String primaryError = null;
        if (Files.exists(primaryPath)) {
            try {
                return new LoadResult(read(primaryPath), primaryPath.toString(), null);
            } catch (IOException | JsonParseException | IllegalStateException ex) {
                primaryError = "Could not read " + primaryPath + ": " + ex.getMessage();
            }
        }
        if (fallbackPath != null && Files.exists(fallbackPath)) {
            try {
                return new LoadResult(read(fallbackPath), fallbackPath.toString(), primaryError);
            } catch (IOException | JsonParseException | IllegalStateException ex) {
                String fallbackError = "Could not read " + fallbackPath + ": " + ex.getMessage();
                return new LoadResult(List.of(), "none", primaryError == null ? fallbackError : primaryError + "; " + fallbackError);
            }
        }
        return new LoadResult(List.of(), "none", primaryError);
    }

    /** Writes all points atomically (temp file then move). Returns an error message, or null on success. */
    public String save(List<CalibrationPoint> points) {
        FileContents contents = new FileContents();
        for (CalibrationPoint point : points) {
            PointJson json = new PointJson();
            json.distanceMeters = point.distanceMeters();
            json.targetHeightMeters = point.targetHeightMeters();
            json.hoodDegrees = point.hoodDegrees();
            json.flywheelRps = point.flywheelRps();
            json.savedAt = point.savedAt();
            contents.points.add(json);
        }
        try {
            Path parent = primaryPath.toAbsolutePath().getParent();
            if (parent != null) {
                Files.createDirectories(parent);
            }
            Path temp = primaryPath.resolveSibling(primaryPath.getFileName() + ".tmp");
            Files.writeString(temp, GSON.toJson(contents), StandardCharsets.UTF_8);
            Files.move(temp, primaryPath, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
            return null;
        } catch (IOException | UnsupportedOperationException ex) {
            return "Could not write " + primaryPath + ": " + ex.getMessage();
        }
    }

    private static List<CalibrationPoint> read(Path path) throws IOException {
        FileContents contents = GSON.fromJson(Files.readString(path, StandardCharsets.UTF_8), FileContents.class);
        List<CalibrationPoint> points = new ArrayList<>();
        if (contents == null || contents.points == null) {
            return points;
        }
        for (PointJson json : contents.points) {
            if (json == null) {
                continue;
            }
            CalibrationPoint point = new CalibrationPoint(
                json.distanceMeters,
                json.targetHeightMeters,
                json.hoodDegrees,
                json.flywheelRps,
                json.savedAt == null ? "" : json.savedAt
            );
            if (point.isValid()) {
                points.add(point);
            }
        }
        return points;
    }
}

package frc.robot.demo;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class CalibrationStoreTest {
    @TempDir
    Path tempDir;

    @Test
    void saveThenLoad_roundTripsPoints() {
        CalibrationStore store = new CalibrationStore(tempDir.resolve("live.json"), tempDir.resolve("deploy.json"));
        List<CalibrationPoint> points = List.of(
            new CalibrationPoint(2.0, 1.2, 52.2, 23.1, "a"),
            new CalibrationPoint(3.0, 2.0, 56.8, 30.4, "b")
        );

        assertNull(store.save(points));
        CalibrationStore.LoadResult result = store.load();

        assertEquals(points, result.points());
        assertEquals(tempDir.resolve("live.json").toString(), result.source());
        assertNull(result.error());
    }

    @Test
    void missingLiveFile_usesDeployBaseline() throws IOException {
        Path deploy = tempDir.resolve("deploy.json");
        new CalibrationStore(deploy, null).save(List.of(new CalibrationPoint(3.0, 1.2, 50.0, 27.0, "baseline")));
        CalibrationStore store = new CalibrationStore(tempDir.resolve("live.json"), deploy);

        CalibrationStore.LoadResult result = store.load();

        assertEquals(1, result.points().size());
        assertEquals(deploy.toString(), result.source());
    }

    @Test
    void corruptLiveFile_fallsBackAndReportsError() throws IOException {
        Path live = tempDir.resolve("live.json");
        Files.writeString(live, "{ not json");
        Path deploy = tempDir.resolve("deploy.json");
        new CalibrationStore(deploy, null).save(List.of(new CalibrationPoint(3.0, 1.2, 50.0, 27.0, "baseline")));

        CalibrationStore.LoadResult result = new CalibrationStore(live, deploy).load();

        assertEquals(1, result.points().size());
        assertNotNull(result.error());
    }

    @Test
    void noFiles_emptyWithoutError() {
        CalibrationStore.LoadResult result =
            new CalibrationStore(tempDir.resolve("a.json"), tempDir.resolve("b.json")).load();

        assertTrue(result.points().isEmpty());
        assertEquals("none", result.source());
        assertNull(result.error());
    }
}

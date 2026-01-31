package frc.robot.commands.autos;

import java.util.HashSet;
import java.util.Set;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoShootingZoneManager {
    private static AutoShootingZoneManager instance;

    public static AutoShootingZoneManager getInstance() {
        if (instance == null) {
            instance = new AutoShootingZoneManager();
        }
        return instance;
    }

    private final Set<Integer> activeZones = new HashSet<>();

    private AutoShootingZoneManager() {
    }

    public void enterShootingZone(int zoneId) {
        activeZones.add(zoneId);

        Logger.recordOutput("AutoShootingZone/active", true);
        Logger.recordOutput("AutoShootingZone/activeZones",
                activeZones.stream().mapToLong(Integer::longValue).toArray());
        Logger.recordOutput("AutoShootingZone/activeZoneCount", activeZones.size());
    }

    /**
     * Called when exiting a shooting zone via event trigger.
     *
     * @param zoneId The unique identifier for this zone
     */
    public void exitShootingZone(int zoneId) {
        activeZones.remove(zoneId);

        Logger.recordOutput("AutoShootingZone/active", !activeZones.isEmpty());
        Logger.recordOutput("AutoShootingZone/activeZones",
                activeZones.stream().mapToLong(Integer::longValue).toArray());
        Logger.recordOutput("AutoShootingZone/activeZoneCount", activeZones.size());
    }

    @AutoLogOutput(key = "AutoShootingZone/isInShootingZone")
    public boolean isInShootingZone() {
        return !activeZones.isEmpty();
    }

    /**
     * @return true if currently within a specific shooting zone
     */
    public boolean isInShootingZone(int zoneId) {
        return activeZones.contains(zoneId);
    }

    /**
     * Resets all zone state. Should be called at the start of autonomous.
     */
    public void reset() {
        activeZones.clear();

        Logger.recordOutput("AutoShootingZone/active", false);
        Logger.recordOutput("AutoShootingZone/activeZones", new long[0]);
        Logger.recordOutput("AutoShootingZone/activeZoneCount", 0);
        Logger.recordOutput("AutoShootingZone/reset", true);
    }
}

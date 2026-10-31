package frc.robot.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class PhoenixUtilTest {
    @Test
    void normalizeNetworkName_consolidatesRioCanBusAliases() {
        assertEquals("rio", PhoenixUtil.normalizeNetworkName(null));
        assertEquals("rio", PhoenixUtil.normalizeNetworkName(""));
        assertEquals("rio", PhoenixUtil.normalizeNetworkName("   "));
        assertEquals("rio", PhoenixUtil.normalizeNetworkName("rio"));
        assertEquals("rio", PhoenixUtil.normalizeNetworkName("RIO"));
    }

    @Test
    void normalizeNetworkName_preservesCustomCanivoreBusNames() {
        assertEquals("swerve", PhoenixUtil.normalizeNetworkName("swerve"));
        assertEquals("canivore1", PhoenixUtil.normalizeNetworkName(" canivore1 "));
    }
}


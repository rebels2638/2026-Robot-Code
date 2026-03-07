// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Supplier;

public class PhoenixUtil {
  private static final Map<String, BaseStatusSignal[]> registeredSignalsByNetwork =
      new LinkedHashMap<>();

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /** Registers a set of signals for synchronized refresh on a specific Phoenix network. */
  public static synchronized void registerSignals(String network, BaseStatusSignal... signals) {
    String normalizedNetwork = network == null ? "" : network;
    BaseStatusSignal[] registeredSignals =
        registeredSignalsByNetwork.getOrDefault(normalizedNetwork, new BaseStatusSignal[0]);
    BaseStatusSignal[] newSignals = new BaseStatusSignal[registeredSignals.length + signals.length];
    System.arraycopy(registeredSignals, 0, newSignals, 0, registeredSignals.length);
    System.arraycopy(signals, 0, newSignals, registeredSignals.length, signals.length);
    registeredSignalsByNetwork.put(normalizedNetwork, newSignals);
  }

  /** Refresh all registered signals once per main robot loop. */
  public static synchronized void refreshAll() {
    for (BaseStatusSignal[] registeredSignals : registeredSignalsByNetwork.values()) {
      if (registeredSignals.length > 0) {
        BaseStatusSignal.refreshAll(registeredSignals);
      }
    }
  }
}

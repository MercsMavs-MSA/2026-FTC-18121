package com.qualcomm.robotcore.hardware;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Simulated hardwareMap: devices are registered by name by the sim, like the robot configuration. */
public class HardwareMap {
    private final Map<String, Object> byName = new LinkedHashMap<>();
    private final List<Object> unnamed = new ArrayList<>();
    public final List<VoltageSensor> voltageSensor = new ArrayList<>();

    public void put(String name, Object device) { byName.put(name, device); }
    public void putUnnamed(Object device) { unnamed.add(device); }

    public <T> T get(Class<T> type, String name) {
        Object d = byName.get(name);
        if (d == null) throw new IllegalArgumentException("Unable to find a hardware device with name \"" + name + "\" (sim)");
        if (!type.isInstance(d)) throw new IllegalArgumentException("Device \"" + name + "\" is not a " + type.getSimpleName() + " (sim)");
        return type.cast(d);
    }

    public <T> List<T> getAll(Class<T> type) {
        List<T> out = new ArrayList<>();
        for (Object d : byName.values()) if (type.isInstance(d)) out.add(type.cast(d));
        for (Object d : unnamed) if (type.isInstance(d)) out.add(type.cast(d));
        return out;
    }
}

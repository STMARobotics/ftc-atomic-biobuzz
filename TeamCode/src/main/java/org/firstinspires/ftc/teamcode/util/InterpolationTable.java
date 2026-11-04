package org.firstinspires.ftc.teamcode.util;

import java.util.Map;
import java.util.TreeMap;

/**
 * Lookup table that linearly interpolates between points. Inputs outside the table use the
 * closest end point.
 */
public class InterpolationTable {

    private final TreeMap<Double, Double> points = new TreeMap<>();

    public InterpolationTable add(double x, double y) {
        points.put(x, y);
        return this;
    }

    public double get(double x) {
        if (points.isEmpty()) {
            throw new IllegalStateException("InterpolationTable has no points");
        }

        Map.Entry<Double, Double> low = points.floorEntry(x);
        Map.Entry<Double, Double> high = points.ceilingEntry(x);

        if (low == null) return high.getValue();
        if (high == null) return low.getValue();
        if (low.getKey().equals(high.getKey())) return low.getValue();

        double t = (x - low.getKey()) / (high.getKey() - low.getKey());
        return low.getValue() + t * (high.getValue() - low.getValue());
    }
}

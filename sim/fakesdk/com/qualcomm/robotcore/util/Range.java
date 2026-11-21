package com.qualcomm.robotcore.util;

public class Range {
    public static double clip(double v, double min, double max) { return Math.max(min, Math.min(max, v)); }
    public static float clip(float v, float min, float max) { return Math.max(min, Math.min(max, v)); }
    public static double scale(double n, double x1, double x2, double y1, double y2) {
        return y1 + (n - x1) * (y2 - y1) / (x2 - x1);
    }
}

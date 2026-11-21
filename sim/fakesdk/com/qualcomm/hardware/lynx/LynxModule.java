package com.qualcomm.hardware.lynx;

public class LynxModule {
    public enum BulkCachingMode { OFF, AUTO, MANUAL }
    public BulkCachingMode mode = BulkCachingMode.OFF;
    public int clears = 0;
    public void setBulkCachingMode(BulkCachingMode m) { mode = m; }
    public void clearBulkCache() { clears++; }
}

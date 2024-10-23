package com.team841.lib.util.atan2;


public class FastTrig
{
    // Constants for fast atan approximation
    private static final float PI = (float) Math.PI;
    private static final float HALF_PI = PI / 2;
    private static final float QUARTER_PI = PI / 4;
    private static final float THREE_QUARTER_PI = 3 * PI / 4;

    // Fast approximation for atan(y/x)
    private static float fastAtan(float z) {
        // Polynomial approximation of atan
        // atan(r) ≈ π/4 * r - r * (|r| - 1) / (r^2 + 0.28)
        float absZ = Math.abs(z);
        float result = QUARTER_PI * z - z * (absZ - 1) / (absZ * absZ + 0.28f);
        return result;
    }

    // Fast approximation for atan2(y, x)
    public static float fastAtan2(float y, float x) {
        // Handle edge cases for x = 0
        if (x == 0.0f) {
            if (y > 0.0f) return HALF_PI;
            if (y < 0.0f) return -HALF_PI;
            return 0.0f;  // Undefined atan2(0, 0)
        }

        // Compute atan of the ratio
        float atanResult = fastAtan(y / x);

        // Adjust result based on the quadrant
        if (x > 0) {
            return atanResult;
        } else {
            if (y >= 0) {
                return atanResult + PI;
            } else {
                return atanResult - PI;
            }
        }
    }
}
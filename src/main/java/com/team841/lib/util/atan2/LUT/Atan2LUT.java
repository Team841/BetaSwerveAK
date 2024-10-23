package com.team841.lib.util.atan2.LUT;

public class Atan2LUT {
    public static double getAtan2(double x, double y) {
        // Ensure inputs are within the valid range
        if (x < 0 || x > 16.54 || y < 0 || y > 8.21) {
            return 0;
        }

        // Calculate nearest indices in LUT based on 0.1 precision
        int xIndex = (int) Math.round(x * 10);  // Multiply by 10 for 0.1 precision
        int yIndex = (int) Math.round(y * 10);  // Multiply by 10 for 0.1 precision

        // Use appropriate segment based on yIndex
        if (yIndex < 50) {
            // Use first segment
            return Atan2LUTSegment1.atan2LUT[yIndex][xIndex];
        } else if (yIndex < 83) {
            // Use second segment, adjusting the yIndex by subtracting 50
            return Atan2LUTSegment2.atan2LUT[yIndex - 50][xIndex];
        } else {
            return 0;
        }
    }
}

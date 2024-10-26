public class Atan2LUT {
    public static double getAtan2(double x, double y) {
        if (x < 0 || x > 16.54 || y < 0 || y > 8.21) {
            throw new IllegalArgumentException("Inputs out of bounds");
        }
        int xIndex = (int)Math.round(x * 100);
        int yIndex = (int)Math.round(y * 100);
        if (yIndex < 100) {
            return Atan2LUTSegment1.atan2LUT[yIndex][xIndex];
        } else if (yIndex < 200) {
            return Atan2LUTSegment2.atan2LUT[yIndex - 100][xIndex];
        } else if (yIndex < 300) {
            return Atan2LUTSegment3.atan2LUT[yIndex - 200][xIndex];
        }
        else {
            throw new IllegalArgumentException("Invalid yIndex");
        }
    }
}

import numpy as np

# Parameters for LUT with 2 decimal precision
x_range = np.round(np.arange(0, 16.55, 0.1), 2)  # Steps of 0.01 for x from 0 to 16.54
y_range = np.round(np.arange(0, 8.22, 0.1), 2)   # Steps of 0.01 for y from 0 to 8.21

# Split the LUT into segments, each handling a portion of the y range
segment_size = 50  # Number of rows per segment (you can adjust this)
num_segments = len(y_range) // segment_size + 1

for segment_idx in range(num_segments):
    # Create a file for each segment
    with open(f"Atan2LUTSegment{segment_idx + 1}.java", "w") as f:

        f.write("package com.team841.lib.util.atan2.LUT;\n\n")

        # Write Java class header for each segment
        f.write(f"public class Atan2LUTSegment{segment_idx + 1} {{\n")

        # Write the 2D LUT array definition for this segment
        f.write("    // Segment of Look-Up Table for atan2 function\n")
        f.write("    public static final double[][] atan2LUT = {\n")

        # Write a portion of the LUT for this segment
        start_y = segment_idx * segment_size
        end_y = min(start_y + segment_size, len(y_range))

        for y_idx in range(start_y, end_y):
            y = y_range[y_idx]
            f.write("        {")
            for x in x_range:
                atan2_value = np.arctan2(y, x)
                f.write(f"{atan2_value:.8f}, ")
            f.write("},\n")

        # Close the LUT array and class
        f.write("    };\n")
        f.write("}\n")

    print(f"Atan2LUTSegment{segment_idx + 1}.java file has been generated.")

# Now create the main Atan2LUT class that decides which segment to use
with open("Atan2LUT.java", "w") as f:

    # Write Java class header
    f.write("public class Atan2LUT {\n")

    # Method to get the closest value from the segmented LUT using x and y
    f.write("    public static double getAtan2(double x, double y) {\n")
    f.write("        if (x < 0 || x > 16.54 || y < 0 || y > 8.21) {\n")
    f.write("            throw new IllegalArgumentException(\"Inputs out of bounds\");\n")
    f.write("        }\n")

    # Calculate nearest indices in LUT
    f.write("        int xIndex = (int)Math.round(x * 100);\n")
    f.write("        int yIndex = (int)Math.round(y * 100);\n")

    # Determine which segment to use based on yIndex
    f.write("        if (yIndex < 100) {\n")
    f.write("            return Atan2LUTSegment1.atan2LUT[yIndex][xIndex];\n")
    f.write("        } else if (yIndex < 200) {\n")
    f.write("            return Atan2LUTSegment2.atan2LUT[yIndex - 100][xIndex];\n")
    f.write("        } else if (yIndex < 300) {\n")
    f.write("            return Atan2LUTSegment3.atan2LUT[yIndex - 200][xIndex];\n")
    f.write("        }\n")
    # Add more segments as needed here...

    f.write("        else {\n")
    f.write("            throw new IllegalArgumentException(\"Invalid yIndex\");\n")
    f.write("        }\n")

    # Close the method and class
    f.write("    }\n")
    f.write("}\n")

print("Atan2LUT.java file has been generated.")

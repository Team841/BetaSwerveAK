import math

# Function to generate atan table for values from -20 to 20
def generate_atan_table():
    table = {}
    for y in range(-20, 21):
        for x in range(-20, 21):
            if x != 0 or y != 0:  # Avoid division by zero when x and y are both 0
                atan_value = math.atan2(y, x)  # Use atan2 for better handling of y/x
                table[(x, y)] = atan_value
    return table

# Function to export the table to a Java file
def export_to_java(table):
    with open("AtanTable.java", "w") as f:
        f.write("public class AtanTable {\n")
        f.write("    // Precomputed atan(y/x) table for x and y ranging from -20 to 20\n")
        f.write("    public static double lookupAtan(int x, int y) {\n")
        f.write("        switch(x) {\n")
        for x in range(-20, 21):
            f.write(f"            case {x}:\n")
            f.write("                switch(y) {\n")
            for y in range(-20, 21):
                if (x, y) in table:
                    f.write(f"                    case {y}: return {table[(x, y)]:.6f};\n")
            f.write("                }\n")
            f.write("                break;\n")
        f.write("        }\n")
        f.write("        throw new IllegalArgumentException(\"Out of bounds\");\n")
        f.write("    }\n")
        f.write("}\n")

# Main function
if __name__ == "__main__":
    atan_table = generate_atan_table()
    export_to_java(atan_table)
    print("AtanTable.java file generated successfully.")

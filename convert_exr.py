import OpenEXR
import Imath
import sys

if len(sys.argv) != 3:
    print("Usage: python convert_exr.py <input.exr> <output.exr>")
    sys.exit(1)

input_file = sys.argv[1]
output_file = sys.argv[2]

# Open the input file
exr_file = OpenEXR.InputFile(input_file)

# Get the header
header = exr_file.header()

# Get the data window
dw = header['dataWindow']
width = dw.max.x - dw.min.x + 1
height = dw.max.y - dw.min.y + 1

# Read the channels
pt = Imath.PixelType(Imath.PixelType.FLOAT)
r = exr_file.channel('R', pt)
g = exr_file.channel('G', pt)
b = exr_file.channel('B', pt)

# Create new header with ZIP compression
new_header = OpenEXR.Header(width, height)
new_header['compression'] = Imath.Compression(Imath.Compression.ZIP_COMPRESSION)

# Create output file
exr_out = OpenEXR.OutputFile(output_file, new_header)

# Write the channels
exr_out.writePixels({'R': r, 'G': g, 'B': b})

exr_out.close()
exr_file.close()

print(f"Converted {input_file} to {output_file} with ZIP compression.")

# Python code to extract data from a C header file format, convert packed 
# 0x00BBGGRR format to standard RGB888, and save it as a PNG image.

import re
import numpy as np
from PIL import Image
import os

# --- Configuration ---
WIDTH = 128
HEIGHT = 128
OUTPUT_FILENAME = "output_image_sample.png"
HEADER_FILENAME = "sampledata.h"

def parse_c_header_data(filename: str, width: int, height: int) -> np.ndarray:
    """
    Reads the content of the C header file and parses the SAMPLE_INPUT_0 macro 
    to extract 32-bit hexadecimal pixel values.
    """
    try:
        with open(filename, 'r') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        return np.array([], dtype=np.uint32)
    except Exception as e:
        print(f"Error reading file '{filename}': {e}")
        return np.array([], dtype=np.uint32)

    # Regex to find all '0x' followed by 8 hexadecimal characters
    # This pattern works regardless of line breaks, spaces, or backslashes.
    hex_pattern = r'0x[0-9a-fA-F]{8}'
    hex_strings = re.findall(hex_pattern, content)
    
    # Convert hex strings to integers
    int_values = [int(h, 16) for h in hex_strings]
    
    # Convert the list of integers into a NumPy array of 32-bit unsigned integers
    # We slice to ensure we only process the expected number of pixels (W*H).
    expected_pixels = width * height
    if len(int_values) < expected_pixels:
        print(f"Warning: Found {len(int_values)} pixels, expected {expected_pixels}. Output image may be incomplete.")
        
    return np.array(int_values[:expected_pixels], dtype=np.uint32)


def convert_packed_to_rgb888(packed_data: np.ndarray, width: int, height: int) -> np.ndarray:
    """
    Converts the packed 0x00BBGGRR, zero-centered data to a standard (H, W, 3) RGB array.
    """
    
    # Reverses the zero-centering (XOR 0x00808080)
    denormalized_data = packed_data.astype(np.uint32) ^ 0x00808080
    
    # R channel (bits 0-7)
    R = (denormalized_data >> 0) & 0xFF
    
    # G channel (bits 8-15)
    G = (denormalized_data >> 8) & 0xFF
    
    # B channel (bits 16-23)
    B = (denormalized_data >> 16) & 0xFF
    
    # Stack channels in RGB order and cast to uint8 (0-255 range)
    rgb_channels = np.stack([R, G, B], axis=-1).astype(np.uint8)
    
    # Reshape to (Height, Width, Channels) for image processing
    rgb_image_array = rgb_channels.reshape((height, width, 3))
    
    return rgb_image_array

def save_rgb_to_png(rgb_array: np.ndarray, filename: str):
    """
    Saves the RGB array to a PNG file using the Pillow library.
    """
    try:
        # Create a PIL Image object from the NumPy array
        image = Image.fromarray(rgb_array)
        
        # Save the image as PNG
        image.save(filename)
        
        print(f"Success: Image saved to '{filename}'.")
    except Exception as e:
        print(f"Error saving PNG with Pillow: {e}")
        

def save_image_from_header(header_filename: str, output_filename: str, width: int, height: int):
    """
    Main workflow function to read data from a C header file, process it, 
    and save the result as a PNG image.
    """
    try:
        # Parse the C header content by reading the file
        packed_pixels = parse_c_header_data(header_filename, width, height)
        
        if packed_pixels.size == 0:
            print("Execution halted due to no pixel data found or file error.")
            return

        # Convert the packed data to RGB888
        rgb_output_array = convert_packed_to_rgb888(packed_pixels, width, height)
        
        # Save the RGB array as a PNG file
        save_rgb_to_png(rgb_output_array, output_filename)

    except Exception as e:
        print(f"\nAn unexpected error occurred during image processing: {e}")

# --- Main Execution ---
if __name__ == "__main__":
    save_image_from_header(HEADER_FILENAME, OUTPUT_FILENAME, WIDTH, HEIGHT)


# if __name__ == "__main__":
#     try:
#         # Parse the C header content by reading the file
#         packed_pixels = parse_c_header_data(HEADER_FILENAME, WIDTH, HEIGHT)
        
#         if packed_pixels.size == 0:
#             print("Execution halted due to no pixel data found or file error.")
#         else:
#             # Convert the packed data to RGB888
#             rgb_output_array = convert_packed_to_rgb888(packed_pixels, WIDTH, HEIGHT)
            
#             # Save the RGB array as a PNG file
#             save_rgb_to_png(rgb_output_array, OUTPUT_FILENAME)
            
#     except Exception as e:
#         print(f"\nAn unexpected error occurred during execution: {e}")
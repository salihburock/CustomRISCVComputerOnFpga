import numpy as np
import cv2
import sys

# --- 1. Palette Generation---
R_VOLTS = [0.386, 0.187, 0.077]  # R0, R1, R2
G_VOLTS = [0.386, 0.187, 0.077]  # G0, G1, G2
B_VOLTS = [0.407, 0.197]  # B0, B1


def create_hardware_palette():
    global R_VOLTS, G_VOLTS, B_VOLTS
    V_MAX_R = sum(R_VOLTS)
    V_MAX_G = sum(G_VOLTS)
    V_MAX_B = sum(B_VOLTS)
    palette_bgr = []
    pixel_values = []

    for r_bits_val in range(8):  # 0-7
        bit_r0 = (r_bits_val >> 2) & 1
        bit_r1 = (r_bits_val >> 1) & 1
        bit_r2 = (r_bits_val >> 0) & 1
        v_r = (bit_r0 * R_VOLTS[0]) + (bit_r1 * R_VOLTS[1]) + (bit_r2 * R_VOLTS[2])

        for g_bits_val in range(8):  # 0-7
            bit_g0 = (g_bits_val >> 2) & 1
            bit_g1 = (g_bits_val >> 1) & 1
            bit_g2 = (g_bits_val >> 0) & 1
            v_g = (bit_g0 * G_VOLTS[0]) + (bit_g1 * G_VOLTS[1]) + (bit_g2 * G_VOLTS[2])

            for b_bits_val in range(4):  # 0-3
                bit_b0 = (b_bits_val >> 1) & 1
                bit_b1 = (b_bits_val >> 0) & 1
                v_b = (bit_b0 * B_VOLTS[0]) + (bit_b1 * B_VOLTS[1])

                pixel_data_byte = (bit_r0, bit_r1, bit_r2, bit_g0, bit_g1, bit_g2, bit_b0, bit_b1)
                r_255 = int((v_r / V_MAX_R) * 255)
                g_255 = int((v_g / V_MAX_G) * 255)
                b_255 = int((v_b / V_MAX_B) * 255)

                palette_bgr.append((b_255, g_255, r_255))
                pixel_values.append(pixel_data_byte)

    return palette_bgr, pixel_values


palette, pixel_values = create_hardware_palette()
print(f"Generated {len(palette)} colors.")

# --- 2. Image Loading and Resizing (MODIFIED FOR DOOM VRAM) ---
image = cv2.imread("load_screen.png")
if image is None:
    print("Error: Could not load image")
    sys.exit()

image_original = image.copy()

target_width = 320
target_height = 200
print(f"Resizing image to EXACTLY {target_width}x{target_height} for the VRAM Bootloader...")

# Force the resize to exactly 320x200
image = cv2.resize(image, (target_width, target_height), interpolation=cv2.INTER_AREA)

palette_lab = cv2.cvtColor(np.array([palette], dtype=np.uint8), cv2.COLOR_BGR2LAB)[0]

# --- 3. Dithering Loop (Floyd-Steinberg) ---
image_float = image.astype(np.float32)
output_image = np.zeros_like(image)
data_flattened_hex = []

for y in range(target_height):
    print(f"Processing row {y + 1}/{target_height}...", end='\r')
    for x in range(target_width):
        old_pixel = image_float[y, x]
        old_pixel_clipped = np.clip(old_pixel, 0, 255).astype(np.uint8)

        test_bgr = np.array([[old_pixel_clipped]], dtype=np.uint8)
        test_lab = cv2.cvtColor(test_bgr, cv2.COLOR_BGR2LAB)[0][0].astype(np.int32)

        diffs = palette_lab.astype(np.int32) - test_lab
        dist_sq = np.sum(np.square(diffs), axis=1)
        min_index = np.argmin(dist_sq)

        closest_color = palette[min_index]
        output_image[y, x] = closest_color

        # Convert the tuple of bits into an 8-bit integer, then to a 2-char hex string
        bits = pixel_values[min_index]
        byte_val = 0
        for i, bit in enumerate(bits):
            byte_val |= (bit << (7 - i))
        data_flattened_hex.append(f"{byte_val:02X}")

        # Distribute the error
        new_pixel_float = np.array(closest_color, dtype=np.float32)
        quant_error = old_pixel - new_pixel_float

        if x + 1 < target_width: image_float[y, x + 1] += quant_error * (7 / 16)
        if (x - 1 >= 0) and (y + 1 < target_height): image_float[y + 1, x - 1] += quant_error * (3 / 16)
        if y + 1 < target_height: image_float[y + 1, x] += quant_error * (5 / 16)
        if (x + 1 < target_width) and (y + 1 < target_height): image_float[y + 1, x + 1] += quant_error * (1 / 16)

print("\nDithering complete.")

# --- 4. Display Preview ---
resized_image = cv2.resize(output_image, (800, 600), interpolation=cv2.INTER_NEAREST)
cv2.imshow("Hardware Output Preview", resized_image)
cv2.waitKey(1000)  # Show preview for 1 second, then auto-close
cv2.destroyAllWindows()

# --- 5. Write to .mi File (MODIFIED FOR 32-BIT WORDS) ---
print("Packing pixels into 32-bit VRAM blocks...")

with open("boot_splash.mi", "w") as f:
    # Notice Data_width is now 32!
    f.write("""#File_format=Hex
#Address_depth=16384
#Data_width=32\n""")
    # The VGA Controller reads: Word 0 = [31:24], Word 1 = [23:16], Word 2 = [15:8], Word 3 = [7:0]

    word_count = 0
    for i in range(0, len(data_flattened_hex), 4):
        p0 = data_flattened_hex[i]
        p1 = data_flattened_hex[i + 1]
        p2 = data_flattened_hex[i + 2]
        p3 = data_flattened_hex[i + 3]

        # Combine the 4 hex strings into one 8-character string (32 bits)
        packed_word = f"{p0}{p1}{p2}{p3}"
        f.write(packed_word + "\n")
        word_count += 1

    # Gowin BSRAM requires exactly 16384 lines, pad the rest with black (00000000)
    while word_count < 16384:
        f.write("00000000\n")
        word_count += 1

print("boot_splash.mi file generated successfully! Ready to initialize BSRAM.")
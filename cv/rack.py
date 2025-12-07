import cv2
import numpy as np
import yaml

# -----------------------------
# Load map metadata (YAML)
# -----------------------------

yaml_path = "/home/aech/ros2_ws/src/ros2-slam-auto-navigation/maps/mymap.yaml"

with open(yaml_path, 'r') as f:
    metadata = yaml.safe_load(f)

resolution = metadata["resolution"]
origin_x, origin_y, origin_yaw = metadata["origin"]

print("Map Resolution:", resolution)
print("Map Origin:", origin_x, origin_y)

# -----------------------------
# Adjustable parameters
# -----------------------------

min_width_m = 0.10    # minimum 10 cm
max_width_m = 0.90    # maximum 90 cm

min_w_px = int(min_width_m / resolution)
max_w_px = int(max_width_m / resolution)

print("Accepted rack thickness (pixels):", min_w_px, "to", max_w_px)

# -----------------------------
# Load map image
# -----------------------------

pgm_path = "/home/aech/ros2_ws/src/ros2-slam-auto-navigation/maps/mymap.pgm"
img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
height, width = img.shape

# -----------------------------
# Pixel → Map coordinate conversion
# -----------------------------

def pixel_to_map(px, py):
    mx = origin_x + px * resolution
    my = origin_y + (height - py) * resolution
    return mx, my

# -----------------------------
# Threshold map to isolate obstacles
# -----------------------------

_, thresh = cv2.threshold(img, 150, 255, cv2.THRESH_BINARY_INV)

# -----------------------------
# Dilate to thicken SLAM lines
# -----------------------------

dilate_kernel = np.ones((7, 7), np.uint8)
dilated = cv2.dilate(thresh, dilate_kernel, iterations=1)

# Clean noise
clean_kernel = np.ones((5, 5), np.uint8)
clean = cv2.morphologyEx(dilated, cv2.MORPH_OPEN, clean_kernel)

# -----------------------------
# Orientation via PCA
# -----------------------------

def compute_orientation_pca(cnt):
    pts = cnt.reshape(-1, 2).astype(np.float32)

    mean = np.empty((0))
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(pts, mean)

    vx, vy = eigenvectors[0]   # pixel coordinates (vx, vy)

    # Convert to map frame (invert Y axis)
    vx_map = vx
    vy_map = -vy

    # Compute angle in map frame
    angle_rad = np.arctan2(vy_map, vx_map)
    angle_deg = np.degrees(angle_rad)

    return angle_deg, (vx_map, vy_map)


# -----------------------------
# Contour detection
# -----------------------------

contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

rack_contours = []
rack_positions = []

for cnt in contours:
    x, y, w, h = cv2.boundingRect(cnt)
    thickness = min(w, h)

    if min_w_px <= thickness <= max_w_px:
        rack_contours.append((x, y, w, h))

        cx = x + w / 2
        cy = y + h / 2

        cx_map, cy_map = pixel_to_map(cx, cy)

        # Orientation
        angle_deg, direction_vec = compute_orientation_pca(cnt)

        # Orientation classification
        if abs(angle_deg) < 20:
            orient_type = "horizontal"
        elif abs(angle_deg) > 70:
            orient_type = "vertical"
        else:
            orient_type = "tilted"

        rack_positions.append({
            "bbox_px": (x, y, w, h),
            "center_px": (cx, cy),
            "center_map": (cx_map, cy_map),
            "orientation_deg": angle_deg,
            "orientation_type": orient_type,
            "direction_vector": direction_vec
        })

# -----------------------------
# Print Results
# -----------------------------

print("\n=====================================")
print("        RACKS DETECTED IN MAP        ")
print("=====================================\n")

for i, rack in enumerate(rack_positions):
    print(f"Rack {i + 1}:")
    print("  Bounding Box (px):", rack["bbox_px"])
    print("  Center (px):      ", rack["center_px"])
    print("  Center (meters):  ", rack["center_map"])
    print("  Orientation (deg):", round(rack["orientation_deg"], 2))
    print("  Orientation type: ", rack["orientation_type"])
    print("  Direction vector :", np.round(rack["direction_vector"], 3))
    print("")

# -----------------------------
# Draw detected racks
# -----------------------------

output = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

for r in rack_positions:
    x, y, w, h = r["bbox_px"]
    cv2.rectangle(output, (x, y), (x + w, y + h), (0,255,0), 2)
    cv2.putText(output, f"{r['orientation_type']} {round(r['orientation_deg'],1)}°",
                (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)

# -----------------------------
# Display large images
# -----------------------------

scale = 4.0
cv2.imshow("Original Map", cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST))
cv2.imshow("Dilated Map", cv2.resize(dilated, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST))
cv2.imshow("Detected Racks", cv2.resize(output, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST))

cv2.waitKey(0)

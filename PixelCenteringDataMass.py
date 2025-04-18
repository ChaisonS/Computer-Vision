import cv2
import numpy as np

cap = cv2.VideoCapture(0)

lower_blue = np.array([95, 75, 50])
upper_blue = np.array([145, 255, 255])
kernel = np.ones((7, 7), np.uint8)

radrat = 0.4

def is_majority_near_center(frame, mask, center_radius_ratio = radrat):
    h, w = frame.shape[:2]
    center_x, center_y = w // 2, h // 2
    max_distance = np.sqrt(center_x**2 + center_y**2)
    center_radius = center_radius_ratio * max_distance

    y_indices, x_indices = np.where(mask != 0)
    total_pixels = len(x_indices)
    if total_pixels == 0:
        return False

    distances = np.sqrt((x_indices - center_x)**2 + (y_indices - center_y)**2)
    near_center_count = np.sum(distances <= center_radius)
    return near_center_count > (total_pixels / 2)

def get_dominant_quadrant(frame, mask):
    h, w = frame.shape[:2]
    center_x, center_y = w // 2, h // 2

    y_indices, x_indices = np.where(mask != 0)

    quadrant_counts = [0, 0, 0, 0]  # Q1, Q2, Q3, Q4

    for x, y in zip(x_indices, y_indices):
        if x < center_x and y < center_y:
            quadrant_counts[0] += 1  # Q1
        elif x >= center_x and y < center_y:
            quadrant_counts[1] += 1  # Q2
        elif x < center_x and y >= center_y:
            quadrant_counts[2] += 1  # Q3
        else:
            quadrant_counts[3] += 1  # Q4

    max_index = np.argmax(quadrant_counts)
    return f"Q{max_index + 1}"

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    non_blue_mask = cv2.bitwise_not(blue_mask)

    white_frame = np.ones_like(frame) * 255
    result = cv2.bitwise_and(frame, frame, mask=non_blue_mask)
    white_areas = cv2.bitwise_and(white_frame, white_frame, mask=blue_mask)
    final_result = cv2.add(result, white_areas)

    contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) > 500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(final_result, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(final_result, "Filtered out!", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

    centered = is_majority_near_center(frame, non_blue_mask)
    label = "Centered" if centered else "Not Centered"
    color = (0, 255, 0) if centered else (0, 0, 255)
    cv2.putText(final_result, label, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)

    # --- NEW: Add quadrant info ---
    dominant_quadrant = get_dominant_quadrant(frame, non_blue_mask)
    cv2.putText(final_result, f"Most mass: {dominant_quadrant}", (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 200, 0), 2)

    # Optional: draw center area
    h, w = frame.shape[:2]
    center_x, center_y = w // 2, h // 2
    radius = int(radrat * np.sqrt(center_x**2 + center_y**2))
    cv2.circle(final_result, (center_x, center_y), radius, (255, 0, 255), 2)

    cv2.imshow("Result", final_result)
    with open("quadrant.txt", "w") as f:
        f.write(dominant_quadrant)
    with open("centered_status.txt", "w") as f:
        f.write(str(centered))  # Write True/False


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# Dominant Quadrant = the quadrant that has the most shit in it,
# Q1: Forward and to the Right
# Q2: Forward and to the Left
# Q3: Backward and to the Left
# Q4: Backward and to the Right
# Based on the Dominant Quadrant, the simulation should output some nice hsit

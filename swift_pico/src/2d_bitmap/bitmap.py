import cv2
import argparse
import numpy as np

# parser = argparse.ArgumentParser()
# parser.add_argument("--image", type=str)

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

PADDING_BORDER_SIZE = 10


def get_center_coordinates_from_rect(list_of_coordinates):
    return (np.mean(list_of_coordinates[:, 0]), np.mean(list_of_coordinates[:, 1]))


def get_array_transform_values(list_of_coordinates, center_coordinates):
    center_x, center_y = center_coordinates

    rad_angles = [np.atan2(y - center_y, x - center_x) for x, y in list_of_coordinates]
    deg_angles = [(np.degrees(angle) + 360) % 360 for angle in rad_angles]

    # Sort the points based on the angles (in clockwise order)
    indices = np.argsort(deg_angles)

    sorted_coordinates = list_of_coordinates[indices]

    array_shift_value = np.argmin(sorted_coordinates[:, 0] + sorted_coordinates[:, 1])

    return (array_shift_value, indices)


def process_aruco(image):

    # Load the ArUCo dictionary, grab the ArUCo parameters, and
    # attempt to detect the markers for the current dictionary
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    arucoParams = cv2.aruco.DetectorParameters()
    (aruco_corners, aruco_ids, rejected) = cv2.aruco.detectMarkers(
        image, arucoDict, parameters=arucoParams
    )

    # Reordering the aruco markers
    aruco_marker_centers = []
    for i in range(len(aruco_corners)):
        center_x, center_y = get_center_coordinates_from_rect(aruco_corners[i][0])
        aruco_marker_centers.append((center_x, center_y))

    (h, w) = image.shape[:2]
    image_center_x = w // 2
    image_center_y = h // 2

    shift_value, rotational_indexes = get_array_transform_values(
        np.array(aruco_marker_centers), (image_center_x, image_center_y)
    )

    reordered_markers = tuple(aruco_corners[i] for i in rotational_indexes)
    reordered_markers = np.roll(reordered_markers, -shift_value, axis=0)

    # Reordering the markers coordinates
    sorted_marker_coordinates = []
    for marker_coordinates in reordered_markers:
        center_x, center_y = get_center_coordinates_from_rect(marker_coordinates[0])
        shift_value, rotational_indexes = get_array_transform_values(
            marker_coordinates[0], (center_x, center_y)
        )
        reordered_corners = tuple(marker_coordinates[0][i] for i in rotational_indexes)
        reordered_corners = np.roll(reordered_corners, -shift_value, axis=0)
        sorted_marker_coordinates.append(reordered_corners)

    return (sorted_marker_coordinates, aruco_ids)


def apply_warp_perspective(image, marker_coordinates, resize_size=1000):
    top_left_marker = marker_coordinates[0]
    top_right_marker = marker_coordinates[1]
    bottom_right_marker = marker_coordinates[2]
    bottom_left_marker = marker_coordinates[3]

    pt_A = top_left_marker[2]
    pt_B = top_right_marker[3]
    pt_C = bottom_right_marker[0]
    pt_D = bottom_left_marker[1]

    width = int(np.linalg.norm(pt_B - pt_A))
    height = int(np.linalg.norm(pt_D - pt_A))

    input_pts = np.float32([pt_A, pt_B, pt_C, pt_D])
    output_pts = np.float32([[0, 0], [width, 0], [width, height], [0, height]])

    # Compute the perspective transform M
    M = cv2.getPerspectiveTransform(input_pts, output_pts)

    wp_image = cv2.warpPerspective(image, M, (width, height), flags=cv2.INTER_LINEAR)
    wp_image = cv2.resize(wp_image, (resize_size, resize_size))

    return wp_image


def treshold_and_find_contours(image, min_contour_area):
    padded_image = cv2.copyMakeBorder(
        cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
        PADDING_BORDER_SIZE,
        PADDING_BORDER_SIZE,
        PADDING_BORDER_SIZE,
        PADDING_BORDER_SIZE,
        cv2.BORDER_CONSTANT,
        value=(255, 255, 255),
    )
    _, thresh = cv2.threshold(padded_image, 127, 255, cv2.THRESH_BINARY, image)

    # Find contours and adjusting them because of the added border
    all_contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    valid_contours = [
        cnt - [PADDING_BORDER_SIZE, PADDING_BORDER_SIZE] for cnt in all_contours
    ]

    # Filter based on area
    max_area = max(cv2.contourArea(cnt) for cnt in valid_contours)
    return [
        cnt
        for cnt in valid_contours
        if min_contour_area < cv2.contourArea(cnt) < max_area
    ]


def scale_contours(contours, inflation_factor):
    
    for contour in contours:
        
        # Create a convex hull
        hull = cv2.convexHull(contour)
        
        # Calculate the center of the contour
        M = cv2.moments(hull)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # Inflate the contour points by moving them outward from the center
        scaled_hull = []
        for point in hull:
            x, y = point[0]

            # Calculate the vector from the center to the point
            vector_x = x - cx
            vector_y = y - cy

            new_x = int(cx + inflation_factor * vector_x)
            new_y = int(cy + inflation_factor * vector_y)
            scaled_hull.append([[new_x, new_y]])

        # Convert to NumPy array format
    return np.array(scaled_hull, dtype=np.int32)


# # Extract the argument
# args = parser.parse_args()

# Loading the image from arguments
bgr_image = cv2.imread(
    "ws/dtu_ws/src/wd_task_2b_bitmap/test_bitmap.png", cv2.IMREAD_COLOR
)
rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)

# # Processing Aruco markers
# marker_coordinates, aruco_ids = process_aruco(image)

# # Applying warp-perspective
# wp_image = apply_warp_perspective(marker_coordinates, 1000)

# Thresholding the image and find contours
# binary_image = wp_image.copy()
contours = treshold_and_find_contours(bgr_image, 50)
scaled_contours = scale_contours(contours, 1.1)

cv2.drawContours(rgb_image, [scaled_contours], -1, (0, 255, 0), 2)

# for contour in contours:
#     # Get the bounding rectangle for each contour

#     x, y, w, h = cv2.boundingRect(contour)

#     # Draw the rectangle around the contour
#     cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0, 0, 255), 2)

cv2.imwrite("ws/dtu_ws/src/wd_task_2b_bitmap/output.jpg", rgb_image)


# lines_to_write = []
# lines_to_write.append("Aruco ID: {}\n".format(aruco_ids.flatten().tolist()))
# lines_to_write.append("Obstacles: {}\n".format(len(obstacle_areas)))
# lines_to_write.append("Area: {}".format(sum(obstacle_areas)))


# f = open("obstacles.txt", "w")
# f.writelines(lines_to_write)
# f.close()

# cv2.imwrite("output.jpg", binary_image)

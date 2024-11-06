import cv2
import numpy as np
import os

# Just for local dev with local pictures
FILE_DIR_PATH = os.path.dirname(__file__)

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

CLEAN_ARUCO_PADDING_SIZE = 5


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


# Needs OpenCV 4.10.0 and NumPy 2.1.1
def process_aruco(image):

    # Load the ArUCo dictionary, grab the ArUCo parameters, and
    # attempt to detect the markers for the current dictionary
    arucoDict = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_100
    )  # TODO: make it more robust
    arucoParams = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

    (aruco_corners, aruco_ids, _) = detector.detectMarkers(image)

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

    pt_A = top_left_marker[0]
    pt_B = top_right_marker[1]
    pt_C = bottom_right_marker[2]
    pt_D = bottom_left_marker[3]

    width = int(np.linalg.norm(pt_B - pt_A))
    height = int(np.linalg.norm(pt_D - pt_A))

    input_pts = np.float32([pt_A, pt_B, pt_C, pt_D])
    output_pts = np.float32([[0, 0], [width, 0], [width, height], [0, height]])

    # Compute the perspective transform M
    M = cv2.getPerspectiveTransform(input_pts, output_pts)

    wp_image = cv2.warpPerspective(image, M, (width, height), flags=cv2.INTER_LINEAR)

    wp_image = crop_image_towards_center(wp_image, 3)
    wp_image = cv2.resize(wp_image, (resize_size, resize_size))

    return wp_image


def clean_image_from_aruco(image, aruco_marker_coordinates):
    cleaned_image = image.copy()
    for marker in aruco_marker_coordinates:
        pt_A = marker[0].astype(int) - CLEAN_ARUCO_PADDING_SIZE  # top-left
        pt_C = marker[2].astype(int) + CLEAN_ARUCO_PADDING_SIZE  # bottom-right
        cv2.rectangle(cleaned_image, pt_A, pt_C, (255, 255, 255), thickness=-1)
    return cleaned_image


def crop_image_towards_center(image, crop_size):
    h, w = image.shape[:2]
    x_start, y_start = crop_size, crop_size
    x_end, y_end = w - crop_size, h - crop_size
    return image[y_start:y_end, x_start:x_end].copy()


def clean_contours(contours):
    # Find the biggest contour area which is the image itself
    max_area = max([cv2.contourArea(cnt) for cnt in contours])
    return [cnt for cnt in contours if cv2.contourArea(cnt) < max_area]


# def treshold_and_find_contours(image):
#     _, tresh = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY, image)

#     # Find all contours in the padded image
#     contours, _ = cv2.findContours(tresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#     contours = clean_contours(contours)

#     return (tresh, contours)


def scale_contours(image):
    _, tresh = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)
    distance = cv2.distanceTransform(tresh, cv2.DIST_L2, 5)
    _, offset_thresh = cv2.threshold(distance, 0.05 * distance.max(), 255, 0)
    offset_thresh = np.uint8(offset_thresh)
    offset_contours, _ = cv2.findContours(
        offset_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )

    offset_contours = clean_contours(offset_contours)

    cv2.drawContours(offset_thresh, offset_contours, -1, (0, 0, 0), -1)
    return offset_thresh


def create_2d_bitmap(image) -> bool:
    gray_image = cv2.imread(image, cv2.IMREAD_GRAYSCALE)

    # Processing Aruco markers
    marker_coordinates, _ = process_aruco(gray_image)

    cleaned_image = clean_image_from_aruco(gray_image, marker_coordinates)

    # Applying warp-perspective
    wp_image = apply_warp_perspective(cleaned_image, marker_coordinates, 1000)

    # tresh, contours = treshold_and_find_contours(wp_image)

    # Binarise the image, scale and find the contours
    scaled_contours_image = scale_contours(wp_image)

    boolean_obsticales = scaled_contours_image == 0  # TODO: return?

    cv2.imwrite(FILE_DIR_PATH + "/2D_bit_map.png", scaled_contours_image)

    return True

gray_image = cv2.imread(FILE_DIR_PATH + "/2D_bit_map.png", cv2.IMREAD_GRAYSCALE)
create_2d_bitmap(gray_image)
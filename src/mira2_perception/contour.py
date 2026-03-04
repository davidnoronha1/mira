import matplotlib.pyplot as plt
import cv2
import numpy as np
from bp_vision.rect import isRectangleFromApproximation
from bp_vision.rect import correctRectangleContour



# -------------------------------------------------
# SAFE TYPE (works on old OpenCV)
# -------------------------------------------------
try:
    Mat_T = cv2.typing.MatLike
except AttributeError:
    Mat_T = np.ndarray


# -------------------------------------------------
# Largest contours
# -------------------------------------------------
def _getLargestContours(contours, top_n=3):
    if not contours:
        return []

    sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
    largest = sorted_contours[:top_n]

    print(f"Selected top {len(largest)} largest contours out of {len(contours)}")
    return largest


# -------------------------------------------------
# Find likely contours
# -------------------------------------------------
def _findLikelyContours(gray, frame, params, quality, visualize=True):

    block_size = 23
    C = 7 if quality > 200 else 3

    thresh = cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_MEAN_C,
        cv2.THRESH_BINARY,
        block_size,
        C,
    )

    kernel = np.ones((3, 3), np.uint8)
    opened = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
    morphed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel, iterations=1)
    blurred = cv2.GaussianBlur(morphed, (5, 5), 0)

    edges_hough = cv2.Canny(blurred, 50, 150, apertureSize=3)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    dilated = cv2.dilate(edges_hough, kernel, iterations=1)

    contours_lines, _ = cv2.findContours(
        dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    h, w = edges_hough.shape[:2]
    min_area = max(20, int(0.0005 * h * w))
    filtered = [c for c in contours_lines if cv2.contourArea(c) >= min_area]

    print(
        f"Found {len(contours_lines)} raw contours, {len(filtered)} after filtering"
    )

    return filtered


# -------------------------------------------------
# Find connecting contours
# -------------------------------------------------
def _findConnectingContours(contours, frame=None, distance_factor=0.2, viz=False):

    if not contours:
        raise ValueError("No contours provided")

    bbox_areas = [cv2.boundingRect(c)[2] * cv2.boundingRect(c)[3] for c in contours]
    largest_idx = int(np.argmax(bbox_areas))
    largest_contour = contours[largest_idx]

    x, y, w, h = cv2.boundingRect(largest_contour)
    largest_bbox = (x, y, x + w, y + h)

    connecting_contours = []

    for i, cnt in enumerate(contours):
        if i == largest_idx:
            continue

        x2, y2, w2, h2 = cv2.boundingRect(cnt)
        bbox2 = (x2, y2, x2 + w2, y2 + h2)

        overlap = not (
            bbox2[2] < largest_bbox[0]
            or bbox2[0] > largest_bbox[2]
            or bbox2[3] < largest_bbox[1]
            or bbox2[1] > largest_bbox[3]
        )

        if overlap:
            connecting_contours.append(cnt)

    return largest_contour, connecting_contours


# -------------------------------------------------
# Find missing corner
# -------------------------------------------------
def _findConnectingContourWithFourthPoint(largest_contour, connecting_contours):

    if largest_contour is None or len(largest_contour) < 3:
        return None, None, None
    if not connecting_contours:
        return None, None, None

    epsilon = 0.02 * cv2.arcLength(largest_contour, True)
    approx = cv2.approxPolyDP(largest_contour, epsilon, True)

    if len(approx) != 3:
        hull = cv2.convexHull(largest_contour)
        epsilon = 0.02 * cv2.arcLength(hull, True)
        approx = cv2.approxPolyDP(hull, epsilon, True)

    corners = approx.reshape(-1, 2).astype(np.float32)

    if len(corners) < 3:
        return None, None, None

    corners = corners[:3]

    p1, p2, p3 = corners
    predicted_point = p1 + p3 - p2
    predicted_point = tuple(map(int, predicted_point))

    min_dist = float("inf")
    best_contour = None

    for cnt in connecting_contours:
        dist = abs(cv2.pointPolygonTest(cnt, predicted_point, True))
        if dist < min_dist:
            min_dist = dist
            best_contour = cnt

    return predicted_point, best_contour, min_dist


# -------------------------------------------------
# Final contours
# -------------------------------------------------
params = {
    "adaptiveThreshConstant": 7,    #7
    "adaptiveThreshWinSizeMax": 23,   #23
    "adaptiveThreshWinSizeMin": 3,
    "adaptiveThreshWinSizeStep": 10,
}


def _getFinalContours(gh, iframe, quality, viz=False):

    possibleContours = _getLargestContours(
        _findLikelyContours(gh, iframe, params, quality, visualize=viz),
        top_n=100,
    )

    largest, connecting = _findConnectingContours(
        possibleContours, frame=iframe, distance_factor=0.1, viz=viz
    )

    is_rect, _ = isRectangleFromApproximation(largest, iframe, viz=viz)

    if is_rect:
        return [largest]

    print("Largest contour not full gate — assuming split gate")

    pt, next_contour, _ = _findConnectingContourWithFourthPoint(
        largest, connecting
    )

    return [largest, next_contour]

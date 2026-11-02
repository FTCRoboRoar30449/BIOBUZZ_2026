# BIOBUZZ - count balls visible inside the raised CELL
#
# Limelight SnapScript, pipeline 2. OPTIONAL and EXPERIMENTAL.
#
# WHY THIS IS HARD, AND WHY IT IS TELEMETRY ONLY
#   You cannot see into the CELL's mouth from anywhere you would shoot from.
#   The mouth's outward normal sits 30.2 deg above horizontal; from a camera
#   at 17.4 in looking up at an opening centred at 59.55 in, you are 77-85 deg
#   off that normal across the whole 20-39 in shooting range. At that angle
#   you see the rim, not the contents.
#
#   The only route is through the CELL's clear side panels, which are near
#   vertical and therefore viewed at a reasonable 25-40 deg. A 2.8 in ball at
#   30 in subtends roughly 62 px, so the balls are big enough to see. What is
#   NOT reliable is counting them: they stack and occlude each other, the
#   frame members cut across the view, and you are looking through a
#   reflective panel at a backlit interior.
#
#   Expect this to work in a quiet gym and degrade under arena lighting.
#   MechController treats the result as telemetry and never gates firing on
#   it -- the hive FSM shoots everything regardless, because your alliance
#   partner is feeding the same CELL and any count you hold is already stale.
#
# OUTPUT (llpython)
#   [0] yellow blobs  (POLLEN)
#   [1] red blobs     (NECTAR red)
#   [2] blue blobs    (NECTAR blue)
#   [3] 1 if the frame looked usable, 0 if it was rejected
#   [4] total contour count before filtering, for debugging thresholds
#
# TUNING
#   Hue here is OpenCV's 0-179, NOT 0-360. Halve anything you measured with
#   a colour sensor. Start by widening V, which is what arena lighting moves
#   most, and keep MIN_AREA high enough to reject the frame's own highlights.

import cv2
import numpy as np

# --- colour gates, OpenCV HSV: H 0-179, S 0-255, V 0-255 -------------------
YELLOW_LO = np.array([20, 110, 110]);  YELLOW_HI = np.array([35, 255, 255])
RED1_LO   = np.array([0, 120, 90]);    RED1_HI   = np.array([8, 255, 255])
RED2_LO   = np.array([170, 120, 90]);  RED2_HI   = np.array([179, 255, 255])
BLUE_LO   = np.array([100, 120, 80]);  BLUE_HI   = np.array([125, 255, 255])

MIN_AREA = 250          # px, rejects speckle and frame highlights
MAX_AREA = 40000        # px, rejects a whole-frame colour cast
MIN_CIRCULARITY = 0.50  # balls are round; frame members and glare are not

KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def _count(mask):
    """Blobs that are round enough and big enough to be a ball."""
    # Open kills speckle; close fills the holes in a perforated ball, which
    # otherwise fragments into several small contours and over-counts.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    kept = 0
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA or area > MAX_AREA:
            continue
        per = cv2.arcLength(c, True)
        if per <= 0:
            continue
        if 4.0 * np.pi * area / (per * per) < MIN_CIRCULARITY:
            continue
        kept += 1
    return kept, len(contours)


def runPipeline(image, llrobot):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    yellow, ty = _count(cv2.inRange(hsv, YELLOW_LO, YELLOW_HI))
    red, tr = _count(cv2.bitwise_or(cv2.inRange(hsv, RED1_LO, RED1_HI),
                                    cv2.inRange(hsv, RED2_LO, RED2_HI)))
    blue, tb = _count(cv2.inRange(hsv, BLUE_LO, BLUE_HI))

    # A frame where a third of the pixels pass one gate is a lighting failure,
    # not a CELL full of balls. Report it as unusable rather than as a count.
    total_px = image.shape[0] * image.shape[1]
    saturated = (yellow + red + blue) > 12
    usable = 0 if saturated else 1

    llpython = [yellow, red, blue, usable, ty + tr + tb]

    # No contour to steer by -- this pipeline reports counts, nothing aims at
    # it. Returning an empty contour keeps tx/ty from being misread as a
    # target by anything downstream.
    return np.array([[]]), image, llpython

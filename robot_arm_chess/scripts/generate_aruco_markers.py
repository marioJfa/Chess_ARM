#!/usr/bin/env python3
"""
generate_aruco_markers.py
Generates a printable PNG of the 4 ArUco board-reference markers.

Usage:
    python3 generate_aruco_markers.py [output.png]

Print the output image, cut out the 4 markers, and tape them face-up at the
four corners of the chess board (outside the playing area):

    ID 0 → a1 corner  (bottom-left,  white's near-left)
    ID 1 → h1 corner  (bottom-right, white's near-right)
    ID 2 → a8 corner  (top-left,     black's near-left)
    ID 3 → h8 corner  (top-right,    black's near-right)

The inner corner of each marker (the corner touching the board playing area)
is used as the exact reference point — align it with the board corner.
"""

import sys
import cv2
import numpy as np

MARKER_SIZE_PX  = 200   # size of each marker in the output image
MARGIN_PX       = 30    # white border around each marker
LABEL_HEIGHT_PX = 40    # space below each marker for the label

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  # OpenCV 4.6

LABELS = {
    0: 'ID 0  — a1 corner (bottom-left)',
    1: 'ID 1  — h1 corner (bottom-right)',
    2: 'ID 2  — a8 corner (top-left)',
    3: 'ID 3  — h8 corner (top-right)',
}

# Inner corner direction label per marker (which corner touches the board)
INNER_LABEL = {
    0: 'inner -> top-right',
    1: 'inner -> top-left',
    2: 'inner -> bottom-right',
    3: 'inner -> bottom-left',
}


def make_sheet(output_path: str):
    cell_w = MARKER_SIZE_PX + 2 * MARGIN_PX
    cell_h = MARKER_SIZE_PX + 2 * MARGIN_PX + LABEL_HEIGHT_PX

    # 2 columns × 2 rows
    sheet_w = cell_w * 2
    sheet_h = cell_h * 2
    sheet   = np.full((sheet_h, sheet_w, 3), 255, dtype=np.uint8)

    positions = [(0, 0), (0, 1), (1, 0), (1, 1)]   # (row, col) for IDs 0-3

    for marker_id, (row, col) in enumerate(positions):
        # Generate marker image
        marker_img = cv2.aruco.generateImageMarker(ARUCO_DICT, marker_id, MARKER_SIZE_PX)
        marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)

        # Destination in sheet
        y0 = row * cell_h + MARGIN_PX
        x0 = col * cell_w + MARGIN_PX
        sheet[y0:y0 + MARKER_SIZE_PX, x0:x0 + MARKER_SIZE_PX] = marker_bgr

        # Main label
        label_y = y0 + MARKER_SIZE_PX + 24
        cv2.putText(sheet, LABELS[marker_id],
                    (x0, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1, cv2.LINE_AA)

        # Inner-corner hint
        hint_y = label_y + 16
        cv2.putText(sheet, INNER_LABEL[marker_id],
                    (x0, hint_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (80, 80, 80), 1, cv2.LINE_AA)

        # Highlight the inner corner with a red circle
        inner_idx = {0: 1, 1: 0, 2: 2, 3: 3}[marker_id]
        corners_norm = np.array([
            [0, 0], [1, 0], [1, 1], [0, 1]   # TL TR BR BL in normalised [0,1]
        ], dtype=np.float32)
        cx = int(x0 + corners_norm[inner_idx][0] * MARKER_SIZE_PX)
        cy = int(y0 + corners_norm[inner_idx][1] * MARKER_SIZE_PX)
        cv2.circle(sheet, (cx, cy), 10, (0, 0, 220), 2)

    # Title
    cv2.putText(sheet, 'Chess Board ArUco Reference Markers  (DICT_4X4_50)',
                (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(sheet, 'Red circle = inner corner — align with board corner',
                (10, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 180), 1, cv2.LINE_AA)

    cv2.imwrite(output_path, sheet)
    print(f'Saved: {output_path}  ({sheet_w}x{sheet_h} px)')
    print('Print at 100%, cut out markers, tape at board corners.')


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else 'aruco_markers.png'
    make_sheet(out)

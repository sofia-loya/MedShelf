SUPPLY_TO_COLOR = {
    "gauze": "red",
    "gloves": "green",
    "syringes": "blue",
    "masks": "yellow",
    "tape": "orange",
    "wipes": "purple",
    "dressings": "pink",
    "saline": "cyan",
    "specimen_cups": "black",
}

SLOT_NAMES = (
    "top_left",
    "top_center",
    "top_right",
    "middle_left",
    "middle_center",
    "middle_right",
    "bottom_left",
    "bottom_center",
    "bottom_right",
)

# OpenCV uses H in [0, 179], S in [0, 255], and V in [0, 255]
HSV_RANGES = {
    "red": [
        ((0, 100, 80), (10, 255, 255)),
        ((170, 100, 80), (179, 255, 255)),
    ],
    "green": [
        ((40, 80, 60), (85, 255, 255)),
    ],
    "blue": [
        ((95, 80, 60), (130, 255, 255)),
    ],
    "yellow": [
        ((24, 100, 60), (38, 255, 255)),
    ],
    "orange": [
        ((10, 100, 60), (23, 255, 255)),
    ],
    "purple": [
        ((130, 70, 50), (160, 255, 255)),
    ],
    "pink": [
        ((160, 60, 80), (175, 255, 255)),
    ],
    "cyan": [
        ((80, 80, 60), (100, 255, 255)),
    ],
    "black": [
        ((0, 0, 0), (179, 255, 70)),
    ],
}

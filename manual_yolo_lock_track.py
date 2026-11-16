from pathlib import Path
import gc
import math
import time

import cv2


# ============================================================
# PERFORMANCE
# ============================================================

cv2.setUseOptimized(True)


# ============================================================
# PROJECT / MODEL
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parent

MODEL_PATH = (
    PROJECT_ROOT
    / "yolo26n_best.pt"
)


# ============================================================
# CAMERA
# ============================================================

# Your working camera test uses camera 1.
CAMERA_INDEX = 1

WINDOW_NAME = "Drone Target Lock"


# IMPORTANT:
# Do NOT force 1280x720.
#
# Your camera already gives around 30-40 FPS at its native
# 640x480 mode, so we let the camera use that mode.


# ============================================================
# YOLO26N
# ============================================================

# 640 is enough for the manually selected target crop
# and is considerably faster than 960.
YOLO_IMGSZ = 640

YOLO_CONF = 0.10

YOLO_IOU = 0.60

DRONE_CLASS_NAME = "drone"


# ============================================================
# MANUAL TARGET SELECTION
# ============================================================

MIN_SELECTION_SIZE = 20

SELECTION_PADDING = 0.20


# ============================================================
# TRACKING PERFORMANCE
# ============================================================

# Camera:
#     approximately 640 x 480
#
# Tracker:
#     approximately 320 x 240
#
# Display:
#     original 640 x 480
#
TRACKING_SCALE = 0.50


# Do one expensive CSRT update every N video frames.
#
# 1 = maximum tracking update rate, lower display FPS
# 2 = recommended
# 3 = even faster display, slightly more tracking lag
TRACKER_UPDATE_EVERY_N_FRAMES = 2


# ============================================================
# TARGET SIZE / DISTANCE TREND
# ============================================================

# At target lock:
#
# scale = 1.00
#
# If bounding box grows:
#
# scale > 1.00
#
# If bounding box shrinks:
#
# scale < 1.00

APPROACH_THRESHOLD = 1.08

AWAY_THRESHOLD = 0.92


# Prevent tracker-box jitter from changing the state rapidly.
SIZE_SMOOTHING = 0.88


# ============================================================
# CENTER TARGET WINDOW
# ============================================================

TARGET_WINDOW_WIDTH = 180

TARGET_WINDOW_HEIGHT = 140

TARGET_CORNER_LENGTH = 35

CENTER_CIRCLE_RADIUS = 10

CENTER_CROSS_LENGTH = 24


# ============================================================
# GLOBAL VARIABLES
# ============================================================

latest_clean_frame = None


# ------------------------------------------------------------
# Mouse
# ------------------------------------------------------------

drawing = False

start_x = 0
start_y = 0

mouse_x = 0
mouse_y = 0


pending_selection = None

pending_selection_frame = None


# ------------------------------------------------------------
# Tracking
# ------------------------------------------------------------

tracker = None

tracking = False


# Last good tracker box in full-resolution coordinates
last_full_bbox = None


# Tracker frame counter
tracker_frame_counter = 0


# ------------------------------------------------------------
# Locked target
# ------------------------------------------------------------

reference_image = None

locked_confidence = 0.0


# Bounding-box size at lock time
reference_box_width = 0

reference_box_height = 0

reference_box_area = 0.0


# Current smoothed size relative to original target
smoothed_target_scale = 1.0


# ------------------------------------------------------------
# FPS
# ------------------------------------------------------------

display_fps = 0.0

fps_frame_counter = 0

fps_measurement_start = time.perf_counter()


# ============================================================
# CREATE FAST CSRT TRACKER
# ============================================================

def create_tracker():

    # Try the configurable CSRT API first.

    if (
        hasattr(cv2, "TrackerCSRT_Params")
        and
        hasattr(cv2, "TrackerCSRT_create")
    ):

        try:

            params = cv2.TrackerCSRT_Params()


            # ------------------------------------------------
            # Disable segmentation
            # ------------------------------------------------

            if hasattr(
                params,
                "use_segmentation"
            ):

                params.use_segmentation = False


            # ------------------------------------------------
            # Smaller internal template
            # ------------------------------------------------

            if hasattr(
                params,
                "template_size"
            ):

                params.template_size = 100.0


            # ------------------------------------------------
            # Keep scale adaptation, but reduce search load
            # ------------------------------------------------

            if hasattr(
                params,
                "number_of_scales"
            ):

                params.number_of_scales = 17


            if hasattr(
                params,
                "scale_model_max_area"
            ):

                params.scale_model_max_area = 256.0


            # ------------------------------------------------
            # Disable unnecessary color-name features
            # ------------------------------------------------

            if hasattr(
                params,
                "use_color_names"
            ):

                params.use_color_names = False


            # Keep HOG because it helps tracking accuracy.

            return cv2.TrackerCSRT_create(
                params
            )


        except Exception:

            pass


    # --------------------------------------------------------
    # Standard API
    # --------------------------------------------------------

    if hasattr(
        cv2,
        "TrackerCSRT_create"
    ):

        return cv2.TrackerCSRT_create()


    # --------------------------------------------------------
    # Legacy API
    # --------------------------------------------------------

    if (
        hasattr(cv2, "legacy")
        and
        hasattr(
            cv2.legacy,
            "TrackerCSRT_create"
        )
    ):

        return (
            cv2.legacy
            .TrackerCSRT_create()
        )


    raise RuntimeError(
        "\nCSRT tracker is not available.\n"
        "Install opencv-contrib-python."
    )


# ============================================================
# CREATE SMALL TRACKING FRAME
# ============================================================

def make_tracking_frame(frame):

    if TRACKING_SCALE == 1.0:

        return frame


    return cv2.resize(
        frame,
        None,
        fx=TRACKING_SCALE,
        fy=TRACKING_SCALE,
        interpolation=cv2.INTER_LINEAR
    )


# ============================================================
# FULL BOX -> SMALL TRACKER BOX
# ============================================================

def full_to_tracking_bbox(
    x,
    y,
    w,
    h
):

    return (

        int(round(
            x * TRACKING_SCALE
        )),

        int(round(
            y * TRACKING_SCALE
        )),

        max(
            1,
            int(round(
                w * TRACKING_SCALE
            ))
        ),

        max(
            1,
            int(round(
                h * TRACKING_SCALE
            ))
        )
    )


# ============================================================
# SMALL TRACKER BOX -> FULL BOX
# ============================================================

def tracking_to_full_bbox(bbox):

    x, y, w, h = [
        float(v)
        for v in bbox
    ]


    inverse_scale = (
        1.0
        /
        TRACKING_SCALE
    )


    return (

        int(round(
            x * inverse_scale
        )),

        int(round(
            y * inverse_scale
        )),

        int(round(
            w * inverse_scale
        )),

        int(round(
            h * inverse_scale
        ))
    )


# ============================================================
# CLIP XYXY BOX
# ============================================================

def clip_xyxy(
    box,
    frame_shape
):

    x1, y1, x2, y2 = [
        int(v)
        for v in box
    ]


    frame_h, frame_w = (
        frame_shape[:2]
    )


    x1 = max(
        0,
        min(
            x1,
            frame_w - 1
        )
    )


    y1 = max(
        0,
        min(
            y1,
            frame_h - 1
        )
    )


    x2 = max(
        0,
        min(
            x2,
            frame_w
        )
    )


    y2 = max(
        0,
        min(
            y2,
            frame_h
        )
    )


    return (
        x1,
        y1,
        x2,
        y2
    )


# ============================================================
# IOU
# ============================================================

def bbox_iou(
    box_a,
    box_b
):

    ax1, ay1, ax2, ay2 = (
        box_a
    )

    bx1, by1, bx2, by2 = (
        box_b
    )


    ix1 = max(
        ax1,
        bx1
    )

    iy1 = max(
        ay1,
        by1
    )


    ix2 = min(
        ax2,
        bx2
    )

    iy2 = min(
        ay2,
        by2
    )


    iw = max(
        0,
        ix2 - ix1
    )

    ih = max(
        0,
        iy2 - iy1
    )


    intersection = (
        iw
        *
        ih
    )


    area_a = (
        max(
            0,
            ax2 - ax1
        )

        *

        max(
            0,
            ay2 - ay1
        )
    )


    area_b = (
        max(
            0,
            bx2 - bx1
        )

        *

        max(
            0,
            by2 - by1
        )
    )


    union = (
        area_a
        +
        area_b
        -
        intersection
    )


    if union <= 0:

        return 0.0


    return (
        intersection
        /
        union
    )


# ============================================================
# RELEASE YOLO MEMORY
# ============================================================

def release_model_memory(
    model=None,
    results=None
):

    try:

        if results is not None:

            del results

    except Exception:

        pass


    try:

        if model is not None:

            del model

    except Exception:

        pass


    gc.collect()


    try:

        import torch


        if torch.cuda.is_available():

            torch.cuda.empty_cache()


    except Exception:

        pass


# ============================================================
# ONE-SHOT YOLO26N TARGET VALIDATION
# ============================================================

def validate_selected_object(
    frame,
    selection
):

    model = None

    results = None


    sx1, sy1, sx2, sy2 = (
        selection
    )


    frame_h, frame_w = (
        frame.shape[:2]
    )


    selection_width = (
        sx2 - sx1
    )


    selection_height = (
        sy2 - sy1
    )


    # ========================================================
    # ADD PADDING AROUND USER SELECTION
    # ========================================================

    padding = int(

        SELECTION_PADDING

        *

        max(
            selection_width,
            selection_height
        )
    )


    crop_x1 = max(
        0,
        sx1 - padding
    )


    crop_y1 = max(
        0,
        sy1 - padding
    )


    crop_x2 = min(
        frame_w,
        sx2 + padding
    )


    crop_y2 = min(
        frame_h,
        sy2 + padding
    )


    crop = frame[
        crop_y1:crop_y2,
        crop_x1:crop_x2
    ].copy()


    if crop.size == 0:

        return None


    # ========================================================
    # RUN YOLO26N ONCE
    # ========================================================

    try:

        from ultralytics import YOLO


        model = YOLO(
            str(MODEL_PATH)
        )


        results = model.predict(
            source=crop,
            imgsz=YOLO_IMGSZ,
            conf=YOLO_CONF,
            iou=YOLO_IOU,
            verbose=False
        )


        if not results:

            return None


        result = results[0]


        if (
            result.boxes is None
            or
            len(result.boxes) == 0
        ):

            return None


        boxes = (
            result.boxes.xyxy
            .cpu()
            .numpy()
        )


        classes = (
            result.boxes.cls
            .int()
            .cpu()
            .tolist()
        )


        confidences = (
            result.boxes.conf
            .cpu()
            .numpy()
        )


        names = (
            result.names
        )


        best_candidate = None

        best_score = -1.0


        # ====================================================
        # FIND BEST DRONE INSIDE USER SELECTION
        # ====================================================

        for (
            local_box,
            class_id,
            confidence

        ) in zip(
            boxes,
            classes,
            confidences
        ):

            class_name = str(
                names[class_id]
            ).strip()


            confidence = float(
                confidence
            )


            if (
                class_name.lower()
                !=
                DRONE_CLASS_NAME.lower()
            ):

                continue


            lx1, ly1, lx2, ly2 = [
                int(v)
                for v in local_box
            ]


            # Crop coordinates -> camera coordinates

            gx1 = (
                crop_x1
                +
                lx1
            )


            gy1 = (
                crop_y1
                +
                ly1
            )


            gx2 = (
                crop_x1
                +
                lx2
            )


            gy2 = (
                crop_y1
                +
                ly2
            )


            global_box = (
                gx1,
                gy1,
                gx2,
                gy2
            )


            center_x = (
                gx1 + gx2
            ) / 2.0


            center_y = (
                gy1 + gy2
            ) / 2.0


            center_inside = (
                sx1 <= center_x <= sx2

                and

                sy1 <= center_y <= sy2
            )


            overlap = bbox_iou(
                selection,
                global_box
            )


            if (
                not center_inside
                and
                overlap < 0.05
            ):

                continue


            score = (
                confidence
                +
                overlap
            )


            if center_inside:

                score += 1.0


            if score > best_score:

                best_score = (
                    score
                )


                best_candidate = {

                    "bbox":
                        global_box,

                    "confidence":
                        confidence
                }


        return best_candidate


    finally:

        release_model_memory(
            model,
            results
        )


# ============================================================
# TARGET SIZE / APPROACH-AWAY STATE
# ============================================================

def calculate_target_motion(
    current_width,
    current_height
):

    global smoothed_target_scale


    if (
        reference_box_area <= 0
        or
        current_width <= 0
        or
        current_height <= 0
    ):

        return (
            "LOCK RANGE",
            1.0
        )


    # ========================================================
    # CURRENT BOX AREA
    # ========================================================

    current_area = (

        float(current_width)

        *

        float(current_height)
    )


    # ========================================================
    # LINEAR SCALE RELATIVE TO LOCK-TIME TARGET
    #
    # Example:
    #
    # original area = 10000
    # new area      = 22500
    #
    # scale = sqrt(22500/10000)
    #       = 1.5
    #
    # target appears 1.5x larger
    # ========================================================

    raw_scale = math.sqrt(

        current_area

        /

        reference_box_area
    )


    # ========================================================
    # SMOOTH
    # ========================================================

    smoothed_target_scale = (

        SIZE_SMOOTHING
        *
        smoothed_target_scale

        +

        (
            1.0
            -
            SIZE_SMOOTHING
        )
        *
        raw_scale
    )


    # ========================================================
    # MOTION STATE
    # ========================================================

    if (
        smoothed_target_scale
        >
        APPROACH_THRESHOLD
    ):

        return (
            "APPROACHING",
            smoothed_target_scale
        )


    if (
        smoothed_target_scale
        <
        AWAY_THRESHOLD
    ):

        return (
            "MOVING AWAY",
            smoothed_target_scale
        )


    return (
        "LOCK RANGE",
        smoothed_target_scale
    )


# ============================================================
# DRAW CENTER TARGET RETICLE
# ============================================================

def draw_target_window(image):

    frame_h, frame_w = (
        image.shape[:2]
    )


    center_x = (
        frame_w // 2
    )


    center_y = (
        frame_h // 2
    )


    half_w = (
        TARGET_WINDOW_WIDTH // 2
    )


    half_h = (
        TARGET_WINDOW_HEIGHT // 2
    )


    left = (
        center_x - half_w
    )


    right = (
        center_x + half_w
    )


    top = (
        center_y - half_h
    )


    bottom = (
        center_y + half_h
    )


    color = (
        255,
        255,
        255
    )


    corner = (
        TARGET_CORNER_LENGTH
    )


    # ========================================================
    # FOUR CORNERS
    # ========================================================

    cv2.line(
        image,
        (left, top),
        (
            left + corner,
            top
        ),
        color,
        2
    )


    cv2.line(
        image,
        (left, top),
        (
            left,
            top + corner
        ),
        color,
        2
    )


    cv2.line(
        image,
        (right, top),
        (
            right - corner,
            top
        ),
        color,
        2
    )


    cv2.line(
        image,
        (right, top),
        (
            right,
            top + corner
        ),
        color,
        2
    )


    cv2.line(
        image,
        (left, bottom),
        (
            left + corner,
            bottom
        ),
        color,
        2
    )


    cv2.line(
        image,
        (left, bottom),
        (
            left,
            bottom - corner
        ),
        color,
        2
    )


    cv2.line(
        image,
        (right, bottom),
        (
            right - corner,
            bottom
        ),
        color,
        2
    )


    cv2.line(
        image,
        (right, bottom),
        (
            right,
            bottom - corner
        ),
        color,
        2
    )


    # ========================================================
    # CENTER CIRCLE
    # ========================================================

    cv2.circle(
        image,
        (
            center_x,
            center_y
        ),
        CENTER_CIRCLE_RADIUS,
        color,
        2
    )


    gap = (
        CENTER_CIRCLE_RADIUS
        +
        4
    )


    # ========================================================
    # CENTER CROSS
    # ========================================================

    cv2.line(
        image,
        (
            center_x
            -
            CENTER_CROSS_LENGTH,
            center_y
        ),
        (
            center_x - gap,
            center_y
        ),
        color,
        2
    )


    cv2.line(
        image,
        (
            center_x + gap,
            center_y
        ),
        (
            center_x
            +
            CENTER_CROSS_LENGTH,
            center_y
        ),
        color,
        2
    )


    cv2.line(
        image,
        (
            center_x,
            center_y
            -
            CENTER_CROSS_LENGTH
        ),
        (
            center_x,
            center_y - gap
        ),
        color,
        2
    )


    cv2.line(
        image,
        (
            center_x,
            center_y + gap
        ),
        (
            center_x,
            center_y
            +
            CENTER_CROSS_LENGTH
        ),
        color,
        2
    )


    # Center dot

    cv2.circle(
        image,
        (
            center_x,
            center_y
        ),
        2,
        color,
        -1
    )


    # ========================================================
    # OUTER TICKS
    # ========================================================

    tick = 12


    cv2.line(
        image,
        (
            left - tick,
            center_y
        ),
        (
            left,
            center_y
        ),
        color,
        2
    )


    cv2.line(
        image,
        (
            right,
            center_y
        ),
        (
            right + tick,
            center_y
        ),
        color,
        2
    )


    cv2.line(
        image,
        (
            center_x,
            top - tick
        ),
        (
            center_x,
            top
        ),
        color,
        2
    )


    cv2.line(
        image,
        (
            center_x,
            bottom
        ),
        (
            center_x,
            bottom + tick
        ),
        color,
        2
    )


# ============================================================
# MOUSE CALLBACK
# ============================================================

def mouse_callback(
    event,
    x,
    y,
    flags,
    param
):

    global drawing

    global start_x
    global start_y

    global mouse_x
    global mouse_y

    global pending_selection
    global pending_selection_frame

    global latest_clean_frame


    # ========================================================
    # START
    # ========================================================

    if event == cv2.EVENT_LBUTTONDOWN:

        drawing = True


        start_x = x
        start_y = y


        mouse_x = x
        mouse_y = y


    # ========================================================
    # DRAG
    # ========================================================

    elif (
        event == cv2.EVENT_MOUSEMOVE
        and
        drawing
    ):

        mouse_x = x

        mouse_y = y


    # ========================================================
    # RELEASE
    # ========================================================

    elif (
        event == cv2.EVENT_LBUTTONUP
        and
        drawing
    ):

        drawing = False


        mouse_x = x

        mouse_y = y


        x1 = min(
            start_x,
            mouse_x
        )


        y1 = min(
            start_y,
            mouse_y
        )


        x2 = max(
            start_x,
            mouse_x
        )


        y2 = max(
            start_y,
            mouse_y
        )


        if (
            x2 - x1
            <
            MIN_SELECTION_SIZE

            or

            y2 - y1
            <
            MIN_SELECTION_SIZE
        ):

            return


        if latest_clean_frame is None:

            return


        pending_selection = (
            x1,
            y1,
            x2,
            y2
        )


        # Copy only when the user actually selects something.
        # We do NOT copy every camera frame.

        pending_selection_frame = (
            latest_clean_frame.copy()
        )


# ============================================================
# CHECK MODEL
# ============================================================

if not MODEL_PATH.exists():

    raise FileNotFoundError(
        f"\nModel does not exist:\n"
        f"{MODEL_PATH}"
    )


# ============================================================
# CHECK TRACKER
# ============================================================

test_tracker = (
    create_tracker()
)

del test_tracker


# ============================================================
# OPEN CAMERA
#
# Use EXACTLY the same method as the camera test that gave
# approximately 30-40 FPS.
# ============================================================

cap = cv2.VideoCapture(
    CAMERA_INDEX
)


if not cap.isOpened():

    raise RuntimeError(
        f"Could not open camera {CAMERA_INDEX}."
    )


# ============================================================
# READ ACTUAL CAMERA SETTINGS
# ============================================================

actual_camera_width = int(
    cap.get(
        cv2.CAP_PROP_FRAME_WIDTH
    )
)


actual_camera_height = int(
    cap.get(
        cv2.CAP_PROP_FRAME_HEIGHT
    )
)


actual_camera_fps = cap.get(
    cv2.CAP_PROP_FPS
)


print(
    f"Camera: "
    f"{actual_camera_width} x "
    f"{actual_camera_height}"
)


print(
    f"Camera reported FPS: "
    f"{actual_camera_fps:.1f}"
)


# ============================================================
# WINDOW
# ============================================================

cv2.namedWindow(
    WINDOW_NAME
)


cv2.setMouseCallback(
    WINDOW_NAME,
    mouse_callback
)


# ============================================================
# MAIN LOOP
# ============================================================

while True:

    # ========================================================
    # READ CAMERA
    # ========================================================

    ret, frame = cap.read()


    if not ret:

        break


    # Store reference only.
    # No full-frame copy here.

    latest_clean_frame = frame


    frame_h, frame_w = (
        frame.shape[:2]
    )


    # ========================================================
    # NEW TARGET SELECTION
    # ========================================================

    if (
        pending_selection is not None
        and
        pending_selection_frame is not None
    ):

        selection = (
            pending_selection
        )


        selection_frame = (
            pending_selection_frame
        )


        pending_selection = None

        pending_selection_frame = None


        # ====================================================
        # ONE YOLO26N INFERENCE
        # ====================================================

        detection = (
            validate_selected_object(
                selection_frame,
                selection
            )
        )


        # ====================================================
        # VALID DRONE
        # ====================================================

        if detection is not None:

            x1, y1, x2, y2 = (
                clip_xyxy(
                    detection["bbox"],
                    selection_frame.shape
                )
            )


            width = (
                x2 - x1
            )


            height = (
                y2 - y1
            )


            if (
                width > 0
                and
                height > 0
            ):

                # =============================================
                # SAVE LOCKED IMAGE
                # =============================================

                new_reference_image = (

                    selection_frame[
                        y1:y2,
                        x1:x2
                    ]

                    .copy()
                )


                # =============================================
                # SAVE ORIGINAL BOUNDING-BOX SIZE
                # =============================================

                reference_box_width = (
                    width
                )


                reference_box_height = (
                    height
                )


                reference_box_area = (

                    float(width)

                    *

                    float(height)
                )


                # Exactly 1.0 at lock time.

                smoothed_target_scale = (
                    1.0
                )


                # =============================================
                # CREATE SMALL TRACKING FRAME
                # =============================================

                small_initial_frame = (
                    make_tracking_frame(
                        selection_frame
                    )
                )


                small_initial_bbox = (
                    full_to_tracking_bbox(
                        x1,
                        y1,
                        width,
                        height
                    )
                )


                # =============================================
                # INITIALIZE TRACKER
                # =============================================

                new_tracker = (
                    create_tracker()
                )


                new_tracker.init(
                    small_initial_frame,
                    small_initial_bbox
                )


                tracker = (
                    new_tracker
                )


                reference_image = (
                    new_reference_image
                )


                locked_confidence = float(
                    detection["confidence"]
                )


                tracking = True


                last_full_bbox = (
                    x1,
                    y1,
                    width,
                    height
                )


                tracker_frame_counter = 0


        # Restart FPS measurement after one-shot inference.

        fps_frame_counter = 0

        fps_measurement_start = (
            time.perf_counter()
        )


    # ========================================================
    # CREATE DISPLAY FRAME
    # ========================================================

    display = (
        frame.copy()
    )


    status_text = (
        "NOT LOCKED"
    )


    status_color = (
        0,
        0,
        255
    )


    target_motion_text = ""

    target_motion_color = (
        255,
        255,
        255
    )


    target_center = None


    # ========================================================
    # TRACK TARGET
    # ========================================================

    if (
        tracking
        and
        tracker is not None
    ):

        tracker_frame_counter += 1


        # ====================================================
        # RUN CSRT ONLY EVERY N FRAMES
        # ====================================================

        should_update_tracker = (

            tracker_frame_counter
            %
            TRACKER_UPDATE_EVERY_N_FRAMES

            == 0
        )


        if should_update_tracker:

            small_frame = (
                make_tracking_frame(
                    frame
                )
            )


            success, small_bbox = (
                tracker.update(
                    small_frame
                )
            )


            if success:

                x, y, w, h = (
                    tracking_to_full_bbox(
                        small_bbox
                    )
                )


                # =============================================
                # KEEP BOX INSIDE IMAGE
                # =============================================

                x = max(
                    0,
                    min(
                        x,
                        frame_w - 1
                    )
                )


                y = max(
                    0,
                    min(
                        y,
                        frame_h - 1
                    )
                )


                w = max(
                    1,
                    min(
                        w,
                        frame_w - x
                    )
                )


                h = max(
                    1,
                    min(
                        h,
                        frame_h - y
                    )
                )


                last_full_bbox = (
                    x,
                    y,
                    w,
                    h
                )


            else:

                tracking = False

                tracker = None

                last_full_bbox = None


        # ====================================================
        # USE MOST RECENT GOOD BOX
        # ====================================================

        if (
            tracking
            and
            last_full_bbox is not None
        ):

            x, y, w, h = (
                last_full_bbox
            )


            # =================================================
            # APPROACH / AWAY
            # =================================================

            (
                target_motion_text,
                target_scale

            ) = calculate_target_motion(
                w,
                h
            )


            if (
                target_motion_text
                ==
                "APPROACHING"
            ):

                target_motion_color = (
                    0,
                    255,
                    0
                )


            elif (
                target_motion_text
                ==
                "MOVING AWAY"
            ):

                target_motion_color = (
                    0,
                    165,
                    255
                )


            else:

                target_motion_color = (
                    255,
                    255,
                    255
                )


            # =================================================
            # TARGET CENTER
            # =================================================

            target_center_x = (
                x + w // 2
            )


            target_center_y = (
                y + h // 2
            )


            target_center = (
                target_center_x,
                target_center_y
            )


            # =================================================
            # TARGET BOX
            # =================================================

            cv2.rectangle(
                display,
                (
                    x,
                    y
                ),
                (
                    x + w,
                    y + h
                ),
                (
                    0,
                    255,
                    0
                ),
                2
            )


            # =================================================
            # TARGET CENTER DOT
            # =================================================

            cv2.circle(
                display,
                target_center,
                4,
                (
                    0,
                    0,
                    255
                ),
                -1
            )


            status_text = (
                "TARGET LOCKED"
            )


            status_color = (
                0,
                255,
                0
            )


    # ========================================================
    # CENTER -> TARGET LINE
    # ========================================================

    if target_center is not None:

        frame_center = (
            frame_w // 2,
            frame_h // 2
        )


        cv2.line(
            display,
            frame_center,
            target_center,
            (
                255,
                255,
                0
            ),
            1
        )


    # ========================================================
    # TARGET RETICLE
    # ========================================================

    draw_target_window(
        display
    )


    # ========================================================
    # LIVE MANUAL SELECTION
    # ========================================================

    if drawing:

        sx1 = min(
            start_x,
            mouse_x
        )


        sy1 = min(
            start_y,
            mouse_y
        )


        sx2 = max(
            start_x,
            mouse_x
        )


        sy2 = max(
            start_y,
            mouse_y
        )


        cv2.rectangle(
            display,
            (
                sx1,
                sy1
            ),
            (
                sx2,
                sy2
            ),
            (
                255,
                0,
                255
            ),
            2
        )


    # ========================================================
    # LOCKED TARGET PREVIEW
    # ========================================================

    if (
        reference_image is not None
        and
        tracking
    ):

        reference_h, reference_w = (
            reference_image.shape[:2]
        )


        if (
            reference_h > 0
            and
            reference_w > 0
        ):

            preview_width = (
                120
            )


            preview_height = int(

                preview_width

                *

                reference_h

                /

                reference_w
            )


            preview_height = max(
                35,
                min(
                    preview_height,
                    100
                )
            )


            preview = cv2.resize(
                reference_image,
                (
                    preview_width,
                    preview_height
                )
            )


            preview_x1 = (
                frame_w
                -
                preview_width
                -
                15
            )


            preview_y1 = (
                40
            )


            preview_x2 = (
                preview_x1
                +
                preview_width
            )


            preview_y2 = (
                preview_y1
                +
                preview_height
            )


            if (
                preview_x1 >= 0
                and
                preview_y2 < frame_h
            ):

                display[
                    preview_y1:preview_y2,
                    preview_x1:preview_x2
                ] = preview


                cv2.rectangle(
                    display,
                    (
                        preview_x1,
                        preview_y1
                    ),
                    (
                        preview_x2,
                        preview_y2
                    ),
                    (
                        255,
                        255,
                        255
                    ),
                    1
                )


                cv2.putText(
                    display,
                    "LOCKED TARGET",
                    (
                        preview_x1,
                        preview_y1 - 7
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (
                        255,
                        255,
                        255
                    ),
                    1
                )


    # ========================================================
    # REAL DISPLAY FPS
    # ========================================================

    fps_frame_counter += 1


    now = (
        time.perf_counter()
    )


    fps_elapsed = (
        now
        -
        fps_measurement_start
    )


    if fps_elapsed >= 1.0:

        display_fps = (

            fps_frame_counter

            /

            fps_elapsed
        )


        fps_frame_counter = 0


        fps_measurement_start = (
            now
        )


    # ========================================================
    # FPS DISPLAY
    # ========================================================

    cv2.putText(
        display,
        f"FPS: {display_fps:.1f}",
        (
            15,
            25
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.60,
        (
            255,
            255,
            255
        ),
        2
    )


    # ========================================================
    # TARGET STATUS
    # ========================================================

    cv2.putText(
        display,
        status_text,
        (
            15,
            52
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        status_color,
        2
    )


    # ========================================================
    # TARGET INFO
    # ========================================================

    if tracking:

        cv2.putText(
            display,
            (
                f"CONFIDENCE: "
                f"{locked_confidence * 100.0:.1f}%"
            ),
            (
                15,
                78
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (
                255,
                255,
                255
            ),
            1
        )


        cv2.putText(
            display,
            target_motion_text,
            (
                15,
                103
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            target_motion_color,
            2
        )


    # ========================================================
    # SHOW
    # ========================================================

    cv2.imshow(
        WINDOW_NAME,
        display
    )


    key = (
        cv2.waitKey(1)
        &
        0xFF
    )


    if (
        key == ord("q")
        or
        key == ord("Q")
    ):

        break


# ============================================================
# CLEANUP
# ============================================================

cap.release()

cv2.destroyAllWindows()
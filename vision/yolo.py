"""
Clean YOLOv3 2D webcam runner (TensorFlow 2 / tf.keras friendly).
- No RealSense / depth
- No K.get_session()
- No multi_gpu_model
- Outputs 2D pixel center (x, y) for each detection
"""

import os
import time
import cv2
import numpy as np
import tensorflow as tf

from PIL import Image, ImageFont, ImageDraw

# Repo imports (must exist in your project)
from yolo3.model import yolo_eval, yolo_body, tiny_yolo_body
from yolo3.utils import letterbox_image
from keras.layers import Input
from keras.models import load_model


class YOLO2D:
    def __init__(
        self,
        model_path="model_data/yolo_weights.h5",
        anchors_path="model_data/yolo_anchors.txt",
        classes_path="model_data/coco_classes.txt",
        score=0.3,
        iou=0.45,
        model_image_size=(640, 480),
        tiny=False,
    ):
        self.model_path = model_path
        self.anchors_path = anchors_path
        self.classes_path = classes_path
        self.score = score
        self.iou = iou
        self.model_image_size = model_image_size
        self.tiny = tiny

        self.class_names = self._get_class()
        self.anchors = self._get_anchors()
        self.num_classes = len(self.class_names)

        self.model = self._load_or_build_model()

        # TF2 eager-safe: yolo_eval returns tensors; we run them through a tf.function
        @tf.function
        def _predict(image_data):
            yolo_outputs = self.model(image_data, training=False)
            # model may return a single tensor or list; normalize to list
            if not isinstance(yolo_outputs, (list, tuple)):
                yolo_outputs = [yolo_outputs]
            boxes, scores, classes = yolo_eval(
                yolo_outputs,
                self.anchors,
                self.num_classes,
                image_shape=tf.constant(
                    [self.image_h, self.image_w], dtype=tf.int32),
                score_threshold=self.score,
                iou_threshold=self.iou,
            )
            return boxes, scores, classes

        self._predict_fn = _predict

    def _get_class(self):
        with open(self.classes_path, "r", encoding="utf-8") as f:
            return [c.strip() for c in f.readlines() if c.strip()]

    def _get_anchors(self):
        with open(self.anchors_path, "r", encoding="utf-8") as f:
            anchors = f.readline().strip().split(",")
        anchors = [float(x) for x in anchors]
        return np.array(anchors).reshape(-1, 2)

    def _load_or_build_model(self):
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Model file not found: {self.model_path}")

        model = load_model(self.model_path, compile=False)

        # Some YOLO h5 files load without output shape metadata;
        # we sanity-check last layer compatibility.
        out = model.output
        if isinstance(out, (list, tuple)):
            num_outputs = len(out)
        else:
            num_outputs = 1

        # If you use tiny model, typical outputs = 2; else = 3
        if self.tiny and num_outputs != 2:
            print("[WARN] tiny=True but model outputs != 2. Continuing anyway.")
        if (not self.tiny) and num_outputs not in (1, 3):
            print("[WARN] expected YOLOv3 outputs ~3. Continuing anyway.")

        return model

    def detect_image_bgr(self, frame_bgr):
        """Run detection on an OpenCV BGR frame; return annotated frame + detections."""
        self.image_h, self.image_w = frame_bgr.shape[:2]

        # Convert to PIL RGB for letterbox preprocessing
        image_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(image_rgb)

        boxed_image = letterbox_image(pil_image, self.model_image_size)
        image_data = np.array(boxed_image, dtype=np.float32) / 255.0
        image_data = np.expand_dims(image_data, 0)  # (1, h, w, 3)

        # Run model
        boxes, scores, classes = self._predict_fn(tf.constant(image_data))

        boxes = boxes.numpy()
        scores = scores.numpy()
        classes = classes.numpy().astype(int)

        annotated = frame_bgr.copy()
        detections = []

        for b, s, c in zip(boxes, scores, classes):
            top, left, bottom, right = b
            top = max(0, int(top))
            left = max(0, int(left))
            bottom = min(self.image_h - 1, int(bottom))
            right = min(self.image_w - 1, int(right))

            x_center = int((left + right) / 2)
            y_center = int((top + bottom) / 2)

            label = self.class_names[c] if 0 <= c < len(
                self.class_names) else str(c)
            text = f"{label} {s:.2f} ({x_center},{y_center})"

            cv2.rectangle(annotated, (left, top),
                          (right, bottom), (0, 255, 0), 2)
            cv2.circle(annotated, (x_center, y_center), 3, (0, 0, 255), -1)
            cv2.putText(
                annotated,
                text,
                (left, max(0, top - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            detections.append(
                {
                    "class": label,
                    "score": float(s),
                    "box": (left, top, right, bottom),
                    "center": (x_center, y_center),
                }
            )

        return annotated, detections


def main():
    # --- Adjust these paths to match your repo ---
    yolo = YOLO2D(
        model_path="model_data/yolo_weights.h5",              # <-- your .h5
        anchors_path="model_data/yolo_anchors.txt",   # <-- your anchors
        classes_path="model_data/coco_classes.txt",   # <-- your class names
        score=0.3,
        iou=0.45,
        model_image_size=(640, 480),
        tiny=False,
    )

    cap = cv2.VideoCapture(0)  # change to 1 if you have multiple cameras
    if not cap.isOpened():
        raise RuntimeError(
            "Could not open webcam. Try VideoCapture(1) or check permissions.")

    prev = time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        annotated, _ = yolo.detect_image_bgr(frame)

        # FPS
        now = time.time()
        fps = 1.0 / max(1e-6, (now - prev))
        prev = now
        cv2.putText(
            annotated,
            f"FPS: {fps:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.imshow("YOLOv3 2D Camera", annotated)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):  # ESC or q
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

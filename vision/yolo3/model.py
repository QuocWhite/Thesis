"""
YOLO_v3 Model Defined in Keras (TF/Keras 3 compatible).

This version removes keras.backend usage (K.*) and replaces it with TensorFlow ops (tf.*)
to avoid Keras 3 backend API breakages.
"""

from functools import wraps
import numpy as np
import tensorflow as tf

from keras.layers import (
    Conv2D,
    Add,
    ZeroPadding2D,
    UpSampling2D,
    Concatenate,
    MaxPooling2D,
    LeakyReLU,
    BatchNormalization,
)
from keras.models import Model
from keras.regularizers import l2

from yolo3.utils import compose


@wraps(Conv2D)
def DarknetConv2D(*args, **kwargs):
    """Wrapper to set Darknet parameters for Convolution2D."""
    darknet_conv_kwargs = {"kernel_regularizer": l2(5e-4)}
    darknet_conv_kwargs["padding"] = "valid" if kwargs.get("strides") == (2, 2) else "same"
    darknet_conv_kwargs.update(kwargs)
    return Conv2D(*args, **darknet_conv_kwargs)


def DarknetConv2D_BN_Leaky(*args, **kwargs):
    """Darknet Convolution2D followed by BatchNormalization and LeakyReLU."""
    no_bias_kwargs = {"use_bias": False}
    no_bias_kwargs.update(kwargs)
    return compose(
        DarknetConv2D(*args, **no_bias_kwargs),
        BatchNormalization(),
        LeakyReLU(alpha=0.1),
    )


def resblock_body(x, num_filters, num_blocks):
    """A series of resblocks starting with a downsampling Conv2D."""
    x = ZeroPadding2D(((1, 0), (1, 0)))(x)
    x = DarknetConv2D_BN_Leaky(num_filters, (3, 3), strides=(2, 2))(x)
    for _ in range(num_blocks):
        y = compose(
            DarknetConv2D_BN_Leaky(num_filters // 2, (1, 1)),
            DarknetConv2D_BN_Leaky(num_filters, (3, 3)),
        )(x)
        x = Add()([x, y])
    return x


def darknet_body(x):
    """Darknet body having 52 Conv2D layers."""
    x = DarknetConv2D_BN_Leaky(32, (3, 3))(x)
    x = resblock_body(x, 64, 1)
    x = resblock_body(x, 128, 2)
    x = resblock_body(x, 256, 8)
    x = resblock_body(x, 512, 8)
    x = resblock_body(x, 1024, 4)
    return x


def make_last_layers(x, num_filters, out_filters):
    """6 Conv2D_BN_Leaky layers followed by a Conv2D_linear layer."""
    x = compose(
        DarknetConv2D_BN_Leaky(num_filters, (1, 1)),
        DarknetConv2D_BN_Leaky(num_filters * 2, (3, 3)),
        DarknetConv2D_BN_Leaky(num_filters, (1, 1)),
        DarknetConv2D_BN_Leaky(num_filters * 2, (3, 3)),
        DarknetConv2D_BN_Leaky(num_filters, (1, 1)),
    )(x)
    y = compose(
        DarknetConv2D_BN_Leaky(num_filters * 2, (3, 3)),
        DarknetConv2D(out_filters, (1, 1)),
    )(x)
    return x, y


def yolo_body(inputs, num_anchors, num_classes):
    """Create YOLO_V3 model CNN body in Keras."""
    darknet = Model(inputs, darknet_body(inputs))
    x, y1 = make_last_layers(darknet.output, 512, num_anchors * (num_classes + 5))

    x = compose(
        DarknetConv2D_BN_Leaky(256, (1, 1)),
        UpSampling2D(2),
    )(x)
    x = Concatenate()([x, darknet.layers[152].output])
    x, y2 = make_last_layers(x, 256, num_anchors * (num_classes + 5))

    x = compose(
        DarknetConv2D_BN_Leaky(128, (1, 1)),
        UpSampling2D(2),
    )(x)
    x = Concatenate()([x, darknet.layers[92].output])
    x, y3 = make_last_layers(x, 128, num_anchors * (num_classes + 5))

    return Model(inputs, [y1, y2, y3])


def tiny_yolo_body(inputs, num_anchors, num_classes):
    """Create Tiny YOLO_v3 model CNN body in keras."""
    x1 = compose(
        DarknetConv2D_BN_Leaky(16, (3, 3)),
        MaxPooling2D(pool_size=(2, 2), strides=(2, 2), padding="same"),
        DarknetConv2D_BN_Leaky(32, (3, 3)),
        MaxPooling2D(pool_size=(2, 2), strides=(2, 2), padding="same"),
        DarknetConv2D_BN_Leaky(64, (3, 3)),
        MaxPooling2D(pool_size=(2, 2), strides=(2, 2), padding="same"),
        DarknetConv2D_BN_Leaky(128, (3, 3)),
        MaxPooling2D(pool_size=(2, 2), strides=(2, 2), padding="same"),
        DarknetConv2D_BN_Leaky(256, (3, 3)),
    )(inputs)

    x2 = compose(
        MaxPooling2D(pool_size=(2, 2), strides=(2, 2), padding="same"),
        DarknetConv2D_BN_Leaky(512, (3, 3)),
        MaxPooling2D(pool_size=(2, 2), strides=(1, 1), padding="same"),
        DarknetConv2D_BN_Leaky(1024, (3, 3)),
        DarknetConv2D_BN_Leaky(256, (1, 1)),
    )(x1)

    y1 = compose(
        DarknetConv2D_BN_Leaky(512, (3, 3)),
        DarknetConv2D(num_anchors * (num_classes + 5), (1, 1)),
    )(x2)

    x2 = compose(
        DarknetConv2D_BN_Leaky(128, (1, 1)),
        UpSampling2D(2),
    )(x2)

    y2 = compose(
        Concatenate(),
        DarknetConv2D_BN_Leaky(256, (3, 3)),
        DarknetConv2D(num_anchors * (num_classes + 5), (1, 1)),
    )([x2, x1])

    return Model(inputs, [y1, y2])


# -------------------------
# TF replacements for K.* ops
# -------------------------

def _dtype(x):
    return x.dtype


def yolo_head(feats, anchors, num_classes, input_shape, calc_loss=False):
    """Convert final layer features to bounding box parameters."""
    feats = tf.convert_to_tensor(feats)
    num_anchors = len(anchors)

    anchors_tensor = tf.reshape(
        tf.constant(anchors, dtype=tf.float32), [1, 1, 1, num_anchors, 2]
    )

    grid_shape = tf.shape(feats)[1:3]  # height, width

    grid_y = tf.tile(
        tf.reshape(tf.range(grid_shape[0]), [-1, 1, 1, 1]),
        [1, grid_shape[1], 1, 1],
    )
    grid_x = tf.tile(
        tf.reshape(tf.range(grid_shape[1]), [1, -1, 1, 1]),
        [grid_shape[0], 1, 1, 1],
    )

    grid = tf.concat([grid_x, grid_y], axis=-1)
    grid = tf.cast(grid, feats.dtype)

    feats = tf.reshape(
        feats, [-1, grid_shape[0], grid_shape[1], num_anchors, num_classes + 5]
    )

    # Adjust predictions to each spatial grid point and anchor size.
    box_xy = (tf.sigmoid(feats[..., :2]) + grid) / tf.cast(
        grid_shape[::-1], feats.dtype
    )
    box_wh = tf.exp(feats[..., 2:4]) * anchors_tensor / tf.cast(
        input_shape[::-1], feats.dtype
    )
    box_confidence = tf.sigmoid(feats[..., 4:5])
    box_class_probs = tf.sigmoid(feats[..., 5:])

    if calc_loss:
        return grid, feats, box_xy, box_wh
    return box_xy, box_wh, box_confidence, box_class_probs


def yolo_correct_boxes(box_xy, box_wh, input_shape, image_shape):
    """Get corrected boxes."""
    box_yx = box_xy[..., ::-1]
    box_hw = box_wh[..., ::-1]

    input_shape = tf.cast(input_shape, box_yx.dtype)
    image_shape = tf.cast(image_shape, box_yx.dtype)

    new_shape = tf.round(image_shape * tf.reduce_min(input_shape / image_shape))
    offset = (input_shape - new_shape) / 2.0 / input_shape
    scale = input_shape / new_shape

    box_yx = (box_yx - offset) * scale
    box_hw = box_hw * scale

    box_mins = box_yx - (box_hw / 2.0)
    box_maxes = box_yx + (box_hw / 2.0)

    boxes = tf.concat(
        [
            box_mins[..., 0:1],  # y_min
            box_mins[..., 1:2],  # x_min
            box_maxes[..., 0:1],  # y_max
            box_maxes[..., 1:2],  # x_max
        ],
        axis=-1,
    )

    # Scale boxes back to original image shape.
    boxes = boxes * tf.concat([image_shape, image_shape], axis=-1)
    return boxes


def yolo_boxes_and_scores(feats, anchors, num_classes, input_shape, image_shape):
    """Process Conv layer output."""
    box_xy, box_wh, box_confidence, box_class_probs = yolo_head(
        feats, anchors, num_classes, input_shape
    )
    boxes = yolo_correct_boxes(box_xy, box_wh, input_shape, image_shape)
    boxes = tf.reshape(boxes, [-1, 4])
    box_scores = box_confidence * box_class_probs
    box_scores = tf.reshape(box_scores, [-1, num_classes])
    return boxes, box_scores


def yolo_eval(
    yolo_outputs,
    anchors,
    num_classes,
    image_shape,
    max_boxes=20,
    score_threshold=0.6,
    iou_threshold=0.5,
):
    """Evaluate YOLO model on given input and return filtered boxes."""
    num_layers = len(yolo_outputs)
    anchor_mask = (
        [[6, 7, 8], [3, 4, 5], [0, 1, 2]]
        if num_layers == 3
        else [[3, 4, 5], [1, 2, 3]]
    )

    input_shape = tf.shape(yolo_outputs[0])[1:3] * 32

    boxes = []
    box_scores = []
    for l in range(num_layers):
        _boxes, _box_scores = yolo_boxes_and_scores(
            yolo_outputs[l],
            anchors[anchor_mask[l]],
            num_classes,
            input_shape,
            image_shape,
        )
        boxes.append(_boxes)
        box_scores.append(_box_scores)

    boxes = tf.concat(boxes, axis=0)
    box_scores = tf.concat(box_scores, axis=0)

    mask = box_scores >= score_threshold
    max_boxes_tensor = tf.constant(max_boxes, dtype=tf.int32)

    boxes_ = []
    scores_ = []
    classes_ = []

    for c in range(num_classes):
        class_boxes = tf.boolean_mask(boxes, mask[:, c])
        class_box_scores = tf.boolean_mask(box_scores[:, c], mask[:, c])

        nms_index = tf.image.non_max_suppression(
            class_boxes,
            class_box_scores,
            max_boxes_tensor,
            iou_threshold=iou_threshold,
        )

        class_boxes = tf.gather(class_boxes, nms_index)
        class_box_scores = tf.gather(class_box_scores, nms_index)
        classes = tf.ones_like(class_box_scores, dtype=tf.int32) * c

        boxes_.append(class_boxes)
        scores_.append(class_box_scores)
        classes_.append(classes)

    boxes_ = tf.concat(boxes_, axis=0)
    scores_ = tf.concat(scores_, axis=0)
    classes_ = tf.concat(classes_, axis=0)

    return boxes_, scores_, classes_


# -------------------------
# Training utilities (kept mostly as-is; still uses tf ops heavily)
# preprocess_true_boxes is numpy-based and fine.
# box_iou and yolo_loss below are made TF-compatible too.
# -------------------------

def preprocess_true_boxes(true_boxes, input_shape, anchors, num_classes):
    """Preprocess true boxes to training input format."""
    assert (true_boxes[..., 4] < num_classes).all(), "class id must be less than num_classes"
    num_layers = len(anchors) // 3
    anchor_mask = (
        [[6, 7, 8], [3, 4, 5], [0, 1, 2]]
        if num_layers == 3
        else [[3, 4, 5], [1, 2, 3]]
    )

    true_boxes = np.array(true_boxes, dtype="float32")
    input_shape = np.array(input_shape, dtype="int32")

    boxes_xy = (true_boxes[..., 0:2] + true_boxes[..., 2:4]) // 2
    boxes_wh = true_boxes[..., 2:4] - true_boxes[..., 0:2]
    true_boxes[..., 0:2] = boxes_xy / input_shape[::-1]
    true_boxes[..., 2:4] = boxes_wh / input_shape[::-1]

    m = true_boxes.shape[0]
    grid_shapes = [input_shape // {0: 32, 1: 16, 2: 8}[l] for l in range(num_layers)]
    y_true = [
        np.zeros(
            (m, grid_shapes[l][0], grid_shapes[l][1], len(anchor_mask[l]), 5 + num_classes),
            dtype="float32",
        )
        for l in range(num_layers)
    ]

    anchors = np.expand_dims(anchors, 0)
    anchor_maxes = anchors / 2.0
    anchor_mins = -anchor_maxes
    valid_mask = boxes_wh[..., 0] > 0

    for b in range(m):
        wh = boxes_wh[b, valid_mask[b]]
        if len(wh) == 0:
            continue
        wh = np.expand_dims(wh, -2)
        box_maxes = wh / 2.0
        box_mins = -box_maxes

        intersect_mins = np.maximum(box_mins, anchor_mins)
        intersect_maxes = np.minimum(box_maxes, anchor_maxes)
        intersect_wh = np.maximum(intersect_maxes - intersect_mins, 0.0)
        intersect_area = intersect_wh[..., 0] * intersect_wh[..., 1]
        box_area = wh[..., 0] * wh[..., 1]
        anchor_area = anchors[..., 0] * anchors[..., 1]
        iou = intersect_area / (box_area + anchor_area - intersect_area)

        best_anchor = np.argmax(iou, axis=-1)

        for t, n in enumerate(best_anchor):
            for l in range(num_layers):
                if n in anchor_mask[l]:
                    i = np.floor(true_boxes[b, t, 0] * grid_shapes[l][1]).astype("int32")
                    j = np.floor(true_boxes[b, t, 1] * grid_shapes[l][0]).astype("int32")
                    k = anchor_mask[l].index(n)
                    c = true_boxes[b, t, 4].astype("int32")
                    y_true[l][b, j, i, k, 0:4] = true_boxes[b, t, 0:4]
                    y_true[l][b, j, i, k, 4] = 1
                    y_true[l][b, j, i, k, 5 + c] = 1

    return y_true


def box_iou(b1, b2):
    """Return IoU tensor.

    b1: (..., 4) xywh
    b2: (j, 4) xywh
    """
    b1 = tf.expand_dims(b1, -2)
    b1_xy = b1[..., :2]
    b1_wh = b1[..., 2:4]
    b1_wh_half = b1_wh / 2.0
    b1_mins = b1_xy - b1_wh_half
    b1_maxes = b1_xy + b1_wh_half

    b2 = tf.expand_dims(b2, 0)
    b2_xy = b2[..., :2]
    b2_wh = b2[..., 2:4]
    b2_wh_half = b2_wh / 2.0
    b2_mins = b2_xy - b2_wh_half
    b2_maxes = b2_xy + b2_wh_half

    intersect_mins = tf.maximum(b1_mins, b2_mins)
    intersect_maxes = tf.minimum(b1_maxes, b2_maxes)
    intersect_wh = tf.maximum(intersect_maxes - intersect_mins, 0.0)
    intersect_area = intersect_wh[..., 0] * intersect_wh[..., 1]
    b1_area = b1_wh[..., 0] * b1_wh[..., 1]
    b2_area = b2_wh[..., 0] * b2_wh[..., 1]
    iou = intersect_area / (b1_area + b2_area - intersect_area)
    return iou


def yolo_loss(args, anchors, num_classes, ignore_thresh=0.5, print_loss=False):
    """Return yolo_loss tensor (TF compatible)."""
    num_layers = len(anchors) // 3
    yolo_outputs = args[:num_layers]
    y_true = args[num_layers:]

    anchor_mask = (
        [[6, 7, 8], [3, 4, 5], [0, 1, 2]]
        if num_layers == 3
        else [[3, 4, 5], [1, 2, 3]]
    )

    input_shape = tf.cast(tf.shape(yolo_outputs[0])[1:3] * 32, y_true[0].dtype)
    grid_shapes = [tf.cast(tf.shape(yolo_outputs[l])[1:3], y_true[0].dtype) for l in range(num_layers)]

    loss = tf.constant(0.0, dtype=tf.float32)
    m = tf.shape(yolo_outputs[0])[0]
    mf = tf.cast(m, tf.float32)

    for l in range(num_layers):
        object_mask = y_true[l][..., 4:5]
        true_class_probs = y_true[l][..., 5:]

        grid, raw_pred, pred_xy, pred_wh = yolo_head(
            yolo_outputs[l],
            anchors[anchor_mask[l]],
            num_classes,
            input_shape,
            calc_loss=True,
        )
        pred_box = tf.concat([pred_xy, pred_wh], axis=-1)

        raw_true_xy = y_true[l][..., :2] * grid_shapes[l][::-1] - grid
        raw_true_wh = tf.math.log(
            (y_true[l][..., 2:4] / anchors[anchor_mask[l]]) * input_shape[::-1]
        )
        raw_true_wh = tf.where(object_mask > 0.0, raw_true_wh, tf.zeros_like(raw_true_wh))
        box_loss_scale = 2.0 - y_true[l][..., 2:3] * y_true[l][..., 3:4]

        # Ignore mask per batch
        object_mask_bool = tf.cast(object_mask, tf.bool)

        def loop_body(b, ta):
            true_box = tf.boolean_mask(y_true[l][b, ..., 0:4], object_mask_bool[b, ..., 0])
            iou = box_iou(pred_box[b], true_box)

            best_iou = tf.reduce_max(iou, axis=-1)
            ignore = tf.cast(best_iou < ignore_thresh, true_box.dtype)
            ta = ta.write(b, ignore)
            return b + 1, ta

        ta = tf.TensorArray(y_true[0].dtype, size=0, dynamic_size=True)
        _, ta = tf.while_loop(lambda b, *_: b < m, loop_body, [0, ta])
        ignore_mask = ta.stack()
        ignore_mask = tf.expand_dims(ignore_mask, -1)

        # Use TF's binary cross entropy from logits
        bce = tf.keras.backend.binary_crossentropy

        xy_loss = object_mask * box_loss_scale * bce(raw_true_xy, raw_pred[..., 0:2], from_logits=True)
        wh_loss = object_mask * box_loss_scale * 0.5 * tf.square(raw_true_wh - raw_pred[..., 2:4])

        conf_loss_obj = object_mask * bce(object_mask, raw_pred[..., 4:5], from_logits=True)
        conf_loss_noobj = (1.0 - object_mask) * bce(object_mask, raw_pred[..., 4:5], from_logits=True) * ignore_mask
        confidence_loss = conf_loss_obj + conf_loss_noobj

        class_loss = object_mask * bce(true_class_probs, raw_pred[..., 5:], from_logits=True)

        xy_loss = tf.reduce_sum(xy_loss) / mf
        wh_loss = tf.reduce_sum(wh_loss) / mf
        confidence_loss = tf.reduce_sum(confidence_loss) / mf
        class_loss = tf.reduce_sum(class_loss) / mf

        loss = loss + xy_loss + wh_loss + confidence_loss + class_loss

        if print_loss:
            # tf.print works in graph/eager
            tf.print("loss:", loss, xy_loss, wh_loss, confidence_loss, class_loss, tf.reduce_sum(ignore_mask))

    return loss

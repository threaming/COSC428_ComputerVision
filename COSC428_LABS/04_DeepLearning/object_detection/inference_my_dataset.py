import numpy as np
import torch
import cv2
import matplotlib.pyplot as plt
import json

from detectron2.config import get_cfg
from detectron2.data import MetadataCatalog
from detectron2.data.datasets import register_coco_instances
from detectron2.engine.defaults import DefaultPredictor
from detectron2.utils.visualizer import Visualizer


def get_top_x_predictions(predictions, x):
    predictions["instances"] = predictions["instances"][:x]

def get_detections_above_confidence(predictions, conf=0.9):
    scores = predictions["instances"].scores.cpu().numpy()
    x = len(scores) - np.searchsorted(scores[::-1], conf)
    get_top_x_predictions(predictions, x)

def class_id2label(ids, classes):
    labels = []
    for id in ids:
        labels.append(classes[id])
    return labels

def combine_label_and_score(labels, scores):
    view_strings = []
    for label, score in zip(labels, scores):
        view_str = f"{label} - {score:.3f}"
        view_strings.append(view_str)
    return view_strings

def get_categories(json_path):
    with open(json_path, 'r') as file:
        coco = json.load(file)
    categories_list = ["" for i in range(len(coco["categories"]))]
    for category in coco["categories"]:
        categories_list[category["id"]-1] = category["name"]
    return categories_list


if __name__ == "__main__":
    image = cv2.imread("./images/inputs/card.png")

    cfg = get_cfg()
    cfg.merge_from_file("./object_detection/configs/train_rcnn_fpn.yaml")
    cfg.MODEL.WEIGHTS = "./object_detection/logs/model_final.pth"

    metadata = MetadataCatalog.get(cfg.DATASETS.TEST[0])
    metadata.thing_classes = get_categories("./object_detection/labels/cards_test.json")

    predictor = DefaultPredictor(cfg)
    predictions = predictor(image)
    predictions["instances"] = predictions["instances"].to("cpu")

    get_top_x_predictions(predictions, 1)
    # get_detections_above_confidence(predictions, 0.8)

    labels = class_id2label(predictions["instances"].pred_classes, metadata.thing_classes)
    scores = predictions["instances"].scores.numpy()
    view_strings = combine_label_and_score(labels, scores)

    visualizer = Visualizer(image)
    visualized_detection = visualizer.overlay_instances(
        boxes=predictions["instances"].pred_boxes,
        labels=view_strings,
        ).get_image()

    plt.imshow(cv2.cvtColor(visualized_detection, cv2.COLOR_BGR2RGB))
    plt.show()
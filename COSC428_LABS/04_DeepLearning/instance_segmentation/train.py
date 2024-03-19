import numpy as np
import torch
import cv2
import matplotlib.pyplot as plt

from detectron2.checkpoint import DetectionCheckpointer
from detectron2.config import get_cfg
from detectron2.data import MetadataCatalog
from detectron2.data.datasets import register_coco_instances
from detectron2.engine import DefaultTrainer, default_argument_parser, default_setup, hooks, launch

from detectron2.evaluation import COCOEvaluator


class Trainer(DefaultTrainer):
    @classmethod
    def build_evaluator(cls, cfg, dataset_name, output_folder=None):
        return COCOEvaluator("my_dataset_val", output_dir=cfg.OUTPUT_DIR)
    
    # Online Agumentation
    #@classmethod
    #def build_train_loader(cls, cfg):
    #    mapper = DatasetMapper(cfg, is_train=True, augmentations=ADD_AUGMENTATION_LIST_HERE__ )
    #    return build_detection_train_loader(cfg, mapper=mapper)


if __name__ == "__main__":

    cfg = get_cfg()
    cfg.merge_from_file("./instance_segmentation/configs/train_mask_rcnn_fpn.yaml")

    register_coco_instances(
        name="my_dataset_train",
        metadata={},
        json_file="./instance_segmentation/labels/cards_train.json",
        image_root="./data/card_train")
    register_coco_instances(
        name="my_dataset_val",
        metadata={},
        json_file="./instance_segmentation/labels/cards_test.json",
        image_root="./data/card_test")

    trainer = Trainer(cfg)
    trainer.resume_or_load(resume=False)
    trainer.train()

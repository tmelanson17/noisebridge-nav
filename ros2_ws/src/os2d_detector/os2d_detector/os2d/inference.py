import torch
import os
import glob
import torchvision.transforms as transforms
from PIL import Image

from .config import cfg
from .structures.bounding_box import filter_bbox, convert_xyxy_bbox_to_relative_coords
from .structures.feature_map import FeatureMapSize
from .modeling.model import build_os2d_from_config
from .utils import set_random_seed, get_image_size_after_resize_preserving_aspect_ratio

DIR_PATH = os.path.dirname(os.path.realpath(__file__))


class OS2DDetector:
    def __init__(self):
        cfg.visualization.eval.max_detections = 30
        cfg.visualization.eval.score_threshold = 0.60
        cfg.is_cuda = torch.cuda.is_available()

        if cfg.is_cuda:
            torch.backends.cudnn.benchmark = True

        # random seed
        set_random_seed(cfg.random_seed, cfg.is_cuda)

        # Model
        cfg.init.model = os.path.join(DIR_PATH, "models/os2d_v2-train.pth")
        (
            self.net,
            self.box_coder,
            self.criterion,
            self.img_normalization,
            self.optimizer_state,
        ) = build_os2d_from_config(cfg)

        self.transform_image = transforms.Compose(
            [
                transforms.ToTensor(),
                transforms.Normalize(
                    self.img_normalization["mean"], self.img_normalization["std"]
                ),
            ]
        )
        self.query_images, self.class_ids = self._get_query_images()

    def _get_query_images(self):
        query_images = []
        class_ids = []
        counter = 0
        for img_path in glob.glob(os.path.join(DIR_PATH, "query/*.jpg")):
            with open(img_path, "rb") as fp:
                img = self.preprocess_image(
                    Image.open(fp), cfg.model.class_image_size, cfg.is_cuda
                )
                query_images.append(img)
                class_ids.append(counter)
                counter += 1
        return query_images, class_ids

    def preprocess_image(self, image, target_size, cuda=True):
        h, w = get_image_size_after_resize_preserving_aspect_ratio(
            h=image.size[1], w=image.size[0], target_size=target_size
        )
        image = image.resize((w, h))
        image = self.transform_image(image)
        if cuda:
            image = image.cuda()
        return image

    def predict(self, input_image):
        input_processed = self.preprocess_image(
            input_image, 1500, cfg.is_cuda
        ).unsqueeze(0)

        input_h, input_w = input_processed.size()[-2:]

        with torch.no_grad():
            (
                loc_prediction_batch,
                class_prediction_batch,
                _,
                _,
                transform_corners_batch,
            ) = self.net(images=input_processed, class_images=self.query_images)

        image_loc_scores_pyramid = [loc_prediction_batch[0]]
        image_class_scores_pyramid = [class_prediction_batch[0]]
        img_size_pyramid = [FeatureMapSize(img=input_processed)]
        transform_corners_pyramid = [transform_corners_batch[0]]

        boxes = self.box_coder.decode_pyramid(
            image_loc_scores_pyramid,
            image_class_scores_pyramid,
            img_size_pyramid,
            self.class_ids,
            nms_iou_threshold=cfg.eval.nms_iou_threshold,
            nms_score_threshold=cfg.eval.nms_score_threshold,
            transform_corners_pyramid=transform_corners_pyramid,
        )

        # remove some fields to lighten visualization
        boxes.remove_field("default_boxes")

        scores, boxes_coords = filter_bbox(
            boxes,
            cfg.visualization.eval.score_threshold,
            cfg.visualization.eval.max_detections,
        )
        boxes_coords = [
            convert_xyxy_bbox_to_relative_coords(
                box, im_height=input_h, im_width=input_w
            )
            for box in boxes_coords.tolist()
        ]
        return {"scores": scores.tolist(), "bboxes": boxes_coords}

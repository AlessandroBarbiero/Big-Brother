from det3d.torchie.apis import (
    build_optimizer,
    get_root_logger,
    init_dist,
    set_random_seed,
    train_detector,
)
from det3d.torchie import Config
from det3d.models import build_detector
from det3d.datasets import build_dataset
from det3d import __version__
import yaml
import torch
import numpy as np
import pdb
import sys
import os
import json
import argparse
import warnings
warnings.filterwarnings("ignore")
# sys.path.append('/home/lin1.sun/3D-detection/Det3D1.3')


def parse_args():
    parser = argparse.ArgumentParser(description="Train a detector")
    parser.add_argument("config", help="train config file path")
    parser.add_argument("--work_dir", help="the dir to save logs and models")
    parser.add_argument("--resume_from", help="the checkpoint file to resume from")
    parser.add_argument(
        "--validate",
        action="store_true",
        help="whether to evaluate the checkpoint during training",
    )
    parser.add_argument(
        "--gpus",
        type=int,
        default=1,
        help="number of gpus to use " "(only applicable to non-distributed training)",
    )
    parser.add_argument("--seed", type=int, default=None, help="random seed")
    parser.add_argument(
        "--launcher",
        choices=["none", "pytorch", "slurm", "mpi"],
        default="none",
        help="job launcher",
    )
    parser.add_argument("--local_rank", type=int, default=0)
    parser.add_argument(
        "--autoscale-lr",
        action="store_true",
        help="automatically scale lr with the number of gpus",
    )
    args = parser.parse_args()
    if "LOCAL_RANK" not in os.environ:
        os.environ["LOCAL_RANK"] = str(args.local_rank)

    return args


def main():

    # torch.manual_seed(0)
    # torch.backends.cudnn.deterministic = True
    # torch.backends.cudnn.benchmark = True
    # np.random.seed(0)
    args = parse_args()

    cfg = Config.fromfile(args.config)
    cfg.local_rank = args.local_rank

    # update local according to CLI args
    if args.work_dir is not None:
        cfg.work_dir = args.work_dir
    if args.resume_from is not None:
        cfg.resume_from = args.resume_from

    distributed = False
    logger = get_root_logger(cfg.log_level)
    if "WORLD_SIZE" in os.environ:
        distributed = int(os.environ["WORLD_SIZE"]) > 1

    if distributed:
        torch.cuda.set_device(args.local_rank)
        torch.distributed.init_process_group(backend="nccl", init_method="env://")

        cfg.gpus = torch.distributed.get_world_size()
    # assert torch.backends.cudnn.enabled, "Amp requires cudnn backend to be enabled."
    if args.autoscale_lr:
        cfg.lr_config.lr_max = cfg.lr_config.lr_max * cfg.gpus

    # init logger before other steps
    logger.info("Distributed training: {}".format(distributed))
    logger.info(f"torch.backends.cudnn.benchmark: {torch.backends.cudnn.benchmark}")

    # set random seeds
    if args.seed is not None:
        logger.info("Set random seed to {}".format(args.seed))
        set_random_seed(args.seed)
    model = build_detector(cfg.model, train_cfg=cfg.train_cfg, test_cfg=cfg.test_cfg)

    datasets = [build_dataset(cfg.data.train)]

    if len(cfg.workflow) == 2:
        datasets.append(build_dataset(cfg.data.val))

    # Train_set = datasets[0]
    # Train_set.resample_infos('/home/dev/Projects/data/nuscenes/v1.0-trainval/infos_train_10sweeps_repeat_withvelo.pkl',
    #                          '/home/dev/Projects/data/nuscenes/v1.0-trainval/infos_val_10sweeps_repeat_withvelo.pkl', 10)

    if cfg.checkpoint_config is not None:
        # save det3d version, config file content and class names in
        # checkpoints as meta data
        cfg.checkpoint_config.meta = dict(
            det3d_version=__version__, config=cfg.text, CLASSES=datasets[0].CLASSES
        )

    # add an attribute for visualization convenience
    model.CLASSES = datasets[0].CLASSES
    train_detector(
        model,
        datasets,
        cfg,
        distributed=distributed,
        validate=args.validate,
        logger=logger,
    )


if __name__ == "__main__":
    main()

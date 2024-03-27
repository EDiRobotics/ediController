import argparse
import os
import numpy as np
import torch
from torch.utils.data import Dataset
from torchvision.io import encode_jpeg, decode_jpeg, encode_png, decode_png
from PIL import Image
import pandas as pd
import sys
import pdb

sys.path.append(".")
from edi_data_collection.lmdb_interface import load_keys_from_lmdb, load_episode_from_lmdb, load_step_from_lmdb


def generate_dataset_config(root_dir, csv_file_name):
    import csv
    lmdb_paths = []
    for subdir, dirs, files in os.walk(root_dir):
        if "data.mdb" in files:
            lmdb_paths.append(subdir)
    csv_data = []
    for lmdb_dir in lmdb_paths:
        keys = load_keys_from_lmdb(lmdb_dir)
        for episode_key in keys:
            results = load_episode_from_lmdb(lmdb_dir, episode_key)
            if results is None:
                print(f"Warning: {lmdb_dir}:{episode_key} is None")
                continue
            max_step = results["max_step"]
            relative_lmdb_dir = os.path.relpath(lmdb_dir, root_dir)
            key_to_add_csv = (episode_key, relative_lmdb_dir, max_step)
            csv_data.append(key_to_add_csv)
    # Write data to CSV file
    with open(csv_file_name, mode='w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(['episode_key', 'lmdb_dir', 'max_step'])
        writer.writerows(csv_data)


class EpisodicLMDBDatasetV2(Dataset):
    def __init__(self, root_dir, csv_file):
        self.root_dir = root_dir
        self.index_to_lmdb = []
        df = pd.read_csv(csv_file)
        for _, row in df.iterrows():
            self.index_to_lmdb.append((row['lmdb_dir'], row['episode_key']))

    def __len__(self):
        return len(self.index_to_lmdb)

    def __getitem__(self, idx):
        lmdb_dir, episode_key = self.index_to_lmdb[idx]
        lmdb_dir = os.path.join(self.root_dir, lmdb_dir)
        return load_episode_from_lmdb(lmdb_dir, episode_key)


class StepLMDBDatasetV2(Dataset):
    def __init__(self, csv_file):
        if not os.path.exists(csv_file):
            raise FileNotFoundError
        root_dir = os.path.dirname(csv_file)
        self.root_dir = root_dir
        self.episode_to_lmdb = {}
        self.index_to_episode_step = []
        df = pd.read_csv(csv_file)
        for _, row in df.iterrows():
            episode_key = row['episode_key']
            lmdb_dir = row['lmdb_dir']
            max_step = int(row['max_step'])
            self.episode_to_lmdb[episode_key] = lmdb_dir
            for step in range(max_step):
                self.index_to_episode_step.append((episode_key, step))

    def __len__(self):
        return len(self.index_to_episode_step)

    def __getitem__(self, idx):
        episode_key, step = self.index_to_episode_step[idx]
        lmdb_dir = self.episode_to_lmdb[episode_key]
        lmdb_dir = os.path.join(self.root_dir, lmdb_dir)
        episode_data = load_step_from_lmdb(lmdb_dir, episode_key, step)
        if episode_data:
            obs = episode_data["records"]["obs"]
            action = episode_data["records"]["action"]
            inst = episode_data["instruct"]
            return obs, action, inst
        else:
            if not os.path.exists(lmdb_dir):
                raise Exception(f"Cannot load episode {episode_key}: path {lmdb_dir} not exist!")
            raise Exception(f"Cannot load step {step} from {episode_key} in {lmdb_dir}!")

    @property
    def episode_num(self):
        return len(self.episode_to_lmdb)


if __name__ == "__main__":
    """
    Test load from lmdb and replay
    """

    parser = argparse.ArgumentParser(description='LMDB Dataset Loader Test',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--path', type=str, default="./dataset/",
                        help='Path can be a rosbag file or a directory (recursively search).')
    parser.add_argument('--csv_name', type=str, default="test.csv",
                        help='Path can be a rosbag file or a directory (recursively search).')
    parser.add_argument('--generate_config', '-g', action='store_true',
                        help='Replay action')
    args = parser.parse_args()
    dataset_directory: str = args.path
    csv_name: str = args.csv_name
    generate_config: bool = args.generate_config
    if generate_config:
        print("Generate dataset config")
        generate_dataset_config(dataset_directory, os.path.join(dataset_directory, csv_name))

    print("Preparing dataset")
    dataset = StepLMDBDatasetV2(os.path.join(dataset_directory, csv_name))
    import pdb

    pdb.set_trace()

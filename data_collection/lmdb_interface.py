import argparse
import os
import traceback
from pickle import dumps, loads
import lmdb
import numpy as np
import torch
from torch.utils.data import Dataset
from torchvision.io import encode_jpeg, decode_jpeg, encode_png, decode_png


def save_to_lmdb(results, lmdb_directory):
    """
    results:
    {
        "episode": 20231219153942001,
        "base_timestamp": 1703088159.7284524,
        "instruct": "reach red",
        "params": {...},
        "records": [
            {
                "action": [23.0, -113.0, -102.0, -54.0, 90.0, -170.0],
                "obs": {
                    "status": {"s1": 0},
                    "sensors": {"/camera1": np.ndarray}
                },
                "obs_timestamp": 1703088259.7284524,
                "action_timestamp": 1703088359.7284524,
                "action_mode": "demo",
            },
            # ... Additional records
        ],
        "max_step": 200
    }
    :param results:
    :return:
    """
    if lmdb_directory is None:
        return False

    if not os.path.exists(lmdb_directory):
        os.makedirs(lmdb_directory)
    assert isinstance(results, dict)
    results.setdefault("instruct", "")
    try:
        env = lmdb.open(lmdb_directory, map_size=int(1e12))  # Adjust map size as needed
        with env.begin(write=True) as txn:
            # Save episode information
            episode_key = f'episode_{results["episode"]}_'
            txn.put(f'{episode_key}base_timestamp'.encode(), dumps(results['base_timestamp']))
            txn.put(f'{episode_key}max_step'.encode(), dumps(results['max_step']))
            txn.put(f'{episode_key}instruct'.encode(), dumps(results['instruct']))
            if results['max_step'] > 0:
                cameras = [camera for camera in results['records'][0]['obs']['sensors'].keys()]
            else:
                cameras = []
            txn.put(f'{episode_key}_cameras'.encode(), dumps(cameras))
            txn.put(f'{episode_key}max_step'.encode(), dumps(results['max_step']))

            # Iterate through the records in the episode and save each one
            for i, record in enumerate(results['records']):
                record_key_prefix = f'{episode_key}record_{i}_'

                # Save action
                txn.put(f'{record_key_prefix}action'.encode(), dumps(record['action']))

                # Save observation status
                txn.put(f'{record_key_prefix}obs_status'.encode(), dumps(record['obs']['status']))

                # Save images from observations
                for camera, image in record['obs']['sensors'].items():
                    # Convert image to numpy array if it's not already
                    image_np = np.array(image)

                    # If image has 3 channels (RGB), encode it as JPEG
                    if image_np.shape[2] == 3:  # Assuming the shape is in HxWxC format
                        image_encoded = encode_jpeg(torch.from_numpy(image_np).permute(2, 0, 1)).numpy()
                        txn.put(f'{record_key_prefix}obs_sensor_{camera}'.encode(), dumps(image_encoded))
                    else:
                        # For non-RGB images, store them as numpy arrays
                        txn.put(f'{record_key_prefix}obs_sensor_{camera}'.encode(), dumps(image_np))
                # Save observation timestamp
                txn.put(f'{record_key_prefix}obs_timestamp'.encode(), dumps(record['obs_timestamp']))

                # Save action timestamp
                txn.put(f'{record_key_prefix}action_timestamp'.encode(), dumps(record['action_timestamp']))

                # Save action mode
                txn.put(f'{record_key_prefix}action_mode'.encode(), dumps(record['action_mode']))
            try:
                key_list = loads(txn.get(f'_keys'.encode()))
            except:
                key_list = []
            assert isinstance(key_list, list)
            if episode_key not in key_list:
                key_list.append(episode_key)
            txn.put(f'_keys'.encode(), dumps(key_list))
        env.close()
        return True
    except:
        traceback.print_exc()
        return False


def load_keys_from_lmdb(lmdb_directory):
    try:
        env = lmdb.open(lmdb_directory, map_size=int(1e12))  # Adjust map size as needed
        with env.begin(write=True) as txn:
            try:
                key_list = loads(txn.get(f'_keys'.encode()))
            except:
                key_list = []
            assert isinstance(key_list, list)
        env.close()
        return key_list
    except:
        traceback.print_exc()
        return None


def load_episode_from_lmdb(lmdb_directory, episode_key):
    """
    Load records from an LMDB database for a specific episode.

    :param lmdb_directory: The directory where the LMDB database is stored.
    :param episode_key: The key for the episode to be loaded.
    :return: A dictionary with the loaded data for the specified episode.
    """
    if not os.path.exists(lmdb_directory):
        print(f"Data directory {lmdb_directory} does not exist.")
        return None

    try:
        env = lmdb.open(lmdb_directory, readonly=True)
        results = {"records": []}

        with env.begin() as txn:
            # Load base timestamp and max step
            results["base_timestamp"] = loads(txn.get(f'{episode_key}base_timestamp'.encode()))
            results["max_step"] = loads(txn.get(f'{episode_key}max_step'.encode()))
            results['instruct'] = loads(txn.get(f'{episode_key}instruct'.encode()))
            results["episode"] = episode_key.split('_')[1]

            cameras = loads(txn.get(f'{episode_key}_cameras'.encode()))
            max_step = results["max_step"]
            # Iterate through records
            for i in range(max_step):
                record_key_prefix = f'{episode_key}record_{i}_'
                action_key = f'{record_key_prefix}action'.encode()

                if txn.get(action_key) is None:
                    break  # Exit loop if no more records

                record = {
                    "action": loads(txn.get(action_key)),
                    "obs": {
                        "status": loads(txn.get(f'{record_key_prefix}obs_status'.encode())),
                        "sensors": {}
                    },
                    "obs_timestamp": loads(txn.get(f'{record_key_prefix}obs_timestamp'.encode())),
                    "action_timestamp": loads(txn.get(f'{record_key_prefix}action_timestamp'.encode())),
                    "action_mode": loads(txn.get(f'{record_key_prefix}action_mode'.encode()))
                }

                for camera in cameras:
                    camera_key = f'{record_key_prefix}obs_sensor_{camera}'.encode()
                    if txn.get(camera_key) is None:
                        break  # Exit loop if no more sensors

                    image_data = loads(txn.get(camera_key))
                    if isinstance(image_data, np.ndarray):
                        record["obs"]["sensors"][camera] = decode_jpeg(torch.from_numpy(image_data)).numpy()
                    else:  # RGB images
                        record["obs"]["sensors"][camera] = image_data
                results["records"].append(record)

        return results
    except Exception as e:
        print(f"Error loading data: {e}")
        return None


class EpisodicLMDBDataset(Dataset):
    def __init__(self, root_dir):
        self.lmdb_paths = []
        self.keys_map = []
        self.index_to_lmdb = []

        for subdir, dirs, files in os.walk(root_dir):
            if "data.mdb" in files:
                self.lmdb_paths.append(subdir)

        for lmdb_dir in self.lmdb_paths:
            keys = load_keys_from_lmdb(lmdb_dir)
            if keys:
                self.keys_map.append(keys)
                self.index_to_lmdb.extend([(lmdb_dir, key) for key in keys])
            else:
                print(f"Warning: No keys found in {lmdb_dir}")

    def __len__(self):
        return len(self.index_to_lmdb)

    def __getitem__(self, idx):
        lmdb_dir, episode_key = self.index_to_lmdb[idx]
        return load_episode_from_lmdb(lmdb_dir, episode_key)


class StepLMDBDataset(Dataset):
    def __init__(self, root_dir):
        self.lmdb_paths = []
        self.episode_to_lmdb = {}
        self.index_to_episode_step = []

        for subdir, dirs, files in os.walk(root_dir):
            if "data.mdb" in files:
                self.lmdb_paths.append(subdir)

        for lmdb_dir in self.lmdb_paths:
            keys = load_keys_from_lmdb(lmdb_dir)
            if keys:
                for key in keys:
                    self.episode_to_lmdb[key] = lmdb_dir
                    episode_data = load_episode_from_lmdb(lmdb_dir, key)
                    if episode_data:
                        for step in range(episode_data["max_step"]):
                            self.index_to_episode_step.append((key, step))
            else:
                print(f"Warning: No keys found in {lmdb_dir}")

    def __len__(self):
        return len(self.index_to_episode_step)

    def __getitem__(self, idx):
        episode_key, step = self.index_to_episode_step[idx]
        lmdb_dir = self.episode_to_lmdb[episode_key]
        episode_data = load_episode_from_lmdb(lmdb_dir, episode_key)
        if episode_data:
            obs = episode_data["records"][step]["obs"]
            action = episode_data["records"][step]["action"]
            return obs, action
        else:
            return None, None


if __name__ == "__main__":
    """
    Test load from lmdb and replay
    """
    import rospy
    import sys

    sys.path.append(".")
    rospy.init_node('lmdb_loader')

    parser = argparse.ArgumentParser(description='LMDB Dataset Loader Test',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--path', type=str, default="./dataset/test",
                        help='Path can be a rosbag file or a directory (recursively search).')
    args = parser.parse_args()
    lmdb_directory: str = args.path
    keys = load_keys_from_lmdb(lmdb_directory)
    all_results = [(key, load_episode_from_lmdb(lmdb_directory, key)) for key in keys]
    print(all_results)

    dataset = EpisodicLMDBDataset(lmdb_directory)
    dataset = StepLMDBDataset(lmdb_directory)

    rospy.loginfo(f"----- Starting to replay -----")
    from data_collection.replay import display_sensor_data

    for i, (full_file_name, results) in enumerate(all_results):
        rospy.loginfo(f"Start to replay {full_file_name}...")
        display_sensor_data(results)
    rospy.loginfo(f"Finish all replay tasks...")

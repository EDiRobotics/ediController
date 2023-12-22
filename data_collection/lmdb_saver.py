# TODO: Save data to LMDB Datasets

import os
from pickle import dumps, loads
import lmdb
import numpy as np
import torch
from torchvision.io import encode_jpeg, decode_jpeg, encode_png, decode_png


def save_to_lmdb(records, data_dir):
    """
    records:
    {
        "episode": 20231219153942001,
        "base_timestamp": 1703088159.7284524,
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
    if data_dir is None:
        return False

    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

    try:
        env = lmdb.open(data_dir, map_size=int(1e12))  # Adjust map size as needed
        with env.begin(write=True) as txn:
            # Save episode information
            episode_key = f'episode_{records["episode"]}_'
            txn.put(f'{episode_key}base_timestamp'.encode(), dumps(records['base_timestamp']))
            txn.put(f'{episode_key}max_step'.encode(), dumps(records['max_step']))
            # Iterate through the records in the episode and save each one
            for i, record in enumerate(records['records']):
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

        env.close()
        return True
    except:
        return False


def load_from_lmdb(data_dir, episode_key):
    # TODO: NOT READY
    """
    Load records from an LMDB database for a specific episode.

    :param data_dir: The directory where the LMDB database is stored.
    :param episode_key: The key for the episode to be loaded.
    :return: A dictionary with the loaded data for the specified episode.
    """
    if not os.path.exists(data_dir):
        print(f"Data directory {data_dir} does not exist.")
        return None

    try:
        env = lmdb.open(data_dir, readonly=True)
        records = {"records": []}

        with env.begin() as txn:
            # Load base timestamp and max step
            records["base_timestamp"] = loads(txn.get(f'{episode_key}base_timestamp'.encode()))
            records["max_step"] = loads(txn.get(f'{episode_key}max_step'.encode()))
            records["episode"] = episode_key.split('_')[1]

            # Iterate through records
            i = 0
            while True:
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

                # Load sensor images
                j = 0
                while True:
                    camera_key = f'{record_key_prefix}obs_sensor_/camera{j}'.encode()
                    if txn.get(camera_key) is None:
                        break  # Exit loop if no more sensors

                    image_data = loads(txn.get(camera_key))
                    if isinstance(image_data, np.ndarray):  # Non-RGB images
                        record["obs"]["sensors"][f"/camera{j}"] = image_data
                    else:  # RGB images
                        record["obs"]["sensors"][f"/camera{j}"] = decode_jpeg(torch.from_numpy(image_data)).numpy()

                    j += 1

                records["records"].append(record)
                i += 1

        return records
    except Exception as e:
        print(f"Error loading data: {e}")
        return None

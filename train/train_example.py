import pdb

import numpy as np
from edi_data_collection.data import StepLMDBDatasetV2, SuitableStepLMDBDatasetV2
from edi_gym.edi_env import EdiEnv
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, random_split
from torch.optim.lr_scheduler import CosineAnnealingLR
import os
import cv2

dataset_directory = "dataset/"
csv_file = "27.csv"


class BehaviorCloningModel(nn.Module):
    def __init__(self, input_size=6, output_size=6, hidden_size=64, layer_num=3):
        super(BehaviorCloningModel, self).__init__()

        layers = []
        layers.append(nn.Linear(input_size, hidden_size))
        layers.append(nn.ReLU())
        for _ in range(layer_num - 1):
            layers.append(nn.Linear(hidden_size, hidden_size))
            layers.append(nn.ReLU())
        layers.append(nn.Linear(hidden_size, output_size))
        self.model = nn.Sequential(*layers)

    def forward(self, x):
        # Padding zeros to the last dimension
        res = self.model(x)
        if len(x.shape) == 1:
            zeros = torch.zeros(1).float()
        else:
            zeros = torch.zeros(x.size(0), 1).float()

        res = torch.cat((res, zeros), dim=-1)
        return res


def evaluate(model, env, max_step=200):
    model.eval()
    obs = env.reset()
    sensor_data = obs["sensors"]
    obs["status"] = np.array(obs["status"]["jt_cur_pos"][0])
    obs_images = torch.tensor(list(obs["sensors"].values())[0]).float()
    obs_status = torch.tensor(obs["status"]).float()
    obs = obs_status
    obs = obs.to(device).float()

    for step in range(max_step):
        action = model(obs)
        obs, _, _, info = env.step(action)
        sensor_data = obs["sensors"]
        obs["status"] = np.array(obs["status"]["jt_cur_pos"][0])
        obs_images = torch.tensor(list(obs["sensors"].values())[0]).float()
        obs_status = torch.tensor(obs["status"]).float()
        obs = obs_status
        obs = obs.to(device).float()
        # for camera_name, image in sensor_data.items():
        #     cv2.imshow(camera_name, image)
        # cv2.waitKey(1)


max_epochs = 200
learning_rate = 5e-2
batch_size = 128
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Instantiate the model
model = BehaviorCloningModel().to(device)

# Define loss function and optimizer
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=learning_rate)
scheduler = CosineAnnealingLR(optimizer, T_max=max_epochs)

dataset = SuitableStepLMDBDatasetV2(os.path.join(dataset_directory, csv_file))
env = None

# Split the dataset into train and test
train_size = int(0.8 * len(dataset))
test_size = len(dataset) - train_size
train_dataset, test_dataset = random_split(dataset, [train_size, test_size])
print(f"train_dataset len {len(train_dataset)}")
print(f"test_dataset len {len(test_dataset)}")
# Define data loaders
train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False)

for epoch in range(max_epochs):
    model.train()
    running_train_loss = 0.0
    for batch_idx, (obs, action, instruct) in enumerate(train_loader):
        obs_images = list(obs["sensors"].values())[0].float()
        obs_status = obs["status"].float()
        obs = obs_status
        obs, action = obs.to(device).float(), action.to(device).float()

        # Zero the parameter gradients
        optimizer.zero_grad()

        # Forward pass
        outputs = model(obs)

        # Compute the loss
        train_loss = criterion(outputs, action)

        # Backward pass
        train_loss.backward()
        optimizer.step()

        running_train_loss += train_loss.item()

    scheduler.step()
    print(f"Epoch {epoch + 1}/{max_epochs}, Train Loss: {running_train_loss / len(train_loader)}")

    # Testing loop
    model.eval()
    test_loss = 0.0
    with torch.no_grad():
        for obs, action, instruct in test_loader:
            obs_images = list(obs["sensors"].values())[0].float()
            obs_status = obs["status"].float()
            obs = obs_status
            obs, action = obs.to(device).float(), action.to(device).float()
            outputs = model(obs)
            test_loss += criterion(outputs, action).item()

    print(f"Test Loss: {test_loss / len(test_loader)}")

    # Evaluate the model in the real environment
    if (epoch + 1) % 50 == 0:
        pass
print("start evaluate")
if env is None:
    env = EdiEnv()
evaluate(model, env)
print("finish evaluate")

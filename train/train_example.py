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
from tqdm import tqdm
import cv2

dataset_directory = "dataset/"
csv_file = "28.csv"
env = None


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


def evaluate(model, env, max_step=500):
    input("Press any key to continue evaluate...")
    # 750,-160,300
    if goal_type == "cart":
        cart_coordinates = input("Enter cart coordinates (e.g., '780,-160,220'): ")
        coordinates_list = cart_coordinates.split(',')
        coordinates_float = [float(coord) for coord in coordinates_list]
        goal_cart_tensor = torch.tensor(coordinates_float).to(device).float()
    else:
        raise NotImplementedError

    model.eval()
    print("start reset")
    obs = env.reset()
    print("end reset")

    sensor_data = obs["sensors"]
    obs["status"] = np.array(obs["status"]["jt_cur_pos"][0])
    obs_images = torch.tensor(list(obs["sensors"].values())[0]).float()
    obs_status = torch.tensor(obs["status"]).float()
    obs = obs_status
    obs = obs.to(device).float()

    if goal_type == "cart":
        obs = torch.cat((obs, goal_cart_tensor[:3]), dim=-1)
    else:
        obs = torch.cat((obs, goal_joint_tensor), dim=-1)

    for step in range(max_step):
        if (step + 1) % int(max_step / 10) == 0:
            print(f"Step: {step + 1}")
        action = model(obs)
        obs, _, _, info = env.step(action)
        sensor_data = obs["sensors"]
        obs["status"] = np.array(obs["status"]["jt_cur_pos"][0])
        obs_images = torch.tensor(list(obs["sensors"].values())[0]).float()
        obs_status = torch.tensor(obs["status"]).float()
        obs = obs_status
        obs = obs.to(device).float()

        if goal_type == "cart":
            obs = torch.cat((obs, goal_cart_tensor[:3]), dim=-1)
        else:
            obs = torch.cat((obs, goal_joint_tensor), dim=-1)
        # for camera_name, image in sensor_data.items():
        #     cv2.imshow(camera_name, image)
        # cv2.waitKey(1)

    print(f"Reaching max_step {max_step}")


evaluate_only = True
evaluate_num = 5
max_epochs = 10
used_scale = 1.0
learning_rate = 3e-3
batch_size = 128
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Running on {device}")
# Instantiate the model
goal_type = "cart"
input_size = 6
if goal_type == "cart":
    input_size += 3
elif goal_type == "cart":
    input_size += 3
else:
    pass

model = BehaviorCloningModel(input_size).to(device)
load_success = False
try:
    model.load_state_dict(torch.load('train/latest.pth'))
    print("Load ckpt successfully")
    load_success = True
except:
    print("Unable to load ckpt")

if evaluate_only and load_success:
    print("start evaluate")
    if env is None:
        env = EdiEnv()
    for i in range(evaluate_num):
        evaluate(model, env)
    env.close()
    env = None
    print("finish evaluate")
    exit(0)

# Define loss function and optimizer
criterion = nn.MSELoss(reduction="sum")
optimizer = optim.Adam(model.parameters(), lr=learning_rate)
scheduler = CosineAnnealingLR(optimizer, T_max=max_epochs)

dataset = SuitableStepLMDBDatasetV2(os.path.join(dataset_directory, csv_file))

# Split the dataset into train and test
used_size = int(used_scale * len(dataset))
unused_size = len(dataset) - used_size
dataset, _ = random_split(dataset, [used_size, unused_size])

train_size = int(0.8 * len(dataset))
test_size = len(dataset) - train_size
train_dataset, test_dataset = random_split(dataset, [train_size, test_size])
print(f"train_dataset len {len(train_dataset)}")
print(f"test_dataset len {len(test_dataset)}")
# Define data loaders
train_loader = DataLoader(train_dataset, batch_size=batch_size, pin_memory=True, shuffle=True)
test_loader = DataLoader(test_dataset, batch_size=batch_size, pin_memory=True, shuffle=False)

for epoch in range(max_epochs):
    model.train()
    running_train_loss = 0.0
    print(f"Epoch {epoch + 1}/{max_epochs}:")

    train_progress_bar = tqdm(enumerate(train_loader), total=len(train_loader), desc="Training")
    for batch_idx, (obs, action, instruct) in train_progress_bar:
        obs_images = list(obs["sensors"].values())[0].float()
        obs_status = obs["status"].float()
        obs = obs_status
        obs, action = obs.to(device).float(), action.to(device).float()
        goal_joint_tensor = torch.stack(instruct['goal_joint'], dim=-1).to(device).float()
        goal_cart_tensor = torch.stack(instruct['goal_cart'], dim=-1).to(device).float()

        if goal_type == "cart":
            obs = torch.cat((obs, goal_cart_tensor[:, :3]), dim=-1)
        else:
            obs = torch.cat((obs, goal_joint_tensor), dim=-1)

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
    print(f"Train Loss: {running_train_loss / len(train_dataset)}")

    # Testing loop
    model.eval()
    test_loss = 0.0
    test_progress_bar = tqdm(enumerate(test_loader), total=len(test_loader), desc="Testing")
    with torch.no_grad():
        for _, (obs, action, instruct) in test_progress_bar:
            obs_images = list(obs["sensors"].values())[0].float()
            obs_status = obs["status"].float()
            obs = obs_status
            obs, action = obs.to(device).float(), action.to(device).float()
            goal_joint_tensor = torch.stack(instruct['goal_joint'], dim=-1).to(device).float()
            goal_cart_tensor = torch.stack(instruct['goal_cart'], dim=-1).to(device).float()

            if goal_type == "cart":
                obs = torch.cat((obs, goal_cart_tensor[:, :3]), dim=-1)
            else:
                obs = torch.cat((obs, goal_joint_tensor), dim=-1)

            outputs = model(obs)
            test_loss += criterion(outputs, action).item()

    print(f"Test Loss: {test_loss / len(test_dataset)}")
    print(f"")

    # Evaluate the model in the real environment
    if (epoch + 1) % int(max_epochs / 5) == 0:
        torch.save(model.state_dict(), 'train/latest.pth')
print("start evaluate")
if env is None:
    env = EdiEnv()
for i in range(evaluate_num):
    evaluate(model, env)
print("finish evaluate")
env.close()
env = None

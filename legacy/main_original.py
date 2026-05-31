import math
import random
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

try:
    import airsim
except ImportError as exc:
    raise ImportError(
        "找不到 airsim 套件，請先安裝: pip install airsim"
    ) from exc


@dataclass
class PlannerConfig:
    goal_x: float = 60.0
    goal_y: float = 20.0
    goal_z: float = -8.0
    target_speed: float = 4.0
    control_dt: float = 0.4
    horizon: int = 12
    num_samples: int = 120
    max_steps: int = 600
    min_obstacle_dist: float = 4.0
    goal_reach_dist: float = 3.0
    train_after_steps: int = 40
    train_every: int = 4
    train_epochs: int = 2
    batch_size: int = 128


class WorldModel(nn.Module):
    def __init__(self, state_dim: int, action_dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim + action_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, state_dim),
        )

    def forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        return self.net(torch.cat([state, action], dim=-1))


class AirSimNeuroPlanner:
    def __init__(self, cfg: PlannerConfig):
        self.cfg = cfg
        self.client = airsim.MultirotorClient()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.state_dim = 8
        self.action_dim = 3
        self.model = WorldModel(self.state_dim, self.action_dim).to(self.device)
        self.optimizer = optim.Adam(self.model.parameters(), lr=1e-3)
        self.memory = deque(maxlen=50000)

        self.goal = np.array([cfg.goal_x, cfg.goal_y, cfg.goal_z], dtype=np.float32)
        self.last_action = np.zeros(3, dtype=np.float32)

    def connect_and_prepare(self) -> None:
        print("[INFO] 連線到 AirSim...")
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.client.takeoffAsync(timeout_sec=12).join()
        self.client.moveToZAsync(self.cfg.goal_z, 2.5).join()
        print("[INFO] UAV 已起飛，開始任務。")

    def shutdown(self) -> None:
        self.client.hoverAsync().join()
        self.client.armDisarm(False)
        self.client.enableApiControl(False)

    def _distance_to_goal(self, position_xyz: np.ndarray) -> float:
        return float(np.linalg.norm(position_xyz - self.goal))

    def _estimate_front_obstacle_dist(self) -> float:
        lidar = self.client.getLidarData(lidar_name="LidarSensor1")
        if lidar.point_cloud and len(lidar.point_cloud) >= 3:
            pts = np.array(lidar.point_cloud, dtype=np.float32).reshape(-1, 3)
            dists = np.linalg.norm(pts, axis=1)
            return float(np.clip(np.min(dists), 0.2, 50.0))

        response = self.client.simGetImages(
            [airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True)]
        )[0]
        if response.width > 0 and response.height > 0 and response.image_data_float:
            depth = np.array(response.image_data_float, dtype=np.float32)
            depth = np.clip(depth, 0.2, 100.0)
            center = depth[(response.width * response.height) // 2]
            return float(center)

        return 50.0

    def get_state_vector(self) -> np.ndarray:
        kinematics = self.client.getMultirotorState().kinematics_estimated
        pos = np.array([kinematics.position.x_val, kinematics.position.y_val, kinematics.position.z_val], dtype=np.float32)
        vel = np.array(
            [
                kinematics.linear_velocity.x_val,
                kinematics.linear_velocity.y_val,
                kinematics.linear_velocity.z_val,
            ],
            dtype=np.float32,
        )
        dist_goal = self._distance_to_goal(pos)
        obstacle_dist = self._estimate_front_obstacle_dist()

        state = np.concatenate([pos, vel, np.array([dist_goal, obstacle_dist], dtype=np.float32)])
        return state

    def _sample_action(self) -> np.ndarray:
        vx = random.uniform(-self.cfg.target_speed, self.cfg.target_speed)
        vy = random.uniform(-self.cfg.target_speed, self.cfg.target_speed)
        vz = random.uniform(-1.0, 1.0)
        return np.array([vx, vy, vz], dtype=np.float32)

    def _rollout_cost(self, state: np.ndarray, action_seq: np.ndarray) -> float:
        total_cost = 0.0
        st = torch.tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)

        for t in range(self.cfg.horizon):
            act = torch.tensor(action_seq[t], dtype=torch.float32, device=self.device).unsqueeze(0)
            with torch.no_grad():
                delta = self.model(st, act)
            st = st + delta

            pred = st.squeeze(0).cpu().numpy()
            px, py, pz = pred[0], pred[1], pred[2]
            dist_goal = float(np.linalg.norm(np.array([px, py, pz], dtype=np.float32) - self.goal))
            obstacle_dist = float(np.clip(pred[7], 0.2, 50.0))

            goal_cost = dist_goal
            obstacle_cost = 15.0 * max(0.0, self.cfg.min_obstacle_dist - obstacle_dist)
            smooth_cost = 0.15 * float(np.linalg.norm(action_seq[t] - self.last_action))
            energy_cost = 0.02 * float(np.linalg.norm(action_seq[t]) ** 2)

            sparse_bonus = -120.0 if dist_goal < self.cfg.goal_reach_dist else 0.0
            total_cost += goal_cost + obstacle_cost + smooth_cost + energy_cost + sparse_bonus

        return total_cost

    def plan_action(self, state: np.ndarray) -> np.ndarray:
        best_cost = float("inf")
        best_seq = None

        for _ in range(self.cfg.num_samples):
            seq = np.array([self._sample_action() for _ in range(self.cfg.horizon)], dtype=np.float32)
            cost = self._rollout_cost(state, seq)
            if cost < best_cost:
                best_cost = cost
                best_seq = seq

        if best_seq is None:
            return self._sample_action()
        return best_seq[0]

    def _safe_action(self, action: np.ndarray, state: np.ndarray) -> np.ndarray:
        obstacle_dist = state[7]
        if obstacle_dist < self.cfg.min_obstacle_dist:
            kinematics = self.client.getMultirotorState().kinematics_estimated
            yaw = airsim.to_eularian_angles(kinematics.orientation)[2]
            action[0] = -2.0 * math.cos(yaw)
            action[1] = -2.0 * math.sin(yaw)
            action[2] = -0.2
        return action

    def step_env(self, action: np.ndarray) -> np.ndarray:
        self.client.moveByVelocityAsync(
            vx=float(action[0]),
            vy=float(action[1]),
            vz=float(action[2]),
            duration=self.cfg.control_dt,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0.0),
        ).join()
        return self.get_state_vector()

    def remember(self, state: np.ndarray, action: np.ndarray, next_state: np.ndarray) -> None:
        self.memory.append((state.copy(), action.copy(), next_state.copy()))

    def train_world_model(self) -> None:
        if len(self.memory) < max(self.cfg.batch_size, self.cfg.train_after_steps):
            return

        for _ in range(self.cfg.train_epochs):
            batch = random.sample(self.memory, self.cfg.batch_size)
            states = torch.tensor(np.stack([item[0] for item in batch]), dtype=torch.float32, device=self.device)
            actions = torch.tensor(np.stack([item[1] for item in batch]), dtype=torch.float32, device=self.device)
            next_states = torch.tensor(np.stack([item[2] for item in batch]), dtype=torch.float32, device=self.device)
            targets = next_states - states

            preds = self.model(states, actions)
            loss = nn.functional.mse_loss(preds, targets)

            self.optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(self.model.parameters(), 5.0)
            self.optimizer.step()

    def run(self) -> None:
        self.connect_and_prepare()

        try:
            state = self.get_state_vector()
            start_time = time.time()

            for step in range(self.cfg.max_steps):
                action = self.plan_action(state)
                action = self._safe_action(action, state)

                next_state = self.step_env(action)
                self.remember(state, action, next_state)
                self.last_action = action.copy()

                if step % self.cfg.train_every == 0:
                    self.train_world_model()

                state = next_state
                dist_goal = state[6]
                obs_dist = state[7]

                print(
                    f"[STEP {step:04d}] dist_goal={dist_goal:6.2f}m | "
                    f"obs={obs_dist:5.2f}m | action={np.round(action, 2)}"
                )

                if dist_goal < self.cfg.goal_reach_dist:
                    print("[SUCCESS] 已到達目標區域。")
                    break

            print(f"[INFO] 任務結束，用時 {time.time() - start_time:.1f} 秒。")
        finally:
            self.shutdown()


def main() -> None:
    cfg = PlannerConfig(
        goal_x=60.0,
        goal_y=20.0,
        goal_z=-8.0,
        target_speed=4.0,
        horizon=10,
        num_samples=100,
        max_steps=500,
    )
    planner = AirSimNeuroPlanner(cfg)
    planner.run()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3

# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from abc import ABC, abstractmethod
from threading import Lock
from typing import Annotated

import rmf_adapter.easy_full_control as rmf_easy
from rmf_adapter.robot_update_handle import ActivityIdentifier


class ExecutionHandle:

    def __init__(self, execution: rmf_easy.CommandExecution | None):
        self.execution = execution
        self.goal_id = None
        self.action = None
        self.mutex = Lock()
        self.mutex.acquire(blocking=True)

    def set_goal_id(self, goal_id):
        self.goal_id = goal_id
        self.mutex.release()

    def set_action(self, action):
        self.action = action
        self.mutex.release()

    @property
    def activity(self) -> ActivityIdentifier | None:
        # Move the execution reference into a separate variable just in case
        # another thread modifies self.execution while we're still using it.
        execution = self.execution
        if execution is not None:
            return execution.identifier
        return None


class RobotAdapter(ABC):
    """Abstract Robot Adapter to be used by the free fleet adapter."""

    def __init__(
        self,
        name: str,
        node,
        fleet_handle
    ):
        self.name = name
        self.node = node
        self.fleet_handle = fleet_handle
        self.update_handle = None

    @abstractmethod
    def get_battery_soc(self) -> float:
        """Return battery state of charge as a float between 0 and 1.0."""
        ...

    @abstractmethod
    def get_map_name(self) -> str:
        """Return the name of the map the robot is currently localized on."""
        ...

    @abstractmethod
    def get_pose(self) -> Annotated[list[float], 3] | None:
        """Return the last known [x, y, yaw] pose (meters, radians), or None
        if the last known position of the robot is not available."""
        ...

    @abstractmethod
    def update(self, state: rmf_easy.RobotState):
        """Update RMF with the latest robot state."""
        ...

    @abstractmethod
    def navigate(
        self,
        destination: rmf_easy.Destination,
        execution: rmf_easy.CommandExecution
    ):
        """Send a navigation command to the robot."""
        ...

    @abstractmethod
    def stop(self, activity: ActivityIdentifier):
        """Stop execution/continuation of the given activity."""
        ...

    @abstractmethod
    def execute_action(
        self,
        category: str,
        description: dict,
        execution: ActivityIdentifier
    ):
        """Send a custom action command to the robot."""
        ...

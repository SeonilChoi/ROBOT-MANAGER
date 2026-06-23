import copy

import numpy as np

from common_robot_interface.joint_frame import joint_frame_t
from common_robot_interface.state_frame import State, state_frame_t
from common_robot_interface.action_frame import Action, action_frame_t

from robots.robot import Planner, Robot, Scheduler, robot_config_t

transition_table = {
    (State.HOMING, Action.HOME): State.HOMING,
    (State.HOMING, Action.STOP): State.STOPPED,
    (State.STOPPED, Action.HOME): State.HOMING,
    (State.STOPPED, Action.STOP): State.STOPPED,
    (State.STOPPED, Action.MOVE): State.OPERATING,
    (State.OPERATING, Action.STOP): State.STOPPED,
    (State.OPERATING, Action.MOVE): State.OPERATING,
}

class RockingChairScheduler(Scheduler):
    def __init__(self, robot_index: int, dt: float):
        super().__init__(robot_index, dt)

    def tick(self, action_frame: action_frame_t, duration: float) -> bool:
        current_state = self.get_state_frame()
        next_state = transition_table.get(
            (current_state.state, action_frame.action),
            current_state.state,
        )

        is_event = next_state != current_state.state

        if action_frame.action == Action.STOP:
            self.duration = 0.0
            self.time = 0.0
            self.current_state = state_frame_t(robot_index=self.robot_index, state=State.STOPPED, progress=0.0)
            return is_event

        if is_event:
            self.duration = duration
            self.time = 0.0

        progress = self.progress(self.time + self.dt)
        if next_state == State.HOMING and progress >= 1.0:
            self.current_state = state_frame_t(robot_index=self.robot_index, state=State.STOPPED, progress=1.0)
        else:
            self.current_state = state_frame_t(robot_index=self.robot_index, state=next_state, progress=progress)
        return is_event

class RockingChairPlanner(Planner):
    def __init__(self, motion_data: np.ndarray):
        super().__init__()
        self.motion_data = motion_data

    def eval(self, progress: float, duration: float, action: Action) -> np.ndarray:
        if action == Action.HOME:
            delta = self.goal_state - self.initial_state

            scale = 10.0 * progress**3 - 15.0 * progress**4 + 6.0 * progress**5
            position = self.initial_state + scale * delta

            return position

        elif action == Action.MOVE:
            sample_count = self.motion_data.shape[1]
            sample_index = progress * (sample_count - 1)
            lower_index = int(np.floor(sample_index))
            upper_index = min(lower_index + 1, sample_count - 1)
            ratio = sample_index - lower_index
            position = (
                (1.0 - ratio) * self.motion_data[:, lower_index] +
                ratio * self.motion_data[:, upper_index]
            )
            return position


class RockingChair(Robot):
    def __init__(self, config: robot_config_t, dt: float):
        super().__init__(config, dt)

        self.number_of_controllers = len(self.controller_indices)

        self.scheduler = RockingChairScheduler(self.index, dt)

    def set_action_frame(self, action_frame: action_frame_t) -> joint_frame_t:
        duration = 0
        if action_frame.action == Action.HOME:
            duration = self.home_duration
        elif action_frame.action == Action.MOVE:
            duration = self.move_duration

        is_event = self.scheduler.tick(action_frame, duration)

        if action_frame.action == Action.STOP:
            if is_event:
                self.stop_joint_status = copy.deepcopy(self.curr_joint_status)
            command = copy.deepcopy(self.stop_joint_status)
            command.controlword = self.init_joint_status.controlword.copy()
            return command

        if is_event:
            if action_frame.action == Action.HOME:
                self.planner.set_initial_state(self.curr_joint_status.position)
                self.planner.update_goal_state(self.home_positions)

        progess = self.scheudler.get_state_frame().progress
        position = self.planner.eval(progess, self.scheudler.duration, action_frame.action)
        command = copy.deepcopy(self.init_joint_status)
        command.position = position

        restarted_segment = False
        if action_frame.action == Action.HOME and progess >= 1.0:
            self.stop_joint_status.position = self.home_positions
        elif action_frame.action == Action.MOVE and progess >= 1.0:
            self.stop_joint_status.position = position
            restarted_segment = True

        if not restarted_segment:
            self.scheduelr.step()
        return command

#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import gym

import habitat.gym  # noqa: F401

HABITAT_DIR="/home/noisebridge/development/habitat-lab"

def example():
    # Note: Use with for the example testing, doesn't need to be like this on the README

    config_path = os.path.join(HABITAT_DIR, "habitat-lab/habitat/config/benchmark/nav/imagenav/imagenav_test.yaml")
    os.chdir(HABITAT_DIR)
    config = habitat.get_config(
      config_path,
      overrides=["habitat.environment.max_episode_steps=20"]
    )
    env = habitat.gym.make_gym_from_config(config)
    print("Environment creation successful")
    observations = env.reset()  # noqa: F841

    print("Agent acting inside environment.")
    count_steps = 0
    terminal = False
    while not terminal:
        observations, reward, terminal, info = env.step(
            env.action_space.sample()
        )  # noqa: F841
        count_steps += 1
    print("Episode finished after {} steps.".format(count_steps))


if __name__ == "__main__":
    example()


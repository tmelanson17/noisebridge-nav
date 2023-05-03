# noisebridge-nav
Creating an E2E AI for navigating an indoor environment.

The goal is to create a robot that can navigate an indoor space. Although the original idea is something that can fetch an object or look for things in a space, we will focus on image or location-based navigation for the time being. 
This (rather large) goal can be broken down into several smaller goals:
- Train a PPO to do the [IndoorNav](https://github.com/facebookresearch/habitat-lab) project
    - [Visual Cortex 1](https://github.com/facebookresearch/eai-vc)  can help train a better PPO model by creating a visual representation
- Optionally, create a bare-bones navigation and obstacle avoidance system. I have [Roomba RL](https://github.com/tmelanson17/roomba-rl), but a more successful version can be found [here](https://www.youtube.com/@robotmania8896/videos)
- Develop a map of Noisebridge compatible with the IndoorNav project
    - Needs to convert a 5 DoF position/viewing to an image
    - Can be done through [NeRFs](https://www.matthewtancik.com/nerf)
    - We (i.e. Noisebridge) also have a GitLab with a [3D model of Noisebridge](https://gitlab.com/unityversity/simbridge) , no idea how to convert this to the format above

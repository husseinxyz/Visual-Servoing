# Visual Servoing

## Introduction
Code for control of a mobile robot equipped with a RGB camera to follow a round-shaped square trajectory using visual servoing techniques instead of calculating the pose of the camera and correcting the error. The yaw angle is controlled to follow the path in the middle from visual data using visual servoing techniques.

## Overview of the Repository
In this repo, you'll find :
* `utils.py`: code for circuit generation, frame transformations, and animations.
* `visual_servoing.py`: code for running simulation and plotting results.

## Getting Started
1.  Clone repo: `git clone https://github.com/husseinxyz/Visual-Servoing.git`
2.  Install dependencies:
    ```
    conda create -n visual-servoing python=3.7
    conda activate visual-servoing
    pip install -r requirements.txt
    ```

And you're good to go!

---
layout: post
title: "Capturing Images with CARLA (0.9.15) Python Client: A Simplified Guide"
categories: [carla, python, c++, simulation, autonomous-vehicles]
tags: [carla, python, c++, simulation, api, autonomous-vehicles]
---

![CARLA Simulator](https://carla.readthedocs.io/en/latest/img/tuto_rgb.jpg)

*Image Source: [CARLA Simulator](https://carla.org/)*

## Introduction

Capturing images from a simulated environment is essential for training and testing autonomous vehicle algorithms. While CARLA offers extensive documentation, getting started can sometimes feel overwhelming. This tutorial provides a simplified, step-by-step guide to capturing images using CARLA's Python API. Whether you're a beginner or just looking for a straightforward example, this guide is for you.

## Prerequisites

- **CARLA Simulator Installed**: If you haven't installed CARLA yet, you can install the [compiled version](https://github.com/carla-simulator/carla/releases/tag/0.9.15/) following the [official documentation](https://carla.readthedocs.io/en/latest/start_quickstart/). If you prefer to build CARLA from source on Windows refer to this comprehensive [installation guide](https://wambitz.github.io/tech-blog/carla/python/c++/simulation/autonomous-vehicles/2024/09/29/carla-win11.html) for assistance.
- **Python 3.8**: Ensure you have **Python 3.8** installed on your system.

## Setting Up the Project

1. **Navigate to the PythonAPI Directory**: Open your terminal or command prompt and navigate to the `PythonAPI` directory within your CARLA installation folder.

   ```bash
   cd path_to_carla/PythonAPI
   ```

2. **Create a New Project Directory**: We'll create a new directory for our project to keep things organized.

   ```bash
   mkdir image_capture_tutorial
   cd image_capture_tutorial
   ```

3. **Set Up a Virtual Environment (Optional)**: It's good practice to use a virtual environment.

   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows use `venv\Scripts\activate`
   ```

4. **Install Required Packages**: Install any necessary Python packages.

   This will install the carla package from the Python Package Index (`PyPi`). **If you build from source** with `make PythonAPI` you can install from your own wheel located at: `PythonAPI\carla\dist\carla-0.9.15-cp38-cp38-win_amd64.whl`

   ```bash
   pip install carla
   ```

5. **Create your script file**: Use any preferred method and create: `camera_tutorial.py`


## Step-by-Step Code Explanation

We'll walk through the code step by step, explaining each part as we go. I'll provide the final script at the end of the article.

### 1. Importing Necessary Modules

```python
import carla
import time
import os
import shutil
```

- **carla**: The main module to interact with the CARLA simulator.
- **time**: Used to control the duration of the simulation.
- **os & shutil**: For file system operations, like creating and cleaning directories.

### 2. Cleaning Up the Output Directory

This function ensures that the output directory is clean before saving new images.

```python
def cleanup_output_directory(output_dir):
    # Delete any existing files in the output directory
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)
    print(f"Cleaned up and prepared output directory: {output_dir}")
```

### 3. Connecting to the CARLA Server

We define the `main()` function where the core logic will reside and connect to CARLA sever.

```python
def main():
    # Connect to CARLA server
    client = carla.Client('localhost', 2000) 
    client.set_timeout(10.0)
```

- **localhost, 2000**: Connects to the CARLA server running on your local machine at port 2000.
- **set_timeout(10.0)**: Sets a timeout of 10 seconds for the client.

### 4. Loading the World and Blueprints

```python
    # Load the world and retrieve the blueprint library
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
```

- **get_world()**: Retrieves the world environment.
- **get_blueprint_library()**: Accesses all the available blueprints (vehicles, sensors, etc.).

### 5. Setting the Spectator View

Moves the spectator (the camera view in the simulator) to a specific location and orientation. This helps you see the simulation from a bird's-eye view.

```python
    # Spectator - Pointing to spawn point 0
    spectator = world.get_spectator()
    camera_loc = carla.Location(x=110.03, y=216.0, z=50.0)
    camera_rot = carla.Rotation(pitch=-69.0, yaw=0.0, roll=0.0)
    camera_trans = carla.Transform(camera_loc, camera_rot)
    
    # Move the spectator
    spectator.set_transform(camera_trans)
```

### 6. Spawning the Vehicle

```python
    # Find a cybertruck blueprint
    vehicle_bp = blueprint_library.find("vehicle.tesla.cybertruck") 
    
    # Choose a spawn point for the car
    spawn_points = world.get_map().get_spawn_points()
    vehicle_spawn_point = spawn_points[0]
    
    # Spawn the vehicle
    vehicle = world.spawn_actor(vehicle_bp, vehicle_spawn_point)
    print("Vehicle spawned.")
```

- **vehicle.tesla.cybertruck**: Selects the Tesla Cybertruck model.
- **spawn_points[0]**: Chooses the first available spawn point.
- **spawn_actor()**: Spawns the vehicle into the world.

### 7. (Optional) Retrieving Vehicle Dimensions

This code calculates the full dimensions of the vehicle for positioning the camera correctly.

```python
    # Retrieve vehicle dimensions
    vehicle_extent = vehicle.bounding_box.extent
    vehicle_length = vehicle_extent.x * 2  # Full length of the vehicle
    vehicle_width = vehicle_extent.y * 2   # Full width of the vehicle
    vehicle_height = vehicle_extent.z * 2  # Full height of the vehicle

    print(f"Vehicle dimensions - Length: {vehicle_length} m, Width: {vehicle_width} m, Height: {vehicle_height} m")
```

### 8. Enabling Autopilot

Activate the autopilot so the vehicle moves autonomously.

```python
    # Enable autopilot
    vehicle.set_autopilot(True)
    print("Autopilot enabled.")
```

### 9. Setting Up the Camera

We will use an RGB camera sensor. **NOTE**: This will capture images every second to for testing purposes and to save some memory. This is not optimal but the simulation will run for 30s and we will capture 30 images for demonstration purposes only.

```python
    # Find a camera blueprint
    camera_bp = blueprint_library.find('sensor.camera.rgb')

    # Adjust camera attributes (optional)
    camera_bp.set_attribute('image_size_x', '1920')
    camera_bp.set_attribute('image_size_y', '1080')
    camera_bp.set_attribute('fov', '105') # Field of view in degrees
    camera_bp.set_attribute('sensor_tick', '1.0') # Capture image every second
```

- **Attributes**:
  - **image_size_x & image_size_y**: Sets the resolution of the images.
  - **fov**: Field of view in degrees.
  - **sensor_tick**: Captures an image every second.

### 10. Positioning the Camera

Positions the camera slightly ahead and above the vehicle, facing forward.

```python
    # Calculate camera location based on vehicle dimensions
    camera_location = carla.Location(
        x=vehicle_extent.x + 0.5,    # Slightly forward from the vehicle's front
        y=0.0,                       # Centered horizontally on the vehicle
        z=vehicle_extent.z           # Elevated above the roof
    )

    # Adjust camera rotation to face forward
    camera_rotation = carla.Rotation(pitch=-10.0, yaw=0.0, roll=0.0)  # Tilt slightly downward
    camera_transform = carla.Transform(camera_location, camera_rotation)
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
```

### 11. Preparing to Save Images

Prepare the directory where images will be saved.

```python
    # Print the current working directory (where files will be saved)
    print("Saving files to:", os.getcwd())

    # Create directory for saving images if it doesn't exist
    output_dir = "images"
    cleanup_output_directory(output_dir)
```

### 12. Listening to the Camera Sensor

```python
    # Start listening to camera images
    camera.listen(lambda image: image.save_to_disk(f'{output_dir}/{image.frame}.png'))
```

- **listen()**: Starts the sensor listening for data.

### 13. Running the Simulation

Run the simulation for 30 seconds, then cleans up by destroying the actors.

```python
    try:
        # Let the simulation run for 30 seconds
        time.sleep(30)
    
    finally:
        # Clean up by destroying actors
        camera.stop()
        camera.destroy()
        vehicle.destroy()
        print("Actors destroyed, simulation finished.")
```


## Running the Script

1. **Start the CARLA Simulator**: Before running the script, make sure the CARLA server is up and running.

   There are two options here, if you downloaded the CARLA package from Github you can run:

   ```bash
   ./CarlaUE4.sh  # On Windows, run CarlaUE4.exe
   ```

   Alternatively **if you built CARLA from source**, this command will compile and build CARLA. 

   ```bash
   make launch
   ```

2. **Execute the Script**: In your terminal, run:

   ```bash
   cd PythonAPI/image_capture_tutorial/
   python camera_tutorial.py
   ```

3. **Watch the Simulation**: The vehicle should spawn and start moving with the camera capturing images **every second**.

4. **Check the Output**: After the simulation finishes, you'll find the captured images in the `images` directory.

   ```bash
   ls images/
   ```

## Full Source Code

```python
import carla
import time
import random
import os
import shutil

def cleanup_output_directory(output_dir):
    # Delete any existing files in the output directory
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)
    print(f"Cleaned up and prepared output directory: {output_dir}")

def main():
    ############## SETTINGS #################

    # Connect to CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Load the world and retrieve the blueprint library
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    # Spectator - Pointing to spawn point 0
    spectator = world.get_spectator()
    camera_loc = carla.Location(x=110.02999877929688, y=216.0, z=50.0)
    camera_rot = carla.Rotation(pitch=-69.0, yaw=0.0, roll=0.0)
    camera_trans = carla.Transform(camera_loc, camera_rot)
    
    # Move the spectator
    spectator.set_transform(camera_trans)

    ############## CAR #################

    # Find a cybertruck blueprint
    vehicle_bp = blueprint_library.find("vehicle.tesla.cybertruck") 
    
    # Choose a spawn point for the car
    spawn_points = world.get_map().get_spawn_points()
    vehicle_spawn_point = spawn_points[0]

    # Spawn the vehicle
    vehicle = world.spawn_actor(vehicle_bp, vehicle_spawn_point)
    print("Vehicle spawned.")

    # Retrieve vehicle dimensions
    vehicle_extent = vehicle.bounding_box.extent
    vehicle_length = vehicle_extent.x * 2  # Full length of the vehicle
    vehicle_width = vehicle_extent.y * 2   # Full width of the vehicle
    vehicle_height = vehicle_extent.z * 2  # Full height of the vehicle

    print(f"Vehicle dimensions - Length: {vehicle_length} m, Width: {vehicle_width} m, Height: {vehicle_height} m")

    # Enable autopilot
    vehicle.set_autopilot(True)
    print("Autopilot enabled.")

    ############## CAMERA #################

    # Find a camera blueprint
    camera_bp = blueprint_library.find('sensor.camera.rgb')

    # Adjust camera attributes (optional)
    camera_bp.set_attribute('image_size_x', '1920')
    camera_bp.set_attribute('image_size_y', '1080')
    camera_bp.set_attribute('fov', '105') # Field of view in degrees
    camera_bp.set_attribute('sensor_tick', '1.0') # Capture image every second

    # Calculate camera location based on vehicle dimensions
    camera_location = carla.Location(
        x=vehicle_extent.x + 0.5,    # Slightly forward from the vehicle's front
        y=0.0,                       # Centered horizontally on the vehicle
        z=vehicle_extent.z + 0.0     # Elevated above the roof
    )

    # Adjust camera rotation to face forward
    camera_rotation = carla.Rotation(pitch=-10.0, yaw=0.0, roll=0.0)  # Tilt slightly downward
    camera_transform = carla.Transform(camera_location, camera_rotation)
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    # Print the current working directory (where files will be saved)
    print("Saving files to:", os.getcwd())

    # Create directory for saving images if it doesn't exist
    output_dir = "images"
    cleanup_output_directory(output_dir)

    # Start listening to camera images
    camera.listen(lambda image: image.save_to_disk(f'{output_dir}/{image.frame}.png'))

    ############## RUN SIMULATION #################

    try:
        # Let the simulation run for 30 seconds
        time.sleep(30)
    
    finally:
        # Clean up by destroying actors
        camera.stop()
        camera.destroy()
        vehicle.destroy()
        print("Actors destroyed, simulation finished.")

if __name__ == "__main__":
    main()
```

## Additional: Getting Spawn Point Coordinates

Sometimes, you might want to know the exact coordinates of available spawn points to position your vehicle or spectator camera precisely. This is particularly useful for setting the spectator view or spawning vehicles at specific locations. I used this for [Step 5](#5-setting-the-spectator-view). Here's a simple script to list all spawn points with their coordinates. 

### Code to Retrieve Spawn Point Coordinates

```python
import carla

def main():

    # Connect to CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Load the world
    world = client.get_world()

    # List all vehicle blueprints
    for bp in world.get_blueprint_library().filter('vehicle'):
        print(bp.id)

    # Retrieve all spawn points
    spawn_points = world.get_map().get_spawn_points()

    # Print each spawn point with an index
    print("Available Spawn Points:")
    for idx, spawn_point in enumerate(spawn_points):
        print(f"Spawn Point {idx}: Location({spawn_point.location.x}, {spawn_point.location.y}, {spawn_point.location.z}), "
              f"Rotation({spawn_point.rotation.pitch}, {spawn_point.rotation.yaw}, {spawn_point.rotation.roll})")

if __name__ == "__main__":
    main()
```


## Conclusion

You've now successfully captured images from a CARLA simulation using a custom Python script! This basic example can serve as a foundation for more complex projects, such as data collection for machine learning models or sensor fusion experiments.

Additionally, you learned how to retrieve spawn point coordinates, which can help in customizing your simulation environment further.

For any issues related to installation or more advanced configurations, refer to this detailed [CARLA installation guide](https://wambitz.github.io/tech-blog/carla/python/c++/simulation/autonomous-vehicles/2024/09/29/carla-win11.html).

Feel free to share your experiences or ask questions in the comments below. 

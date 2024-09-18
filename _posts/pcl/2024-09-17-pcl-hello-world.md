---
layout: post
title:  "PCL Series: Visualizing a Simple Cube with PCL in C++"
categories: C++ PCL Docker Devcontainer Pointcloud 3D
---

In this post, we'll walk through a small C++ example using the Point Cloud Library (PCL) to create a basic 3D visualization of a cube. This can be particularly useful for understanding the basics of setting up a simple scene with PCL. We'll also cover how to configure the development environment with Docker, XServer, and VS Code to easily work with PCL.

## Code Explanation

Let's take a look at the following code:

```cpp
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

struct Box
{
    float x_min;
    float y_min;
    float z_min;
    float x_max;
    float y_max;
    float z_max;
};

int main(int argc, char** argv)
{
    /////////////////////////////////////////// CUBE EXAMPLE /////////////////////////////////////////////////
    Box window;
    window.x_min = -10;
    window.x_max = 10;
    window.y_min = -10;
    window.y_max = 10;
    window.z_min = 0;
    window.z_max = 0;
    int zoom = 25;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);

    viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");

    // Set the color of the cube to red using shape rendering properties (r = 1.0, g = 0.0, b = 0.0)
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "window"); // Red cube

    // Set transparency if needed (optional)
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "window"); // 20% opaque

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Main loop to keep the viewer open
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);                                       // Update the viewer
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep for a bit to reduce CPU usage
    }

    return 0;
}
```

### What's Happening in the Code?

1. **Box Structure**:
   We define a simple `Box` structure that holds the coordinates of the minimum and maximum values for the cube along the x, y, and z axes.

2. **PCLVisualizer Setup**:
   - The `PCLVisualizer` class from the PCL library is initialized to create a viewer window.
   - `setBackgroundColor(0, 0, 0)` sets the background of the viewer to black.
   - `initCameraParameters()` initializes the camera to enable proper visualization.
   - `setCameraPosition(0, 0, zoom, 0, 1, 0)` sets the camera position to give a proper 3D view, where `zoom` controls how far the camera is placed from the cube.
   - A coordinate system is added to the scene with `addCoordinateSystem(1.0)`.

3. **Adding a Cube**:
   - The cube is created using the bounds from the `Box` structure.
   - `addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window")` adds the cube to the viewer with the specified bounds and color (white by default).

4. **Shape Rendering Properties**:
   - The cube is set to red with the line `setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "window")`, where `(1.0, 0.0, 0.0)` is the RGB value for red.
   - The transparency of the cube is adjusted to 20% with `setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "window")`.

5. **Main Loop**:
   - The viewer runs in a loop until the user closes it by calling `wasStopped()`. 
   - The `spinOnce()` function updates the viewer in each loop iteration.
   - The thread sleeps for 100 milliseconds to avoid excessive CPU usage.

This simple code demonstrates how to create a 3D scene, set up the camera, and control basic shape properties (like color and transparency) in PCL.

![alt text]({{ site.baseurl }}/assets/images/posts/2024-09-21-pcl-hello-world/cube-2d-viewer.png)


## Setting Up the Development Environment

To work with PCL, Docker, XServer, and VS Code DevContainers, you can set up a convenient development environment by following these steps. I'll be referring to a GitHub repository I created for this purpose:

**Repository**: [PCL Docker Environment with XServer and VS Code DevContainers](https://github.com/wambitz/pcl-devcontainer)

### Prerequisites

Before setting up the environment, make sure you have:

- **Docker**: Install from the official [Docker site](https://docs.docker.com/get-docker/).
- **VS Code**: Download from [here](https://code.visualstudio.com/).
  - Install the **Remote - Containers** extension.
- **XServer**: Required for visualization.
  - For Linux, XServer is often available by default.
  - For macOS or Windows, install [XQuartz](https://www.xquartz.org/) or [VcXsrv](https://sourceforge.net/projects/vcxsrv/), respectively.

### Setup Instructions

1. **Clone the Repository**:
   Start by cloning the repository and navigating into the directory:

   ```bash
   git clone https://github.com/wambitz/pcl-devcontainer
   cd pcl-devcontainer
   ```

2. **Build the Docker Image**:
   Build the Docker image using the provided Dockerfile:

   ```bash
   docker build -t pcl-dev .
   ```

3. **Run the Docker Container with XServer**:
   To run the Docker container with graphical support for PCL, use the following command:

   ```bash
   docker run -it --rm \
       -e DISPLAY=$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       pcl-dev
   ```

   If you're on Linux, ensure Docker has the necessary permissions by running:

   ```bash
   xhost +local:docker
   ```

4. **Develop in VS Code**:
   If you prefer working in VS Code, the repository includes a `devcontainer.json` file for a pre-configured development environment.

   - Open the repository in VS Code.
   - Use the `Remote - Containers: Reopen in Container` command to load the environment within Docker.

5. **Build Your Project**:
   Once the container is running, you can build your PCL projects:

   ```bash
   cmake -S . -B build
   cmake --build build
   ```

## Known Issues

- **VTK Rendering Issues**: There are known issues with VTK on newer versions of Ubuntu. To avoid these, the setup builds the PCL from source. You can read more about the issue in this [GitHub thread](https://github.com/PointCloudLibrary/pcl/issues/5237).

For more details on running GUI applications with Docker and XServer, check out [this blog post](https://wambitz.github.io/tech-blog/jekyll/update/2024/07/25/gui-applications-with-docker.html).

## References

- [Point Cloud Library Documentation](https://pointclouds.org/documentation/)
- [Docker Documentation](https://docs.docker.com/)
- [VS Code Remote - Containers](https://code.visualstudio.com/docs/remote/containers)
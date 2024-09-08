---
layout: post
title:  "Running GUI Applications in Docker on Linux and Windows"
categories: jekyll update
---

# Running GUI Applications in Docker (Linux and Windows)

![Container Image]({{ site.baseurl }}/assets/images/posts/2024-07-25-gui-applications-with-docker/container-image.png)


## Introduction
Docker is widely known for its ability to containerize applications, making deployment and management more straightforward. While it’s often associated with server-side applications, Docker can also be used to run GUI applications. In this article, we’ll explore how to run GUI applications in Docker on both Linux and Windows.

## Prerequisites
Before we start, make sure you have Docker installed on your system. If you don’t have Docker installed yet, you can follow the [official installation guide](https://docs.docker.com/get-docker/).

## Running GUI Applications in Docker (Linux and Windows)

### Step 1: Install Docker

Ensure Docker is installed on your system. You can follow the appropriate instructions for your platform:

- **Linux**: You can install Docker using your package manager (`apt`, `yum`, etc.). I personally prefer to do the [manual installation](https://docs.docker.com/engine/install/ubuntu/) either from the command line or the official Docker installation script.

- **Windows**: Install Docker Desktop by following the official guide.

### Step 2: Install X11 or X Server

- **Linux**: Linux natively supports X11, so there’s no need for additional software.
- **Windows**: Download and install VcXsrv from the [official website](https://sourceforge.net/projects/vcxsrv/). 

### Step 3: Start the X Server

- **Linux**: You don’t need to manually start X11, as it typically runs by default.
- **Windows**: Launch VcXsrv with the following settings:
  - Multiple windows
  - Display number: `0`
  - Start no client
  - Disable access control

### Step 4: Create a Dockerfile

Let’s create a simple Dockerfile for a GUI application. In this example, we’ll use Firefox.

```dockerfile
# Use an official Ubuntu as a parent image
FROM ubuntu:20.04

# Install Firefox and X11
RUN apt-get update && apt-get install -y firefox x11-apps

# Set environment variable to avoid warnings
ENV DISPLAY=:0

# Run Firefox when the container launches
CMD ["firefox"]
```

### Step 5: Build the Docker Image

Navigate to the directory containing your Dockerfile and build the image.

```bash
docker build -t firefox-docker .
```

### Step 6: Configure the Display Environment Variable

- **Linux**: Use the `DISPLAY` environment variable to pass your display to Docker. 

  ```bash
  docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix firefox-docker
  ```

- **Windows**: First, find your host IP address by running `ipconfig` in the command prompt and noting your IPv4 address. Then, use the `DISPLAY` variable to point to your VcXsrv server, replacing `YOUR_HOST_IP` with your IP address.

  ```bash
  docker run -e DISPLAY=host.docker.internal:0.0 firefox-docker
  ```
Now you can run the container, and Firefox should appear on your host’s display. The output on your terminal should look like this:

```bash
$ docker run -e DISPLAY=host.docker.internal:0.0 firefox-docker
[GFX1-]: glxtest: libpci missing
[GFX1-]: glxtest: libEGL missing
[GFX1-]: glxtest: libGL.so.1 missing
[GFX1-]: No GPUs detected via PCI
```

![X Server]({{ site.baseurl }}/assets/images/posts/2024-07-25-gui-applications-with-docker/firefox-screenshot.png)

## Troubleshooting: Allow Docker to Access the X Server

- **Linux**: You may need to allow Docker to access your X server. Run the following command:

  ```bash
  xhost +local:docker
  ```

- **Windows**: Make sure VcXsrv is configured to allow connections from the Docker container. If you have trouble, you might need to disable the Windows firewall or add an exception for VcXsrv.


## Conclusion
Running GUI applications in Docker can be a powerful way to encapsulate and manage your software, even for desktop applications. By leveraging X11 on Linux or VcXsrv on Windows, you can bring the benefits of containerization to your GUI applications.

This guide provides a starting point for using Docker with GUI applications. Experiment with different applications and settings to find what works best for your use case. Happy containerizing!

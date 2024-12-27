## Setting Up the EPSILON Project Environment in Docker

In addition to `ros:melodic`, `carla:0.9.11` and the corresponding `ros_carla_bridge` have been added.

### Download CARLA

Please download the package of `CARLA 0.9.11` from <https://github.com/carla-simulator/carla/releases/tag/0.9.11>.
And, please put the package of `CARLA 0.9.11` in 'docker/build/install' folder.
 
If you don't want `carla` and `ros_carla_bridge`, just delete the relevant content in the Dockerfile, so the entire Docker image will be built only for EPSILON.

## Requirements

* NVIDIA graphics driver
* Docker
* nvidia-docker2

## Run

```shell
# Build Docker image
cd /xxxx/EPSILON/docker/build/
docker build --network=host -t epsilon:0.1.1 --build-arg GID=$(id -g) --build-arg UID=$(id -u) -f Dockerfile.melodic .

# Modify EPSILON_PATH in launch_container.sh to your own path
# Enter Docker container
sh ./launch_container.sh
```
## Reference
[EPSILON](https://github.com/HKUST-Aerial-Robotics/EPSILON) \
[carla_ros_bridge_docker](https://github.com/atinfinity/carla_ros_bridge_docker/tree/master)

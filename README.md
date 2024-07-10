## Introduction 

The following section is for parameters related to pointcloud_to_laserscan nodelet

ref: http://wiki.ros.org/pointcloud_to_laserscan
ref: https://github.com/ros-perception/pointcloud_to_laserscan
ref: https://github.com/ros-gbp/pointcloud_to_laserscan-release

## Node

### pointcloud_to_laserscan_node

pointcloud_to_laserscan_node takes a point cloud and generates a 2D laser scan based on the provided parameters.

#### Subscribed Topics

cloud_in (sensor_msgs/PointCloud2)

The input point cloud.

#### Published Topics

scan (sensor_msgs/LaserScan)
The output laser scan.

#### Parameters

~min_height (double, default: 0.0)

    The minimum height to sample in the point cloud in meters.

~max_height (double, default: 1.0)

    The maximum height to sample in the point cloud in meters.

~angle_min (double, default: -π/2)

    The minimum scan angle in radians.

~angle_max (double, default: π/2)

    The maximum scan angle in radians.

~angle_increment (double, default: π/360)

    Resolution of laser scan in radians per ray.

~scan_time (double, default: 1.0/30.0)

    The scan rate in seconds.

~range_min (double, default: 0.45)

    The minimum ranges to return in meters.

~range_max (double, default: 4.0)

    The maximum ranges to return in meters.

~target_frame (str, default: none)

    If provided, transform the pointcloud into this frame before converting to a laser scan. Otherwise, laser scan will be generated in the same frame as the input point cloud.

~concurrency_level (int, default: 1)

    Number of threads to use for processing pointclouds. If 0, automatically detect number of cores and use the equivalent number of threads. Input queue size for pointclouds is tied to this parameter. In nodelet form, number of threads is capped by the nodelet manager configuration.

~use_inf (boolean, default: true)

    If disabled, report infinite range (no obstacle) as range_max + 1. Otherwise report infinite range as +inf. Associated with the inf_is_valid parameter for costmap_2d obstacle layers.

## Nodelet

Same API as node, available as pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.

##### Metalform Parameters

tf_z_offset (double, default: 0.25)

    the base_link z axis offset relative to the falt ground, we suppose base_link xOy plane is parallel with flat ground. using aucobot prototype with diff wheel radius 0.25 z axis offset between flat ground to base_link origin.

enable_debug_mode (bool, default: false)

    set bool flag for pointcloud_filtered_pub_ enable debug mode

robot_frame (string, default: "base_link") 

    robot frame name, by default "base_link"

## pointcloud_to_laserscan (noetic) - 1.4.1-1

The packages in the `pointcloud_to_laserscan` repository were released into the `noetic` distro by running `/usr/bin/bloom-release -r noetic -t noetic pointcloud_to_laserscan` on `Fri, 05 Jun 2020 13:22:19 -0000`

The `pointcloud_to_laserscan` package was released.

Version of package(s) in repository `pointcloud_to_laserscan`:

- upstream repository: https://github.com/ros-perception/pointcloud_to_laserscan.git
- release repository: unknown
- rosdistro version: `null`
- old version: `null`
- new version: `1.4.1-1`

Versions of tools used:

- bloom version: `0.9.7`
- catkin_pkg version: `0.4.20`
- rosdep version: `0.19.0`
- rosdistro version: `0.8.1`
- vcstools version: `0.1.42`


## pointcloud_to_laserscan (melodic) - 1.4.1-1

The packages in the `pointcloud_to_laserscan` repository were released into the `melodic` distro by running `/usr/bin/bloom-release pointcloud_to_laserscan -r melodic -t melodic` on `Fri, 30 Aug 2019 13:00:13 -0000`

The `pointcloud_to_laserscan` package was released.

Version of package(s) in repository `pointcloud_to_laserscan`:

- upstream repository: https://github.com/ros-perception/pointcloud_to_laserscan.git
- release repository: https://github.com/ros-gbp/pointcloud_to_laserscan-release.git
- rosdistro version: `1.4.0-0`
- old version: `1.4.0-0`
- new version: `1.4.1-1`

Versions of tools used:

- bloom version: `0.8.0`
- catkin_pkg version: `0.4.13`
- rosdep version: `0.15.2`
- rosdistro version: `0.7.4`
- vcstools version: `0.1.42`


## pointcloud_to_laserscan (melodic) - 1.4.0-0

The packages in the `pointcloud_to_laserscan` repository were released into the `melodic` distro by running `/usr/local/bin/bloom-release -r melodic -t melodic pointcloud_to_laserscan` on `Mon, 25 Jun 2018 17:37:20 -0000`

The `pointcloud_to_laserscan` package was released.

Version of package(s) in repository `pointcloud_to_laserscan`:

- upstream repository: https://github.com/ros-perception/pointcloud_to_laserscan.git
- release repository: unknown
- rosdistro version: `null`
- old version: `null`
- new version: `1.4.0-0`

Versions of tools used:

- bloom version: `0.6.5`
- catkin_pkg version: `0.4.3`
- rosdep version: `0.12.2`
- rosdistro version: `0.6.8`
- vcstools version: `0.1.40`


## pointcloud_to_laserscan (kinetic) - 1.4.0-0

The packages in the `pointcloud_to_laserscan` repository were released into the `kinetic` distro by running `/usr/local/bin/bloom-release -r kinetic -t kinetic pointcloud_to_laserscan --edit` on `Tue, 14 Nov 2017 22:24:50 -0000`

The `pointcloud_to_laserscan` package was released.

Version of package(s) in repository `pointcloud_to_laserscan`:

- upstream repository: https://github.com/ros-perception/pointcloud_to_laserscan.git
- release repository: https://github.com/ros-gbp/pointcloud_to_laserscan-release.git
- rosdistro version: `1.3.1-0`
- old version: `1.3.1-0`
- new version: `1.4.0-0`

Versions of tools used:

- bloom version: `0.5.26`
- catkin_pkg version: `0.3.9`
- rosdep version: `0.11.8`
- rosdistro version: `0.6.2`
- vcstools version: `0.1.39`


## pointcloud_to_laserscan (indigo) - 1.4.0-0

The packages in the `pointcloud_to_laserscan` repository were released into the `indigo` distro by running `/usr/local/bin/bloom-release -r indigo -t indigo pointcloud_to_laserscan --edit` on `Tue, 14 Nov 2017 22:23:52 -0000`

The `pointcloud_to_laserscan` package was released.

Version of package(s) in repository `pointcloud_to_laserscan`:

- upstream repository: https://github.com/ros-perception/pointcloud_to_laserscan.git
- release repository: https://github.com/ros-gbp/pointcloud_to_laserscan-release.git
- rosdistro version: `1.3.1-0`
- old version: `1.3.1-0`
- new version: `1.4.0-0`

Versions of tools used:

- bloom version: `0.5.26`
- catkin_pkg version: `0.3.9`
- rosdep version: `0.11.8`
- rosdistro version: `0.6.2`
- vcstools version: `0.1.39`


## pointcloud_to_laserscan (lunar) - 1.4.0-0

The packages in the `pointcloud_to_laserscan` repository were released into the `lunar` distro by running `/usr/local/bin/bloom-release -r lunar -t lunar pointcloud_to_laserscan --edit` on `Tue, 14 Nov 2017 21:55:47 -0000`

The `pointcloud_to_laserscan` package was released.

Version of package(s) in repository `pointcloud_to_laserscan`:

- upstream repository: https://github.com/ros-perception/pointcloud_to_laserscan.git
- release repository: https://github.com/ros-gbp/pointcloud_to_laserscan-release.git
- rosdistro version: `1.3.1-0`
- old version: `1.3.1-0`
- new version: `1.4.0-0`

Versions of tools used:

- bloom version: `0.6.1`
- catkin_pkg version: `0.3.9`
- rosdep version: `0.11.8`
- rosdistro version: `0.6.2`
- vcstools version: `0.1.39`


## pointcloud_to_laserscan (lunar) - 1.3.1-0

The packages in the `pointcloud_to_laserscan` repository were released into the `lunar` distro by running `/usr/local/bin/bloom-release -r lunar -t lunar pointcloud_to_laserscan -y` on `Wed, 26 Apr 2017 23:07:30 -0000`

The `pointcloud_to_laserscan` package was released.

Version of package(s) in repository `pointcloud_to_laserscan`:

- upstream repository: https://github.com/ros-perception/pointcloud_to_laserscan.git
- release repository: unknown
- rosdistro version: `null`
- old version: `null`
- new version: `1.3.1-0`

Versions of tools used:

- bloom version: `0.5.26`
- catkin_pkg version: `0.3.1`
- rosdep version: `0.11.5`
- rosdistro version: `0.6.2`
- vcstools version: `0.1.39`


## pointcloud_to_laserscan (kinetic) - 1.3.1-0

The packages in the `pointcloud_to_laserscan` repository were released into the `kinetic` distro by running `/usr/local/bin/bloom-release -r kinetic -t kinetic pointcloud_to_laserscan -y` on `Wed, 26 Apr 2017 23:05:54 -0000`

The `pointcloud_to_laserscan` package was released.

Version of package(s) in repository `pointcloud_to_laserscan`:

- upstream repository: https://github.com/ros-perception/pointcloud_to_laserscan.git
- release repository: https://github.com/ros-gbp/pointcloud_to_laserscan-release.git
- rosdistro version: `1.3.0-1`
- old version: `1.3.0-1`
- new version: `1.3.1-0`

Versions of tools used:

- bloom version: `0.5.26`
- catkin_pkg version: `0.3.1`
- rosdep version: `0.11.5`
- rosdistro version: `0.6.2`
- vcstools version: `0.1.39`


## pointcloud_to_laserscan (indigo) - 1.3.1-0

The packages in the `pointcloud_to_laserscan` repository were released into the `indigo` distro by running `/usr/local/bin/bloom-release -r indigo -t indigo pointcloud_to_laserscan -y` on `Wed, 26 Apr 2017 23:04:28 -0000`

The `pointcloud_to_laserscan` package was released.

Version of package(s) in repository `pointcloud_to_laserscan`:

- upstream repository: https://github.com/ros-perception/pointcloud_to_laserscan.git
- release repository: https://github.com/ros-gbp/pointcloud_to_laserscan-release.git
- rosdistro version: `1.3.0-0`
- old version: `1.3.0-0`
- new version: `1.3.1-0`

Versions of tools used:

- bloom version: `0.5.26`
- catkin_pkg version: `0.3.1`
- rosdep version: `0.11.5`
- rosdistro version: `0.6.2`
- vcstools version: `0.1.39`


## pointcloud_to_laserscan (kinetic) - 1.3.0-1

The packages in the `pointcloud_to_laserscan` repository were released into the `kinetic` distro by running `/usr/bin/bloom-release -t kinetic -r kinetic pointcloud_to_laserscan` on `Wed, 26 Oct 2016 21:48:31 -0000`

The `pointcloud_to_laserscan` package was released.

Version of package(s) in repository `pointcloud_to_laserscan`:

- upstream repository: https://github.com/ros-perception/pointcloud_to_laserscan.git
- release repository: unknown
- rosdistro version: `null`
- old version: `1.3.0-0`
- new version: `1.3.0-1`

Versions of tools used:

- bloom version: `0.5.23`
- catkin_pkg version: `0.2.10`
- rosdep version: `0.11.5`
- rosdistro version: `0.5.0`
- vcstools version: `0.1.39`


## pointcloud_to_laserscan (kinetic) - 1.3.0-0

The packages in the `pointcloud_to_laserscan` repository were released into the `kinetic` distro by running `/usr/bin/bloom-release -t kinetic -r kinetic pointcloud_to_laserscan` on `Wed, 26 Oct 2016 21:45:53 -0000`

The `pointcloud_to_laserscan` package was released.

Version of package(s) in repository `pointcloud_to_laserscan`:

- upstream repository: https://github.com/ros-perception/pointcloud_to_laserscan.git
- release repository: unknown
- rosdistro version: `null`
- old version: `null`
- new version: `1.3.0-0`

Versions of tools used:

- bloom version: `0.5.23`
- catkin_pkg version: `0.2.10`
- rosdep version: `0.11.5`
- rosdistro version: `0.5.0`
- vcstools version: `0.1.39`


## pointcloud_to_laserscan (jade) - 1.3.0-0

The packages in the `pointcloud_to_laserscan` repository were released into the `jade` distro by running `/usr/bin/bloom-release -r jade -t jade pointcloud_to_laserscan` on `Tue, 09 Jun 2015 18:27:57 -0000`

The `pointcloud_to_laserscan` package was released.

Version of package(s) in repository `pointcloud_to_laserscan`:
- upstream repository: https://github.com/ros-perception/pointcloud_to_laserscan.git
- release repository: unknown
- rosdistro version: `null`
- old version: `null`
- new version: `1.3.0-0`

Versions of tools used:
- bloom version: `0.5.20`
- catkin_pkg version: `0.2.8`
- rosdep version: `0.11.2`
- rosdistro version: `0.4.2`
- vcstools version: `0.1.36`


## pointcloud_to_laserscan (indigo) - 1.3.0-0

The packages in the `pointcloud_to_laserscan` repository were released into the `indigo` distro by running `/usr/bin/bloom-release -r indigo -t indigo pointcloud_to_laserscan` on `Tue, 09 Jun 2015 18:23:38 -0000`

The `pointcloud_to_laserscan` package was released.

Version of package(s) in repository `pointcloud_to_laserscan`:
- upstream repository: https://github.com/ros-perception/pointcloud_to_laserscan.git
- release repository: unknown
- rosdistro version: `null`
- old version: `null`
- new version: `1.3.0-0`

Versions of tools used:
- bloom version: `0.5.20`
- catkin_pkg version: `0.2.8`
- rosdep version: `0.11.2`
- rosdistro version: `0.4.2`
- vcstools version: `0.1.36`



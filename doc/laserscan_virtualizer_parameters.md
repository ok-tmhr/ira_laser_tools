# Laserscan Virtualizer Parameters

Default Config

```yaml
laserscan_virtualizer:
  ros__parameters:
    angle_increment: 0.0058
    angle_max: 3.14
    angle_min: -3.14
    base_frame: base_link
    cloud_topic: /cloud_pcd
    output_laser_topic: /scan
    range_max: 25.0
    range_min: 0.0
    scan_time: 0.0
    virtual_laser_scan: [scan1, scan2]
```

The `laserscan_virtualizer` node converts an input `sensor_msgs/msg/PointCloud2`
message into one or more virtual `sensor_msgs/msg/LaserScan` messages. Each frame
listed in `virtual_laser_scan` must have a TF transform available from the
incoming point cloud frame.

## base_frame

Reference frame used by the TF message filter before processing incoming point clouds.

- Type: `string`
- Default Value: "base_link"

_Constraints:_

- parameter is not empty

## cloud_topic

Input `PointCloud2` topic.

- Type: `string`
- Default Value: "/cloud_pcd"

_Constraints:_

- parameter is not empty

## output_laser_topic

Topic used for virtual LaserScan output.

When this value is not empty, every virtual scan publisher uses this same topic
name. When this value is empty, each virtual scan is published on the
corresponding frame name from `virtual_laser_scan`.

- Type: `string`
- Default Value: "/scan"

_Constraints:_

- parameter is not empty

## virtual_laser_scan

List of virtual laser frame names to generate from the input point cloud.

For each configured frame, the node looks up a transform from the input cloud
frame to that virtual laser frame and publishes a LaserScan in that frame.

- Type: `string_array`
- Default Value: {"scan1", "scan2"}

_Constraints:_

- parameter is not empty
- contains no duplicates

## angle_min

Minimum angle of the generated virtual scan, in radians.

- Type: `double`
- Default Value: -3.14

_Constraints:_

- parameter must be within bounds -3.141592653589793 to 3.141592653589793

## angle_max

Maximum angle of the generated virtual scan, in radians.

- Type: `double`
- Default Value: 3.14

_Constraints:_

- parameter must be within bounds -3.141592653589793 to 3.141592653589793

## angle_increment

Angular resolution of the generated virtual scan, in radians.

- Type: `double`
- Default Value: 0.0058

_Constraints:_

- greater than 0.0

## scan_time

Time duration of a complete scan, in seconds.

- Type: `double`
- Default Value: 0.0

_Constraints:_

- greater than or equal to 0.0

## range_min

Minimum valid range for measurements, in meters.

- Type: `double`
- Default Value: 0.0

_Constraints:_

- greater than or equal to 0.0

## range_max

Maximum valid range for measurements, in meters.

- Type: `double`
- Default Value: 25.0

_Constraints:_

- greater than or equal to 0.0

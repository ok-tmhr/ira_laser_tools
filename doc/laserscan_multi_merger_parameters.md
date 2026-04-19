# Laserscan Multi Merger Parameters

Default Config

```yaml
laserscan_multi_merger:
  ros__parameters:
    angle_increment: 0.0058
    angle_max: 3.14
    angle_min: -3.14
    cloud_destination_topic: /merged_cloud
    destination_frame: cart_frame
    laserscan_topics: [/scan1, /scan2]
    publish_pointcloud: true
    queue_size: 10
    range_max: 25.0
    range_min: 0.0
    scan_destination_topic: /scan_multi
    scan_time: 0.0
    time_increment: 0.0
```

## destination_frame

Target frame for merged LaserScan output

- Type: `string`
- Default Value: "cart_frame"

_Constraints:_

- parameter is not empty

_Additional Constraints:_

## cloud_destination_topic

Topic name for publishing merged PointCloud2 output

- Type: `string`
- Default Value: "/merged_cloud"

_Constraints:_

- parameter is not empty

_Additional Constraints:_

## scan_destination_topic

Topic name for publishing merged LaserScan output

- Type: `string`
- Default Value: "/scan_multi"

_Constraints:_

- parameter is not empty

_Additional Constraints:_

## laserscan_topics

List of input LaserScan topic names (must contain 2-4 topics)

- Type: `string_array`
- Default Value: {"/scan1", "/scan2"}

_Constraints:_

- contains no duplicates
- length is less than 1
- length is greater than 5

_Additional Constraints:_

## angle_min

Minimum angle of merged scan (radians)

- Type: `double`
- Default Value: -3.14

_Constraints:_

- parameter must be within bounds -3.141592653589793

_Additional Constraints:_

## angle_max

Maximum angle of merged scan (radians)

- Type: `double`
- Default Value: 3.14

_Constraints:_

- parameter must be within bounds -3.141592653589793

_Additional Constraints:_

## angle_increment

Angular resolution of merged scan (radians)

- Type: `double`
- Default Value: 0.0058

_Constraints:_

- greater than 0.0

_Additional Constraints:_

## scan_time

Time duration of a complete scan (seconds)

- Type: `double`
- Default Value: 0.0

_Constraints:_

- greater than or equal to 0.0

_Additional Constraints:_

## range_min

Minimum valid range for measurements (meters)

- Type: `double`
- Default Value: 0.0

_Constraints:_

- greater than or equal to 0.0

_Additional Constraints:_

## range_max

Maximum valid range for measurements (meters)

- Type: `double`
- Default Value: 25.0

_Constraints:_

- greater than or equal to 0.0

_Additional Constraints:_

## queue_size

Queue size for message filters and synchronizers

- Type: `int`
- Default Value: 10

_Constraints:_

- greater than 0

_Additional Constraints:_

## time_increment

Time between angle measurements (seconds)

- Type: `double`
- Default Value: 0.0

_Constraints:_

- greater than or equal to 0.0

_Additional Constraints:_

## publish_pointcloud

Enable publishing of merged PointCloud2 output

- Type: `bool`
- Default Value: true

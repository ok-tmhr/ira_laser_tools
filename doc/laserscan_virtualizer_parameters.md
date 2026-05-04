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

## base_frame

Reference frame which laser(s) are related to

- Type: `string`
- Default Value: "base_link"

_Constraints:_

- parameter is not empty

_Additional Constraints:_

## cloud_topic

Input point cloud topic

- Type: `string`
- Default Value: "/cloud_pcd"

_Constraints:_

- parameter is not empty

_Additional Constraints:_

## output_laser_topic

Virtual laser output topic, leave empty to publish on virtual laser names

- Type: `string`
- Default Value: "/scan"

## virtual_laser_scan

List of virtual laser scan frames

- Type: `string_array`
- Default Value: ["scan1", "scan2"]

_Constraints:_

- parameter is not empty
- contains no duplicates

_Additional Constraints:_

## angle_min

Minimum angle of virtual scan (radians)

- Type: `double`
- Default Value: -3.14

_Constraints:_

- parameter must be within bounds -3.141592653589793

_Additional Constraints:_

## angle_max

Maximum angle of virtual scan (radians)

- Type: `double`
- Default Value: 3.14

_Constraints:_

- parameter must be within bounds -3.141592653589793

_Additional Constraints:_

## angle_increment

Angular resolution of virtual scan (radians)

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

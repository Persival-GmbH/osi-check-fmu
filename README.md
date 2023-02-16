# OSI Check FMU
This FMU checks if fields are missing in a received SensorData.
The required fields are defined in an osi_check.txt file located in the /tmp folder.
An example file can be found in example_check_file/.

Currently able to check:
- moving_object
- moving_object.base
- moving_object.base.position
- moving_object.base.orientation
- moving_object.base.velocity
- moving_object.base.acceleration
- moving_object.base.orientation_rate
- moving_object.base.orientation_acceleration
- moving_object.base.base_polygon
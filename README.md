# cartesio_acceleration_support
Plugins to extend CartesIO 2.0 to acceleration level control

## Contents
OpenSot task and constraint adapters for CartesIO 2.0, including
 - Cartesian task
 - Interaction task
 - Postural task
 - Joint limits and Velocity limits

## How to use
Once compiled and installed, set the `lib_name` field of all tasks/constraint to the installed
library name, i.e.

```yaml
Mytask:
   lib_name: libcartesio_acceleration_support.so
   ...
   ...
```

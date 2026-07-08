# SpawnMotionModel

This plugin subscribes to one Ignition topic. When it receives a command, it spawns one model and makes it move forever along a closed polygon.

How it works:
- The command message provides:
  - the model to spawn
  - the polygon waypoints
  - the scalar velocity
- The plugin spawns the model at the first waypoint.
- It moves in a straight line to the next waypoint.
- When it reaches the last waypoint, it goes back to the first one.
- This repeats forever.

Topic:
- Default topic: `/world/<world_name>/spawn_motion_model`
- The topic is hardcoded in the plugin.

Example plugin block:

```xml
<world name="demo">
  <plugin
    filename="libSpawnMotionModel.so"
    name="ignition::gazebo::systems::SpawnMotionModel">
  </plugin>
</world>
```

Command message type:
- `ignition.msgs.Pose_V`

Command message format:
- `header.data[key=model_name]`: model name to spawn from `model://...`
- `header.data[key=velocity]`: speed in meters per second
- `pose`: repeated poses whose positions are the polygon waypoints

Example publish command:

```bash
ign topic -t /world/world_demo/spawn_motion_model \
  -m ignition.msgs.Pose_V \
  -p 'header: {
        data: { key: "model_name" value: "movai_cart" }
        data: { key: "velocity" value: "0.8" }
      }
      pose: { position: { x: -4 y: -2 z: 0 } }
      pose: { position: { x: 0 y: -2 z: 0 } }
      pose: { position: { x: 0 y: 0 z: 0 } }
      pose: { position: { x: -4 y: 0 z: 0 } }'
```

Notes:
- Load this plugin as a world plugin, not inside a model.
- Logs are printed by Ignition Gazebo in the simulator terminal, not by the `ign topic` command itself.
- Send at least two poses in the command.
- The plugin uses one internal spawned instance named `spawn_motion_model`.
- If a new command arrives after the model is already spawned, the plugin reuses that same spawned model and restarts the path from the first waypoint.
- `roll` and `pitch` are optional and default to `0`.
- `request_timeout_ms` is optional and defaults to `100`.
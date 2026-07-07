# README

How to use it:
- run simulation with plugin
- In terminal run:
    ```bash
        ign topic -t /apply_force_to_model_topic -m ignition.msgs.EntityWrench -p "entity: {name: 'cart1', id: 1, type: MODEL} wrench: {force: {x: -1000.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}"
    ```


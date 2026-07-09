# README

How to use it:
- run simulation with plugin
- Send an ignition.msgs.EntityWrench where entity.name is the target model in the sim and wrench is the force/torque to apply.
- In terminal run:
    ```bash
        ign topic -t /apply_force_to_model_topic -m ignition.msgs.EntityWrench -p "entity: {name: 'cart1', id: 1, type: MODEL} wrench: {force: {x: -1000.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}"
    ```


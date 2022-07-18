Battery Station Controller Ignition SYSTEM Plugin.

This plugin can identify and trigger the charge of a linear battery plugin attached to a robot in the simulation.
It is subscribing to a ContactMsg from the contact sensor on the charge station and call the services to charge or discharge
the battery.
The robot model used in simulation must have the Ignition Linear Battery plugin.

## Build

    Inside Ignition-Container run:

    ```
    mkdir -p /models_database/movai_ign_plugins/battery_station/build
    cd /models_database/movai_ign_plugins/battery_station/build
    cmake ..
    make
    cd ..
    rm -rf build
    ```
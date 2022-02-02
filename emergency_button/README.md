Emergency Button Ignition GUI Plugin.

This tool is to select a model in simulation and start to publish emergency mode msgs.

## Build

    Inside Ignition-Container run:

    ```
    mkdir -p /models_database/movai_ign_plugins/emergency_button/build
    cd /models_database/movai_ign_plugins/emergency_button/build
    cmake ..
    make
    cd ..
    rm -rf build
    ```
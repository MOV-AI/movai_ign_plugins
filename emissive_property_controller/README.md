Emissive Property Controller Ignition GUI Plugin.

This tool is to change the material color of a Model_Link inside the simulation in runtime.
It is subscribing to a StringMsg on /model/material
The message must follow the structure:
"Model_Name::Link_Name-RED-GREEN-BLUE-ALPHA"
Also, this is a GUI plugin that needs to be loaded in the simulation to enable the color change.

## Build

    Inside Ignition-Container run:

    ```
    mkdir -p /models_database/movai_ign_plugins/emissive_property_controller/build
    cd /models_database/movai_ign_plugins/emissive_property_controller/build
    cmake ..
    make
    cd ..
    rm -rf build
    ```
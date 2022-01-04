World Launcher Ignition GUI tool.

This tool is to select and start simulation worlds that is on FUEL repository or in Local directory.

## Build

    Inside Ignition-Container run:

    ```
    mkdir -p /models_database/movai_ign_plugins/world_launcher/build
    cd /models_database/movai_ign_plugins/world_launcher/build
    cmake ..
    make
    cd ..
    rm -rf build
    ```

## Run

    Inside Ignition-Container run:

    ```
    ign gui -s WorldLauncher
    ```
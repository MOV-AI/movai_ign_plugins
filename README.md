# movai_ign_plugins
Repository for holding the system and/or gui plugins for the ignition simulator

# World Launcher Plugin
## Install from Debian package (APT)

```
curl -fsSL https://artifacts.aws.cloud.mov.ai/repository/movai-applications/gpg | sudo apt-key add -  # add key
sudo add-apt-repository "deb [arch=all] https://artifacts.aws.cloud.mov.ai/repository/ppa-public main main" # add sources
sudo apt update
sudo apt install movai-ign-plugin-world-launcher # install plugin. currntly gets installed in: /movai_ign_plugins/gui/
```

## Run 
``` 
export IGN_GUI_PLUGIN_PATH=/movai_ign_plugins/gui #where is the libWorldLauncher.so file
ign gui -s WorldLauncher 
```

Note: MOVAI Flow IDE - Spawner Container [https://github.com/MOV-AI/containers-spawner-base] is expecting Ignition Fortress runs with ENV `IGN_PARTITION=movai_ce_flow`. 


## Build from sources


# emergency_button Plugin
## Install from Debian package (APT)
## Run
## Build from sources

# emissive_property_controller  Plugin
## Install from Debian package (APT)
## Run 
## Build from sources

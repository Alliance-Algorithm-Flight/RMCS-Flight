# Development of UAV

This branch contains runtime code for the UAV.

## External Dependencies

Flight controller takes advantage of the SLAM runtime [`rmcs_slam`](https://Alliance-Algorithm/rmcs_slam).
Remember to sync the submodule.

## Simulation

To run the simulation inside of the docker container,
you have to port the internal display to the external real machine.

It's suggested to use VNC, and the repo provides some script for it.

1.  Start the VNC: run `sudo ./start-vuc.sh`.
    This will export the GUI to `localhost:6080`.
2.  Join the env-vars: run `source px4.zshenv`, and run your make commands.
3.  Open `localhost:6080` from the external browser, and login into the noVNC instance.

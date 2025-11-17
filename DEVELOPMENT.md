# Development of UAV

This branch contains runtime code for the UAV.

## External Dependencies

Flight controller takes advantage of the SLAM runtime [`rmcs_slam`](https://Alliance-Algorithm/rmcs_slam).
Remember to sync the submodule.

## Simulation

To run the simulation inside of the docker container,
you have to port the internal display to the external real machine.

Note: Join the env-vars: run `source display.zshenv`, and run your make commands to make the simulator clearer.

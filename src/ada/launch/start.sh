#!/usr/bin/env bash

# Make entrypoint scripts executable
chmod +x f1tenth_entrypoint.sh perception_entrypoint.sh

# give docker permission to use X
xhost +si:localuser:root

# Start the containers using docker compose
docker compose up 

#!/bin/bash
set -e

# Enable container access to X11
xhost +SI:localuser:root
xhost +local:

cleanup() 
{
  echo ""
  echo ">>> Stopping Docker Compose..."
  docker compose down --remove-orphans
}

trap cleanup INT TERM

echo ">>> Running docker compose up..."
docker compose up

cleanup

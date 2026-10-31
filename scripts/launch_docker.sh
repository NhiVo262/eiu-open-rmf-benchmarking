#!/bin/bash
set -e

echo ">>> Allow Docker (root) to access X11"
xhost +SI:localuser:root

cleanup() {
  echo ""
  echo ">>> Stopping Docker Compose..."
  docker compose down --remove-orphans

  echo ">>> Revoking X11 access"
  xhost -SI:localuser:root
}

trap cleanup INT TERM

echo ">>> Running docker compose up..."
docker compose up

cleanup

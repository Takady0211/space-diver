#!/bin/bash

# Define source and destination directories
SCRIPT_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")

SOURCE_DIR="$SCRIPT_DIR/meshes"
DEST_DIR="$HOME/.gazebo/models/spacediver"

# Ensure the destination directory exists
mkdir -p "$DEST_DIR" || { echo "Failed to create destination directory $DEST_DIR"; exit 1; }

# Copy the entire meshes directory to the Gazebo models directory
cp -r "$SOURCE_DIR" "$DEST_DIR" || { echo "Failed to copy files from $SOURCE_DIR to $DEST_DIR"; exit 1; }

echo "Meshes copied successfully to $DEST_DIR!"

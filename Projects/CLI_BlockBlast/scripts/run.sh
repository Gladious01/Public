#!/bin/bash

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Change to local directory
cd "$SCRIPT_DIR" || exit

echo "Building the project..."

# Create the build directory if it doesn't exist
if [ ! -d "../build" ]; then
    mkdir ../build
fi

# Compile the source files using g++ and output to the build directory
g++ -o ../build/CLI_Blaster ../src/*.cpp

# Check if the build was successful
if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

# Run the executable from the build directory
echo "Running the program..."
./..//build/CLI_Blaster

#!/bin/sh

# Exit on the first error
set -e

# Setup Environment
rm -rf build
rm -rf native_build

# Build without ros wrapper to catch warnings and errors
sphinx-build -W -b html . native_build

# Build
#rosdoc_lite -o build .

# Validate if the HTML is valid (i.e. no missing links, images, etc.)
htmlproofer ./build --only-4xx --http-status-ignore 429 --check-html --file-ignore ./build/html/genindex.html,./build/html/search.html,./build/html/index-msg.html --alt-ignore '/.*/' --url-ignore '#'

# Run
xdg-open ./build/html/index.html

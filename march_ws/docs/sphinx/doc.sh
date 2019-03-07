#!/usr/bin/env bash

cwd=$(pwd)
base_output_dir="$cwd/_build/"

for directory in $(find -O3 -L ../../src/ -name "CMakeLists.txt")
do

    if [[ "$directory" == *"march_"* ]]
    then
            package_name=$(basename $(dirname "${directory}"))
            dir_name=$(dirname "${directory}")
            output_dir="$base_output_dir$package_name"
            doxyfile="$dir_name/docs/Doxyfile"

            if [ ! -f $doxyfile ]; then
                echo "Skipping package $package_name, no Doxyfile found at $doxyfile."
            else
                echo "Building documentation for package $package_name, Doxyfile found at $doxyfile."
                cd "$dir_name/docs"

                ( cat Doxyfile ; echo "OUTPUT_DIRECTORY=$output_dir" ) | doxygen -
                cd "$cwd"
                mkdir -p _build/xml/$package_name
                mv  _build/$package_name/xml/*  _build/xml/$package_name
                rmdir _build/$package_name/xml
                rmdir _build/$package_name/
            fi
    fi
done

echo $PWD
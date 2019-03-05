#!/usr/bin/env bash

cwd=$(pwd)
base_output_dir="$cwd/build/"

for directory in $(find -O3 -L ../../src/ -name "CMakeLists.txt")
do

    if [[ "$directory" == *"march_"* ]]
    then
            package_name=$(basename $(dirname "${directory}"))
            echo "$package_name"
            dir_name=$(dirname "${directory}")
            echo "$dir_name"
            output_dir="$base_output_dir$package_name"
            echo "$output_dir"
            doxyfile="$dir_name/docs/Doxyfile"
            echo $doxyfile

            if [ ! -f $doxyfile ]; then
                echo "File not found!"
            else
                echo "File found!!!!!!!!!!"
                cd "$dir_name/docs"

                ( cat Doxyfile ; echo "OUTPUT_DIRECTORY=$output_dir" ) | doxygen -
                cd "$cwd"
                mkdir build/html/$package_name
                mv  build/$package_name/html/*  build/html/$package_name
                rmdir build/$package_name/html
                rmdir build/$package_name/
            fi
            echo ""

    fi
done
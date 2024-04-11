#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
BIN_DIR="$HOME/.local/bin"

if [[ ":$PATH:" == *":$BIN_DIR:"* ]]; then
    echo "Installing to $BIN_DIR"
else
    echo "Installation dir not in PATH! Aborting."
    exit 1
fi

declare -a files=("build-ros" "clean-ros")

for file in "${files[@]}"
do
    src="$SCRIPT_DIR/$file"
    dest="$BIN_DIR/$file"
    cp "$src" "$dest"
    echo "installed: $file -> $dest"
done

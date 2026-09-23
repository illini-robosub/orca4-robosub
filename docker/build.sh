#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR

docker build --platform linux/amd64 -f "$DIR/Dockerfile" -t orca4:latest ..

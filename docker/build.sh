#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR

docker build --network=host -f $DIR/Dockerfile -t orca4:latest .. # added --network=host

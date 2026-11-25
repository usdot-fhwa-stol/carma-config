#!/bin/bash

# Copyright (C) 2018-2026 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0

USERNAME=usdotfhwastol
IMAGE=carma-config
cd "$(dirname "$0")" || exit 1
DIR_NAME=${PWD##*/}
CONFIG_NAME=$(echo "$DIR_NAME" | sed 's/_/-/g')

while [[ $# -gt 0 ]]; do
    case "$1" in
        -d|--develop)
            USERNAME=usdotfhwastoldev
            TAG=develop
            shift
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

if [[ -z "$TAG" ]]; then
    TAG="$("../docker/get-system-version.sh")-$CONFIG_NAME"
else
    TAG="$TAG-$CONFIG_NAME"
fi

echo "Building $USERNAME/$IMAGE:$TAG"

docker build --no-cache -t "$USERNAME/$IMAGE:$TAG" \
    --build-arg VERSION="$TAG" \
    --build-arg VCS_REF="$(git rev-parse --short HEAD)" \
    --build-arg CONFIG_NAME="carma-config:$CONFIG_NAME" \
    --build-arg BUILD_DATE="$(date -u +'%Y-%m-%dT%H:%M:%SZ')" .

echo "Built $USERNAME/$IMAGE:$TAG"

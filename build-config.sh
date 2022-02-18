#!/bin/bash

#  Copyright (C) 2022 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

#
# GENERIC Build Script for CARMA Configuration Images
#
# Do not run this script itself, rather symlink this script into the individual
# configuration folders below and invoke it there to build the appropriate config
# image using docker build. Automatically acquires folder name and system version
# and passes neessary data into the docker build process.

if [[ -z "$1" ]] ; then
    echo "No argument supplied. User must provide path to desired config to build."
    exit 1
fi

CONFIG_PATH=$1

CONFIG_PATH=$(realpath ${CONFIG_PATH})

echo "CONFIG_PATH: ${CONFIG_PATH}"

if [[ ! -d "${CONFIG_PATH}" ]] ; then 
    echo "Specified config folder ${CONFIG_PATH} does not exist."
    exit 1
fi

# Move arguments forward so they can be passed to build-image.sh
shift

current_files=()

for entry in "$CONFIG_PATH"/*
do
    echo "entry: ${entry}"
    current_files+=( $(basename -- ${entry}) )
done

TEMPLATE_PATH=template_config/template_config
TEMPLATE_PATH=$(realpath ${TEMPLATE_PATH})

echo "TEMPLATE_PATH: ${TEMPLATE_PATH}"

if [[ ! -d "${TEMPLATE_PATH}" ]] ; then 
    echo "Could not fine template_config. You must run this script from the same folder."
    exit 1
fi

added_files=()
for template_file in "$TEMPLATE_PATH"/*
do
    template_file=$(basename -- ${template_file})
    echo "template_file: ${template_file}"
    file_exists=false
    for existing_file in ${current_files[@]}; do
        
        if [[ ${existing_file} == ${template_file} ]] ; then
            echo "existing_file: ${existing_file}"
            file_exists=true
            break
        fi
    done

    if [[ ${file_exists} == true ]] ; then
        continue
    fi

    cp -r ${TEMPLATE_PATH}/${template_file} $CONFIG_PATH/${template_file}
    added_files+=( $CONFIG_PATH/${template_file} )
done

# Build the config
current_dir=$PWD
cd $CONFIG_PATH
./build-image.sh "$@"
cd ${PWD}

# Cleanup added files to prevent accidental commits
for added_file in ${added_files[@]}; do
    rm ${added_file}
done

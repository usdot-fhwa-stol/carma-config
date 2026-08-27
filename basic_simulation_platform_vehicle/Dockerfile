# Copyright (C) 2018-2026 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0

FROM busybox:latest

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"
ARG CONFIG_NAME="carma-config:unspecified"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name=${CONFIG_NAME}
LABEL org.label-schema.description="Configuration for a CARMA Platform simulation vehicle"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-config"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

COPY docker-compose.yml /root/vehicle/config/docker-compose.yml
COPY carma_docker.launch.py /root/vehicle/config/carma_docker.launch.py
COPY drivers.launch.py /root/vehicle/config/drivers.launch.py
COPY VehicleConfigParams.yaml /root/vehicle/config/VehicleConfigParams.yaml
COPY SubsystemControllerParams.yaml /root/vehicle/config/SubsystemControllerParams.yaml
COPY carma.config.js /root/vehicle/config/carma.config.js
COPY carma.env /root/vehicle/config/carma.env
COPY carma_rosconsole.conf /root/vehicle/config/carma_rosconsole.conf
COPY carma /root/vehicle/config/carma

VOLUME /opt/carma/vehicle/config
RUN mkdir -p /opt/carma/vehicle/config

CMD ["cp", "-a", "/root/vehicle/config/.", "/opt/carma/vehicle/config/"]

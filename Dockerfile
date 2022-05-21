FROM ubuntu:18.04

MAINTAINER 2maz "https://github.com/2maz"

## BEGIN BUILD ARGUMENTS
# Arguments for creation of the Docker imaged,
# passed via --build-arg

ENV PKG_NAME="planning/templ"

# Github repos
ENV PKG_PROJECT="rock-planning"
ENV PKG_BUILDCONF="planning-templ-buildconf"
# Default buildconf branch
ENV PKG_BUILDCONF_BRANCH=main


# Optional arguments
ARG PKG_BRANCH="main"
ENV PKG_BRANCH=${PKG_BRANCH}

ARG PKG_PULL_REQUEST="false"
ENV PKG_PULL_REQUEST=${PKG_PULL_REQUEST}
## END ARGUMENTS

COPY .ci/prepare-docker.sh prepare-docker.sh
RUN ./prepare-docker.sh

USER docker
WORKDIR /home/docker

ENV LANG de_DE.UTF-8
ENV LANG de_DE:de
ENV LC_ALL de_DE.UTF-8
ENV SHELL /bin/bash

RUN git config --global user.email "rock-users@dfki.de"
RUN git config --global user.name "Rock CI"

RUN wget https://raw.githubusercontent.com/rock-core/autoproj/master/bin/autoproj_bootstrap

RUN mkdir -p /home/docker/rock_test
WORKDIR /home/docker/rock_test

# Use the existing seed configuration
COPY --chown=docker .ci/autoproj-config.yml seed-config.yml

ENV AUTOPROJ_BOOTSTRAP_IGNORE_NONEMPTY_DIR 1
ENV AUTOPROJ_NONINTERACTIVE 1
RUN if git ls-remote --heads https://github.com/${PKG_PROJECT}/${PKG_BUILDCONF}.git | grep "${PKG_BRANCH}" > /dev/null; then export PKG_BUILDCONF_BRANCH="${PKG_BRANCH}"; fi; ruby /home/docker/autoproj_bootstrap git https://github.com/$PKG_PROJECT/$PKG_BUILDCONF.git branch=${PKG_BUILDCONF_BRANCH} --seed-config=seed-config.yml
RUN sed -i "s#rock\.core#knowledge_reasoning/moreorg\n    - ${PKG_NAME}#g" autoproj/manifest
RUN sed -i "s#testing#${PKG_BRANCH}#g" autoproj/overrides.yml
COPY --chown=docker .ci/deb_blacklist.yml autoproj/deb_blacklist.yml

### Activate testing
RUN /bin/bash -c "source env.sh; autoproj test enable ${PKG_NAME}"
#### Update
RUN /bin/bash -c "source env.sh; autoproj update; autoproj osdeps"
RUN /bin/bash -c "source env.sh; autoproj osdeps; amake"

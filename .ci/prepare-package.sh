#!/bin/bash

CURRENT_DIR=$PWD
PACKAGE_DIR=$1
cd $PACKAGE_DIR

if [ "$PKG_PULL_REQUEST" = "false" ]; then
    echo "Using branch: ${PKG_BRANCH}"
    git fetch -a origin
    git checkout -b docker_test_pr origin/${PKG_BRANCH}
fi
## Check if this a pull request and change to pull request
## accordingly
if [ "$PKG_PULL_REQUEST" != "false" ]; then
    echo "Using pull request: ${PKG_PULL_REQUEST}"
    cd "${PKG_NAME}"
    git fetch origin pull/${PKG_PULL_REQUEST}/head:docker_test_pr
    git checkout docker_test_pr
    export PKG_BRANCH=${PKG_PULL_REQUEST_BRANCH}
fi
git branch -a
git remote -v
cd $CURRENT_DIR



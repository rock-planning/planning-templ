#!/bin/bash

apt update
apt upgrade -y

export DEBIAN_FRONTEND=noninteractive
apt install -y ruby ruby-dev wget tzdata locales g++ autotools-dev make cmake sudo git python3 pkg-config vim

echo "Europe/Berlin" > /etc/timezone
dpkg-reconfigure -f noninteractive tzdata

export LANGUAGE=de_DE.UTF-8
export LANG=de_DE.UTF-8
export LC_ALL=de_DE.UTF-8

locale-gen de_DE.UTF-8
dpkg-reconfigure locales

useradd -ms /bin/bash docker
echo "docker ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

git config --global user.email "rock-users@dfki.de"
git config --global user.name "Rock CI"

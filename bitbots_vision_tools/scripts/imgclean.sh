#!/usr/bin/env bash

# This is a small bash script, to quickly sort a directory of images
# using feh with shortcuts.
# Start this inside the directory of images to sort.
#
# Press key:
# 1 -> move current image to $Trash subdirectory

Trash="unusable"

debug="./debug/$Trash"
labels="./labels/$Trash"

mkdir -p $debug
mkdir -p $labels

mv_debug='mv ./debug/$(basename "%f") ./debug/unusable/'
mv_labels='mv ./labels/$(basename "%f") ./labels/unusable/'

feh -Z -F -d --action1 "$mv_debug && $mv_labels" ./debug/*

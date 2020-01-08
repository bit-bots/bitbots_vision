#!/usr/bin/env bash

# This is a small bash script, to quickly sort a directory of images
# using feh with shortcuts.
# Start this inside the directory of images to sort.
#
# Press key:
# 1 -> move current image to $Trash subdirectory

Trash="unusable"

debug="./debug_$Trash"
labels="./labels_$Trash"
images="./images_$Trash"

mkdir -p $debug
mkdir -p $labels
mkdir -p $images

mv_debug='mv ./debug/$(basename "%f")'
mv_labels='mv ./labels/$(basename "%f")'
mv_images='mv ./images/$(basename "%f")'

feh -Z -F -d --action1 "$mv_debug $debug && $mv_labels $labels && $mv_images $images" ./debug/*

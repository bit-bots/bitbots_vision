#! /usr/bin/env python3
import os
import re
import yaml
import pickle
import argparse

"""
A script to combine the parameter description (documentation) of .yaml and dynamic reconfigurable .cfg files.
        This script compares the parameter descriptions of the SOURCE file with the related ones of the DESTINATION file.
        If SAVE, the DESTINATION file will be overwritten.
"""

def compare(source, destination):
    print(f"Comparing '{source}' with '{destination}'...")

    # TODO: save

    print("Finished comparing.")
    # TODO: print statistics

def save(destination, result):
    print(f"Saving to '{destination}'...")

    # TODO: save

    print("Finished saving.")

if __name__ == "__main__":
    main_description = "Combine the parameter description (documentation) of .yaml and dynamic reconfigurable .cfg files. " +\
        "This script compares the parameter descriptions of the SOURCE file with the related ones of the DESTINATION file. " +\
        "If SAVE, the DESTINATION file will be overwritten."

    parser = argparse.ArgumentParser(
        description=main_description)
    parser.add_argument("-s", "--save", action="store_true",
                        help="Save the combined output in the destination file (Default: False)")
    parser.add_argument("-S", "--source", action="store", nargs=1, type=str,
                        help="Source file of the parameter description")
    parser.add_argument("-D", "--destination", action="store", nargs=1, type=str,
                        help="Destination file of the parameter description")
    args = parser.parse_args()

    print(args) # TODO: remove

    # Error handling
    if args.source is None or args.destination is None:
        raise argparse.ArgumentError(None, "Please define a SOURCE and a DESTINATION file.")

    if os.path.samefile(os.path.realpath(args.source[0]), os.path.realpath(args.destination[0])):
        raise argparse.ArgumentError(None, "SOURCE and DESTINATION can NOT be the same file.")

    source = os.path.realpath(args.source[0])
    destination = os.path.realpath(args.destination[0])
    result = compare(source, destination)

    if args.save:
        save(destination, result)

"""
# Compatibility
try:
    input = raw_input
except:
    pass

# Argument parsing
parser = argparse.ArgumentParser()
parser.add_argument("-i", "--ignore-debug", help="Ignore debug params", action='store_true')
parser.add_argument("--no-confirm", help="Do not ask for write permission", action='store_true')
args = parser.parse_args()

# Open old config file
with open(config_path) as fp:
    data = yaml.load(fp)

# Ask user if he wants to save it
if args.no_confirm or input("\n {} parameters changed. Do you want to save? (y/n)".format(changed_params)) == "y":
    # Save new file
    with open(config_path,"w") as fp:
        yaml.dump(data, fp)
    print("Saved file")
else:
    print("Nothing has been saved!")
"""

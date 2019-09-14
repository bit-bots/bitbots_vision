#! /usr/bin/env python3
import os
import re
import yaml
import argparse

"""
A script to combine the parameter description (documentation) of .yaml and dynamic reconfigurable .cfg files.
        This script compares the parameter descriptions of the SOURCE file with the related ones of the DESTINATION file.
        If SAVE, the DESTINATION file will be overwritten.
"""


def main(source_path, destination_path):
    # Parse files for keys and associated description
    print(f"Analysing '{source_path}'...")
    if source_path.endswith(".yaml"):
        source = parse_yaml(source_path)
    elif source_path.endswith(".cfg"):
        source = parse_cfg(source_path)
    print(f"Found {len(source)} parameters in '{source_path}'.")

    print(f"\nAnalysing '{destination_path}'...")
    if destination_path.endswith(".yaml"):
        destination = parse_yaml(destination_path)
    elif destination_path.endswith(".cfg"):
        destination = parse_cfg(destination_path)
    print(f"Found {len(destination)} parameters in '{destination_path}'.")

    result = compare(source, destination)
    save(destination_path, result)


def compare(source, destination):
    print(f"\nComparing SOURCE and DESTINATION...")

    # TODO: compare

    print("Finished comparing.")
    # TODO: print statistics


def parse_yaml(file_path):
    # Search for parameters in .cfg file
    # Load line by line
    lines = tuple(open(file_path, 'r'))
    # Remove empty lines and whitespace at end of line
    parameters = [line.strip() for line in lines if line.strip() != ""]

    # Extract key and description from each parameter
    key_description = {}
    for parameter in parameters:
        # Split parameters into arguments by the # char
        arguments = re.split(r"\s*#\s*", parameter)
        # Key at argument position 0, remove value
        key = re.split(r":", arguments[0])[0]
        # Description at argument position 1, if existent
        description = ""
        if len(arguments) > 1:
            description = arguments[1]
        key_description[key] = description
    return key_description


def parse_cfg(file_path):
    # Search for parameters in .cfg file
    parameters = []
    # Load line by line
    lines = tuple(open(file_path, 'r'))
    for line in lines:
        if re.search(r"\.add\(", line):
            parameters.append(re.split(r".*\.add\(", line)[1])

    # Extract key and description from each parameter
    key_description = {}
    for parameter in parameters:
        arguments = re.split(r",\s", parameter)
        # Key at argument position 0, remove quotation around in "key"
        key = arguments[0][1:-1]
        # Description at argument position 3, remove quotation around in "description"
        description = arguments[3][1:-1]
        key_description[key] = description
    return key_description


def save(destination, result):
    print(f"\nSaving to '{destination}'...")

    """# Ask user if he wants to save it
    if args.no_confirm or input("\n {} parameters changed. Do you want to save? (y/n)".format(changed_params)) == "y":
        # Save new file
        with open(config_path, "w") as fp:
            yaml.dump(data, fp)
        print("Saved file")
    else:
        print("Nothing has been saved!")"""

    # TODO: save

    print("Finished saving.")


if __name__ == "__main__":
    main_description = "Combine the parameter description (documentation) of .yaml and dynamic reconfigurable .cfg files. " +\
        "This script compares the parameter descriptions of the SOURCE file with the related ones of the DESTINATION file. " +\
        "If SAVE, the DESTINATION file will be overwritten."

    parser = argparse.ArgumentParser(
        description=main_description)
    parser.add_argument("-S", "--source", action="store", nargs=1, type=str,
                        help="Source file of the parameter description")
    parser.add_argument("-D", "--destination", action="store", nargs=1, type=str,
                        help="Destination file of the parameter description")
    args = parser.parse_args()

    # Error handling
    if args.source is None or args.destination is None:
        raise argparse.ArgumentError(None, "Please define a SOURCE and a DESTINATION file.")

    source_path = os.path.realpath(args.source[0])
    destination_path = os.path.realpath(args.destination[0])

    if os.path.samefile(source_path, destination_path):
        raise argparse.ArgumentError(None, "SOURCE and DESTINATION can NOT be the same file.")

    if not ((source_path.endswith(".yaml") or source_path.endswith(".cfg")) and
            (destination_path.endswith(".yaml") or destination_path.endswith(".cfg"))):
        raise argparse.ArgumentError(None, f"SOURCE and DESTINATION file must be '.yaml' OR '.cfg' files.")

    main(source_path, destination_path)


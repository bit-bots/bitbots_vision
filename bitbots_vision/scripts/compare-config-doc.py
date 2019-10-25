#! /usr/bin/env python3
import os
import re
import sys
import argparse

from dynamic_reconfigure.parameter_generator_catkin import *

"""
A script to compare the parameter description (documentation) of .yaml and dynamic reconfigurable .cfg files.
This script saves the parameter descriptions of the SOURCE file to the related ones of the DESTINATION file, if wanted.
"""


def main(source_path, destination_path):
    # type: (str, str) -> None
    """
    Handle the main tasks of combining the descriptions of the SOURCE and DESTINATION files.

    :param str source_path: Path of the SOURCE file
    :param str destination_path: Path of the DESTINATION file
    :raturn: None
    """
    # Blacklist for parameters to exclude further
    exclude = []

    # Parse SOURCE file for keys and associated descriptions
    print(f"Analysing SOURCE file '{source_path}'...")
    source_params, source_lines = parse_file(source_path)
    print(f"Found {len(source_params)} parameters in SOURCE file.\n")

    # Warn, if parameter description contains "TODO"
    description_destination_todo = filter_todo(source_params)
    if description_destination_todo:
        print_warn(f"The following {len(description_destination_todo)} parameter descriptions of the SOURCE file contain 'TODO' and will be ignored:")
        print(description_destination_todo)
        exclude.extend(description_destination_todo)

    # Warn, if parameter has no description
    no_description_source = filter_no_description(source_params)
    if no_description_source:
        print_warn(f"\nThe following {len(no_description_source)} parameters have no description in SOURCE file:")
        print(no_description_source)
        if args.yes or yes_or_no_input("Do you want to ignore them? (If no, existing descriptions of the DESTINATION file my be deleted.)"):
            print("These parameters will be ignored...")
            exclude.extend(no_description_source)

    # Parse DESTINATION file for keys and associated descriptions
    print(f"\nAnalysing DESTINATION file'{destination_path}'...")
    destination_params, destination_lines = parse_file(destination_path)
    print(f"Found {len(destination_params)} parameters in DESTINATION file'.")

    # Show information about the number of parameters in destination file without description
    no_description_destination = filter_no_description(destination_params)
    if no_description_destination is not None:
        print(f"{len(no_description_destination)} parameters have no description in DESTINATION file.\n")

    result_params = compare(source_params, destination_params, exclude)

    if args.save:
        save(result_params, destination_lines, destination_path)

def parse_file(file_path):
    # type: (str) -> dict
    """
    Returns dict of parameter keys and descriptions read from file, and lines of the file.
    
    :param str file_path: Path of file
    :return (dict, [str]): Tuple of dict of parameter keys and descriptions read from the file, and lines of the file
    """
    lines = tuple(open(file_path, 'r'))
    # Decide which parser to use by file type
    if file_path.endswith(".yaml"):
        result = parse_yaml(lines)
    elif file_path.endswith(".cfg"):
        result = parse_cfg(lines)
    else:
        raise argparse.ArgumentError(None, f"File type must be '.yaml' OR '.cfg'. The following file has a unknown file type: '{file_path}'")
    return result, lines

def parse_yaml(lines):
    # type: ([str]) -> dict
    """
    Returns dict of parameter keys and descriptions parsed from lines of a .yaml file.
    
    :param str file_path: Lines of a .yaml file
    :return dict: parameter keys and descriptions of a .yaml file
    """
    # Extract key and description from each parameter
    key_description = {}
    for line in lines:
        if re.search(r":", line):
            # Valid parameter line, therefore insert new description
            # Split parameters into arguments by the '#' char
            arguments = re.split(r"#", line)
            # Key at argument position 0, remove value and whitespace
            key = re.split(r":", arguments[0])[0].strip()
            # Description at argument position 1, if existent
            description = ""
            if len(arguments) > 1:
                description = arguments[1].strip()
            key_description[key] = description
    return key_description

def parse_cfg(lines):
    # type: ([str]) -> dict
    """
    Returns dict of parameter keys and descriptions parsed from lines of a .cfg file.
    IMPORTANT: This does not correctly parse a parameter, that is spread between multiple lines.
    
    :param str file_path: Lines of a .cfg file
    :return dict: parameter keys and descriptions of a .cfg file
    """
    # Search for parameters and enums in .cfg file
    enums = []
    parameters = []

    dummy_list = [None] # TODO: remove
    for line in lines:
        if re.search(r"\.enum\(", line):
            enums.append(re.split(r"\=", line)[0].strip())
        if re.search(r"\.add\(", line):
            parameters.append(re.split(r".*\.add\(", line)[1])
    # Define dummy enums
    for enum in enums:
        exec(enum + " = None")
    # Extract key and description from each parameter
    key_description = {}
    for parameter in parameters:
        key, description = eval('_interpret_cfg_helper(' + parameter.rstrip())
        key_description[key] = description
    return key_description

def _interpret_cfg_helper(*args, **kwargs):
    return args[0], args[3]

def compare(source, destination, exclude=[]):
    # type: (dict, dict, list) -> dict
    """
    Returns the dict of parameters resulting of comparing the SOURCE and DESTINATION file parameters excluding the specified keys.
    
    :param dict source: Params of the SOURCE file
    :param dict destination: Params of the DESTINATION file
    :param list exclude: Keys to exclude, defaults to {}
    :return dict: Params resulting of comparing the SOURCE and DESTINATION parameters excluding the specified keys
    """
    print(f"Comparing descriptions of SOURCE and DESTINATION...")

    # Warn, if parameters occur only in one of the two files
    singleton_a, singleton_b = filter_singleton(source, destination)
    if singleton_a:
        print_warn(f"The following {len(singleton_a)} parameters only occur in the SOURCE file. They will be ignored...")
        print(singleton_a)
        exclude.extend(singleton_a)
    if singleton_b:
        print_warn(f"The following {len(singleton_b)} parameters only occur in the DESTINATION file. They will be ignored...")
        print(singleton_b)
        exclude.extend(singleton_b)

    excl_source = filter_exclude(source, exclude)
    excl_dest = filter_exclude(destination, exclude)

    # Warn, if the parameter order of SOURCE and DESTINATION differs
    if excl_source.keys() != excl_dest.keys():
        print_warn("\nThe order of the parameters of the SOURCE and DESTINATION file is not similar.")

    # Warn, for conflicting descriptions if some parameters of the destination file allready have a description
    description_excl_dest = filter_description(excl_dest)
    conflicting_keys = [key for key in description_excl_dest if excl_source[key] != excl_dest[key]]
    if conflicting_keys:
        print_warn(f"\nThe following {len(conflicting_keys)} parameters have conflicting descriptions.")
        print(conflicting_keys)
        if args.yes or yes_or_no_input("Do you want to ignore them? (If no, existing descriptions of the DESTINATION file will be overwritten.)"):
            print("These parameters will be ignored...")
            exclude.extend(conflicting_keys)

    print("Finished comparing.")
    return filter_exclude(source, exclude)

def save(params, destination_lines, destination_path):
    # type: (dict, [str], str) -> None
    """
    Save the descriptions of parameters in the DESTINATION file.

    :param dict params: Parameters to save
    :param [str] destination_lines: Lines of the DESTINATION file
    :param str destination_path: Path of the DESTINATION file to save
    """
    print(f"\nSaving {len(params)} descriptions to '{destination_path}'...")
    
    # Decide which file type to save
    if destination_path.endswith(".yaml"):
        save_yaml(params, destination_lines, destination_path)
    elif destination_path.endswith(".cfg"):
        save_cfg(params, destination_lines, destination_path)
    else:
        raise argparse.ArgumentError(None, f"File type must be '.yaml' OR '.cfg'. The following file has a unknown file type: '{destination_path}'")

    print("Finished saving.")

def save_yaml(params, lines, path):
    # type: (dict, [str], str) -> None
    """
    Save the descriptions of parameters in the .yaml DESTINATION file.

    :param dict params: Parameters to save
    :param [str] lines: Lines of the DESTINATION file
    :param str destination_path: Path of the .yaml DESTINATION file to save
    """
    output_lines = []
    for line in lines:
        if re.search(r":", line):
            # Valid parameter line, therefore insert new description, if not ignored
            # Split parameters into arguments by the '#' char
            arguments = re.split(r"\s*#\s*", line)
            # Key at argument position 0, remove value and whitespace
            key = re.split(r":", arguments[0])[0].strip()
            if key in params.keys():
                # Param has not been ignored
                new_line = arguments[0].rstrip() + "  # " + params[key] + "\n"  # Concatenate line of original key and value (with indentation) with new description
                output_lines.append(new_line)
            else:
                # Param has been ignored...
                output_lines.append(line)  # Insert line as is
        else:
            # Nonvalid parameter line
            output_lines.append(line)  # Insert line as is

    # Save output lines
    with open(path,"w") as f:
        f.writelines(output_lines)

def save_cfg(params, lines, path):
    # type: (dict, [str], str) -> None
    """
    Save the descriptions of parameters in the .cfg DESTINATION file.

    :param dict params: Parameters to save
    :param [str] lines: Lines of the DESTINATION file
    :param str destination_path: Path of the .cfg DESTINATION file to save
    """
    output_lines = []
    for line in lines:
        if re.search(r"\.add\(", line):
            # Valid parameter line, therefore insert new description, if not ignored
            parameter = re.split(r".*\.add\(", line)[1]
            arguments = re.split(r",\s", parameter)
            # Key at argument position 0, remove quotation around in "key"
            key = arguments[0][1:-1]
            if key in params.keys():
                # Param has not been ignored
                split_line = re.split(r"\,", line)  # TODO: fix , in description
                begin_line = split_line[0] + ',' + split_line[1] + ',' + split_line[2]
                end_line = ""
                for i in range(4, len(split_line)):
                    end_line = end_line + split_line[i] + ','
                new_line = begin_line + ', "' + params[key] + '",' + end_line[:-1]
                output_lines.append(new_line)
            else:
                # Param has been ignored...
                output_lines.append(line)  # Insert line as is
        else:
            # Nonvalid parameter line
            output_lines.append(line)  # Insert line as is
    print(output_lines)

    # Save output lines
    with open(path,"w") as f:
        f.writelines(output_lines)

def filter_todo(params):
    # type: (dict) -> list
    """
    Returns keys of parameters which description contains the string "TODO" (Not case sensitive).

    :param dict params: Params to check
    :return list: Keys of params which descriptions contain "TODO" (Not case sensitive)
    """
    return [key for key in params if "todo" in params[key].lower()]

def filter_description(params):
    # type: (dict) -> list
    """
    Returns keys of parameters that have a description.
    
    :param dict params: Params to check for a description
    :return list: Keys of params that have a description
    """
    return [key for key in params if params[key] != ""]

def filter_no_description(params):
    # type: (dict) -> list
    """
    Returns keys of parameters that have no description.
    
    :param dict params: Params to check for no description
    :return list: Keys of params that have no description
    """
    return [key for key in params if params[key] == ""]

def filter_singleton(a, b):
    # type: (dict, dict) -> (list, list)
    """
    Returns parameters that only occur in one of the two dicts.
    
    :param dict a: First dict
    :param dict b: Second dict
    :return (list, list): Returns a tuple of list. The first list contains keys of params, that only occur in the first dict. The second list accordingly.
    """
    singleton_a = [key for key in a if key not in b.keys()]
    singleton_b = [key for key in b if key not in a.keys()]
    return (singleton_a, singleton_b)

def filter_exclude(raw, exclude_keys):
    # type: (dict, list) -> dict
    """
    Returns the dict excluding the specified params.
    
    :param dict raw: Input dict
    :param list exclude_keys: List of keys to exclude
    :return dict: Dict excluding specified params.
    """
    return {key: raw[key] for key in raw if key not in exclude_keys}

def print_warn(message):
    # type: (str) -> None
    """
    Prints a simple yellow warning.

    :param str message: Warning message
    :return: None
    """
    print('\x1b[1;33m' + message + '\x1b[0m')

def yes_or_no_input(question):
    # type: (str) -> bool
    """
    Prints a white yes or no question and returns the answer.
    
    :param str question: Yes or no question
    :return bool: Input answer
    """
    answer = None
    while answer is None:
        reply = str(input('\x1b[1;37m' + question + ' [y|n]: ' + '\x1b[0m')).lower().strip()
        if reply[:1] == 'y':
            answer = True
        if reply[:1] == 'n':
            answer =  False
    return answer


if __name__ == "__main__":
    main_description = "Compare the parameter description (documentation) of .yaml and dynamic reconfigurable .cfg files. " +\
        "This script saves the parameter descriptions of the SOURCE file to the related ones of the DESTINATION file, if wanted."

    parser = argparse.ArgumentParser(
        description=main_description)
    parser.add_argument("-s", "--save", const=True, default=False, type=bool, nargs="?",
                        help="Saves the modified destination file.")
    parser.add_argument("-y", "--yes", const=True, default=False, type=bool, nargs="?",
                        help="Confirm everything with yes.")
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

    main(source_path, destination_path)


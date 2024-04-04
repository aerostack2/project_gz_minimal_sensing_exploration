#!/bin/python3

"""
choose_sim_config.py
"""

import argparse
import sys
from pathlib import Path


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config_folder", nargs='?', default="sim_config",
                        help="Folder containing simulation config files")
    args = parser.parse_args()

    sim_folder = Path(args.config_folder)

    files = list(sim_folder.glob("*.json"))
    print("Choose simulation config file to open:")
    i = 1
    for file in files:
        print(f"\t[{i}] {file.name}")
        i += 1

    try:
        s = input('')
        opt = int(s)
        chosen = files[opt-1]
    except (EOFError, ValueError, IndexError) as e:
        print("Invalid")
        sys.exit(1)

    # Return for use in bash scripts
    print(chosen)

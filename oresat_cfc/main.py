"""Main for the OreSat CFC daemon."""

from time import sleep


def main():
    """The main for the oresat CFC daemon"""

    try:
        # TODO replace
        while True:
            sleep(1)
    except KeyboardInterrupt:
        return 0

    return 1

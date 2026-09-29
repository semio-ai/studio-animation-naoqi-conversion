#!/usr/bin/env python
"""Convert Studio effective-motion JSON to NAOqi Bézier arguments."""

import argparse
import io
import json
import sys

from semio_naoqi_motion import prepare_motion


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="Studio .effective-motion.json file")
    parser.add_argument("-o", "--output", help="Output JSON file (default: stdout)")
    parser.add_argument(
        "--lead-in-seconds", type=float, default=0.2,
        help="time of the earliest key after a NAOqi call (default: 0.2)",
    )
    args = parser.parse_args(argv)

    try:
        converted = prepare_motion(args.input, args.lead_in_seconds)
        rendered = json.dumps(converted, indent=2, allow_nan=False) + "\n"
        if args.output:
            with io.open(args.output, "w", encoding="utf-8") as stream:
                stream.write(rendered)
        else:
            sys.stdout.write(rendered)
    except (IOError, OSError, ValueError) as error:
        parser.exit(2, "error: {}\n".format(error))


if __name__ == "__main__":
    main()

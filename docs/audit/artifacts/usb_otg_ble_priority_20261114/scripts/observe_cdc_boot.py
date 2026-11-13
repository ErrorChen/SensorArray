#!/usr/bin/env python3
"""Read-only formal CDC0 boot/state observer, stable-serial IF0 selection."""
import sys
from observe_bulk_boot import main

if __name__ == '__main__':
    if any(value.startswith('--transport') for value in sys.argv[1:]):
        raise SystemExit('This entry point selects usb_cdc; use observe_bulk_boot.py for another transport.')
    raise SystemExit(main(['--transport', 'usb_cdc', *sys.argv[1:]]))

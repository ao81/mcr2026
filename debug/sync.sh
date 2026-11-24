#!/bin/bash
set -eu
exec python3 "$(dirname "$0")/../tools/config_editor.py" --sync

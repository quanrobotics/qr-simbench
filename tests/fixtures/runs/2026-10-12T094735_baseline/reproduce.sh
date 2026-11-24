#!/usr/bin/env bash
# Re-run this experiment from its frozen source scenario.
set -euo pipefail
cd "/path/to/qr-simbench"
git checkout "$(jq -r .git_commit "/path/to/qr-simbench/runs/2026-10-12T094735_baseline/manifest.json")" 2>/dev/null || true
qrsimbench run "/path/to/qr-simbench/runs/2026-10-12T094735_baseline/scenario.source.yaml"

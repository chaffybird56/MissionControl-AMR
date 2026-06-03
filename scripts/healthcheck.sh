#!/usr/bin/env bash
# Quick sanity check that key Python entrypoints import.
python3 -m py_compile server/app.py
echo "[ok] MissionControl server imports compile"

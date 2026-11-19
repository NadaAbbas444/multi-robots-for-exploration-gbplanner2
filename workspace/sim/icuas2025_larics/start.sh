#!/bin/bash
# Start the ICUAS 2025 two-UAV session (run inside the icuas2025-larics container).
# Attach with:  tmux attach -t icuas2025      Stop with:  tmux kill-server
cd "$(dirname "$(readlink -f "$0")")"
source "$(dirname "$(readlink -f "$0")")/env.sh"
rm -f .tmuxinator.yml
ln -s session.yml .tmuxinator.yml
tmuxinator start icuas2025 -p session.yml --no-attach 2>/dev/null || tmuxinator start -p session.yml

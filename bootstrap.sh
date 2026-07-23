#!/usr/bin/env bash
set -euo pipefail

# One-shot environment setup for a fresh cloud instance (e.g. Nectar).
# Safe to re-run - every step is idempotent.
#
# What this does:
#   1. Installs apt dependencies (compiler toolchain, Boost, LEMON, git, gh)
#   2. Authenticates git via GitHub CLI device login (once per instance -
#      credentials are managed by `gh`, never written into this repo)
#   3. Builds the project (./compile.sh)
#
# Run from the repo root: ./bootstrap.sh

if [ "$(id -u)" -eq 0 ]; then
    SUDO=""
else
    SUDO="sudo"
fi

echo "==> Installing system packages"
$SUDO apt-get update
# liblemon-dev isn't listed in apt.txt (that file only covers the optional
# python/torch track) but is required by CMakeLists.txt's find_package(LEMON).
$SUDO apt-get install -y --no-install-recommends \
    $(cat apt.txt) \
    liblemon-dev \
    git \
    curl

echo "==> Installing GitHub CLI (gh)"
if ! command -v gh &> /dev/null; then
    curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | $SUDO dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg
    $SUDO chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
        | $SUDO tee /etc/apt/sources.list.d/github-cli.list > /dev/null
    $SUDO apt-get update
    $SUDO apt-get install -y gh
fi

echo "==> Git auth via GitHub CLI"
if ! gh auth status &> /dev/null; then
    echo "Not logged in - follow the prompts (device code / browser login)."
    gh auth login
else
    echo "Already logged in as $(gh api user --jq .login 2>/dev/null || echo '<unknown>')."
fi

# Point git's own credential helper at gh's stored auth, so plain
# `git clone/pull/push` over HTTPS stop prompting for credentials.
gh auth setup-git

echo "==> Building project"
chmod +x compile.sh
./compile.sh

echo "==> Done. Binaries at ./build/lifelong and ./build/map_reduction_test"

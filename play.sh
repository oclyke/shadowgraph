#!/usr/bin/env bash
#
# play.sh — draw SVG/ILDA art on the shadowgraph projector.
#
# Coordinates the host tools: classify each input (scenekind), convert SVGs to
# ILDA frames (svg2scene), then stream the resulting frame(s) to the device at a
# frame rate (ildaplay). Inputs may be any mix of .svg and .ild; ILDA files are
# used as-is, SVGs are converted with the options below.
#
# Usage:
#   ./play.sh [options] <input.svg|input.ild> [more inputs...]
#
# Options (each also has an env-var default, shown in []):
#   --host IP[:PORT]   device address                 [$SHADOWGRAPH_HOST]
#   --fps N            animation frame rate           [$SHADOWGRAPH_FPS, def 12]
#   --once             play the playlist once, no loop
#   --points N         svg2scene target points/frame  [$SHADOWGRAPH_POINTS, def 600]
#   --amplitude N      svg2scene field amplitude      [$SHADOWGRAPH_AMPLITUDE, def 16000]
#   --intensity F      svg2scene brightness 0..1      [$SHADOWGRAPH_INTENSITY, def 1.0]
#   --out DIR          where to write generated .ild  [temp dir]
#   -h, --help         this help
#
# With no --host (and no $SHADOWGRAPH_HOST) the inputs are converted and the
# playlist is printed, but nothing is streamed — handy for just building frames.
set -euo pipefail

HOST="${SHADOWGRAPH_HOST:-}"
FPS="${SHADOWGRAPH_FPS:-30}"
POINTS="${SHADOWGRAPH_POINTS:-600}"
AMPLITUDE="${SHADOWGRAPH_AMPLITUDE:-16000}"
INTENSITY="${SHADOWGRAPH_INTENSITY:-1.0}"
OUTDIR="${SHADOWGRAPH_OUTDIR:-}"
ONCE=0

usage() { sed -n '2,/^set -euo/{/^set -euo/d;s/^# \{0,1\}//;p;}' "$0"; }

inputs=()
while [ $# -gt 0 ]; do
  case "$1" in
    --host)      HOST="$2"; shift 2 ;;
    --fps)       FPS="$2"; shift 2 ;;
    --points)    POINTS="$2"; shift 2 ;;
    --amplitude) AMPLITUDE="$2"; shift 2 ;;
    --intensity) INTENSITY="$2"; shift 2 ;;
    --out)       OUTDIR="$2"; shift 2 ;;
    --once)      ONCE=1; shift ;;
    -h|--help)   usage; exit 0 ;;
    --)          shift; while [ $# -gt 0 ]; do inputs+=("$1"); shift; done ;;
    -*)          echo "play.sh: unknown option: $1" >&2; exit 2 ;;
    *)           inputs+=("$1"); shift ;;
  esac
done

if [ ${#inputs[@]} -eq 0 ]; then
  echo "play.sh: no input files given" >&2
  usage >&2
  exit 2
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
svg2scene() { cargo run -q --manifest-path "$ROOT/tools/svg2scene/Cargo.toml" -- "$@"; }
ildaplay()  { cargo run -q --manifest-path "$ROOT/tools/ildaplay/Cargo.toml"  -- "$@"; }
scenekind() { cargo run -q --manifest-path "$ROOT/tools/scenekind/Cargo.toml" -- "$@"; }

if [ -z "$OUTDIR" ]; then
  OUTDIR="$(mktemp -d "${TMPDIR:-/tmp}/shadowgraph.XXXXXX")"
fi
mkdir -p "$OUTDIR"

# Classify each input; convert SVGs, pass ILDA through. Build the playlist.
playlist=()
for f in "${inputs[@]}"; do
  if [ ! -f "$f" ]; then
    echo "play.sh: no such file: $f" >&2
    exit 1
  fi
  if ! kind="$(scenekind "$f")"; then
    echo "play.sh: $f failed the sanity check" >&2
    exit 1
  fi
  case "$kind" in
    ild)
      playlist+=("$f")
      ;;
    svg)
      out="$OUTDIR/$(basename "${f%.*}").ild"
      svg2scene "$f" -o "$out" \
        --points "$POINTS" --amplitude "$AMPLITUDE" --intensity "$INTENSITY"
      playlist+=("$out")
      ;;
    *)
      echo "play.sh: unexpected kind '$kind' for $f" >&2
      exit 1
      ;;
  esac
done

echo "playlist (${#playlist[@]} frame(s)):" >&2
for p in "${playlist[@]}"; do echo "  $p" >&2; done

if [ -z "$HOST" ]; then
  echo "play.sh: no --host / \$SHADOWGRAPH_HOST — converted only, not streaming." >&2
  exit 0
fi

play_args=(--host "$HOST" --fps "$FPS")
if [ "$ONCE" -eq 1 ]; then
  play_args+=(--once)
fi
echo "play.sh: streaming to $HOST at ${FPS} fps" >&2
ildaplay "${play_args[@]}" "${playlist[@]}"

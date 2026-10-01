#!/bin/sh
# Switch the HRTF used by the Maxwell 3D sink and restart PipeWire.
#
#   tools/hrtf-switch.sh          list the installed SOFA files, current one marked
#   tools/hrtf-switch.sh H5       use H5_48K_24bit_256tap_FIR_SOFA.sofa
#
# Files live in ~/.local/share/pipewire/hrtf-sofa/ (see README). Known SADIE II
# subjects get a gain that matches D1's loudness (200 Hz - 8 kHz at the seven
# speaker positions), so comparisons aren't biased towards the louder one.
# A running maxwell3d is stopped first: it doesn't survive a PipeWire restart.
set -eu

dir="$HOME/.local/share/pipewire/hrtf-sofa"
conf="$(dirname "$(readlink -f "$0")")/../pipewire/maxwell-3d-sink.conf"
current=$(sed -nE 's|.*filename = "([^"]*\.sofa)".*|\1|p' "$conf" | head -n 1)

gain_for() {
    case "$1" in
        D1_*) echo 0.125 ;;
        H3_*) echo 0.124 ;;
        H4_*) echo 0.110 ;;
        H5_*) echo 0.126 ;;
        H6_*) echo 0.132 ;;
        H7_*) echo 0.116 ;;
        *) echo "warning: no loudness-matched gain for $1, using 0.125" >&2; echo 0.125 ;;
    esac
}

if [ $# -eq 0 ]; then
    for f in "$dir"/*.sofa; do
        [ "$f" = "$current" ] && mark="*" || mark=" "
        echo "$mark $(basename "$f" .sofa | cut -d_ -f1)    $(basename "$f")"
    done
    exit 0
fi

file=$(ls "$dir/$1"_*.sofa "$dir/$1" 2>/dev/null | head -n 1 || true)
[ -n "$file" ] || { echo "no SOFA file matching '$1' in $dir" >&2; exit 1; }
gain=$(gain_for "$(basename "$file")")

sed -i -E "s|(filename = \")[^\"]*\.sofa(\" gain = )[0-9.]+|\1$file\2$gain|" "$conf"
echo "HRTF: $(basename "$file") (gain $gain)"

if pkill -TERM -x maxwell3d; then
    echo "stopped maxwell3d, start it again after the restart"
    sleep 1
fi
systemctl --user restart pipewire

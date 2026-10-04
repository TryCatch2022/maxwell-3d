#!/bin/sh
# Install maxwell3d for the current user, or remove it again.
#
#   ./install.sh               build and install (again: updates, keeps the HRTF choice)
#   ./install.sh --uninstall   remove everything but the downloaded HRTFs
#
# Installs:
#   ~/.local/bin/maxwell3d, ~/.local/bin/maxwell3d-hrtf (tools/hrtf-switch.sh)
#   ~/.config/pipewire/pipewire.conf.d/maxwell-3d-sink.conf
#   ~/.config/systemd/user/maxwell3d.service (enabled: starts at login)
#   /etc/udev/rules.d/70-audeze-maxwell.rules (through sudo, only if it differs)
# and downloads the SADIE II D1 HRTF into ~/.local/share/pipewire/hrtf-sofa/
# if it isn't there. PipeWire is restarted when the sink config changes.
set -eu
cd "$(dirname "$(readlink -f "$0")")"

bin_dir="$HOME/.local/bin"
hrtf_dir="${XDG_DATA_HOME:-$HOME/.local/share}/pipewire/hrtf-sofa"
conf="${XDG_CONFIG_HOME:-$HOME/.config}/pipewire/pipewire.conf.d/maxwell-3d-sink.conf"
unit="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user/maxwell3d.service"
udev_rule=/etc/udev/rules.d/70-audeze-maxwell.rules
hrtf=D1_48K_24bit_256tap_FIR_SOFA.sofa
hrtf_url='https://zenodo.org/records/12092466/files/D1_HRIR_SOFA.zip?download=1'

step() { printf '\n==> %s\n' "$*"; }
die() { echo "error: $*" >&2; exit 1; }

if [ "${1:-}" = "--uninstall" ]; then
    step "Removing maxwell3d"
    systemctl --user disable --now maxwell3d.service 2>/dev/null || true
    rm -f "$unit" "$bin_dir/maxwell3d" "$bin_dir/maxwell3d-hrtf" "$conf"
    systemctl --user daemon-reload
    systemctl --user restart pipewire
    if [ -e "$udev_rule" ]; then
        sudo rm -f "$udev_rule"
        sudo udevadm control --reload
    fi
    echo "Done. The HRTF files in $hrtf_dir were left in place."
    exit 0
fi
[ $# -eq 0 ] || die "usage: $0 [--uninstall]"

for tool in cargo curl unzip pkg-config; do
    command -v "$tool" >/dev/null || die "$tool is needed"
done
pkg-config --exists libpipewire-0.3 || die "the PipeWire development files are needed (libpipewire-0.3-dev / pipewire-devel)"

step "Building"
cargo build --release --locked
install -Dm755 target/release/maxwell3d "$bin_dir/maxwell3d"
install -Dm755 tools/hrtf-switch.sh "$bin_dir/maxwell3d-hrtf"
echo "installed $bin_dir/maxwell3d and $bin_dir/maxwell3d-hrtf"

step "HRTF"
if [ -f "$hrtf_dir/$hrtf" ]; then
    echo "$hrtf_dir/$hrtf is there"
else
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    curl -fL -o "$tmp/hrtf.zip" "$hrtf_url"
    mkdir -p "$hrtf_dir"
    unzip -j "$tmp/hrtf.zip" "D1_HRIR_SOFA/$hrtf" -d "$hrtf_dir"
fi

step "PipeWire sink"
new_conf=$(mktemp)
sed "s|@HRTF_DIR@|$hrtf_dir|g" pipewire/maxwell-3d-sink.conf > "$new_conf"
# Keep the HRTF picked with maxwell3d-hrtf.
previous=$(sed -nE 's|.*filename = "([^"]*\.sofa)" gain = ([0-9.]+).*|\1 \2|p' "$conf" 2>/dev/null | head -n 1 || true)
if [ -n "$previous" ] && [ -f "${previous% *}" ]; then
    sed -i -E "s|(filename = \")[^\"]*\.sofa(\" gain = )[0-9.]+|\1${previous% *}\2${previous##* }|" "$new_conf"
fi
if [ -f "$conf" ] && [ ! -L "$conf" ] && cmp -s "$new_conf" "$conf"; then
    echo "$conf is up to date"
    rm "$new_conf"
    restart_pipewire=false
else
    mkdir -p "$(dirname "$conf")"
    rm -f "$conf"  # may be a symlink into a checkout
    mv "$new_conf" "$conf"
    chmod 644 "$conf"
    echo "installed $conf"
    restart_pipewire=true
fi
[ -f "$hrtf_dir/room/room_FL.wav" ] ||
    echo "note: no room sound yet, see \"Room sound\" in README.md to generate it (optional)"

step "udev rule (lets you read the headset's HID interface)"
if cmp -s udev/70-audeze-maxwell.rules "$udev_rule"; then
    echo "$udev_rule is up to date"
else
    echo "installing $udev_rule, sudo asks for your password"
    sudo install -m644 udev/70-audeze-maxwell.rules "$udev_rule"
    sudo udevadm control --reload
    sudo udevadm trigger --subsystem-match=hidraw --action=change
fi

step "systemd user service"
install -Dm644 systemd/maxwell3d.service "$unit"
systemctl --user daemon-reload
if $restart_pipewire; then
    echo "restarting PipeWire to load the sink (audio drops for a moment)"
    systemctl --user restart pipewire
fi
systemctl --user enable maxwell3d.service
systemctl --user restart maxwell3d.service
sleep 1
systemctl --user --no-pager status maxwell3d.service | head -n 5

case ":$PATH:" in
    *":$bin_dir:"*) ;;
    *) echo "note: $bin_dir is not in PATH, maxwell3d-hrtf won't be found by name" ;;
esac
echo
echo "Done. Re-centre: systemctl --user kill -s USR1 maxwell3d"
echo "      Toggle:    systemctl --user kill -s USR2 maxwell3d"

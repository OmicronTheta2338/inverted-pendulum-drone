#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/.." && pwd)"
output_dir="${1:-$project_dir/recordings/ff-comparison}"
mkdir -p -- "$output_dir"

godot_bin="${GODOT_BIN:-}"
if [[ -z "$godot_bin" ]]; then
	if command -v godot >/dev/null 2>&1; then
		godot_bin="$(command -v godot)"
	elif [[ -x /home/omicrontheta/Godot/Godot_v4.7.1-stable_linux.x86_64 ]]; then
		godot_bin="/home/omicrontheta/Godot/Godot_v4.7.1-stable_linux.x86_64"
	else
		echo "Set GODOT_BIN to a Godot 4 executable." >&2
		exit 1
	fi
fi

if ! command -v ffmpeg >/dev/null 2>&1; then
	echo "ffmpeg is required to assemble the side-by-side comparison." >&2
	exit 1
fi

fps="${COMPARISON_FPS:-60}"
arm_count=2
trim_start="${COMPARISON_TRIM_START:-19.5}"
comparison_duration="${COMPARISON_DURATION:-6.5}"
ff_on_video="$output_dir/reaction_ff_on.avi"
ff_off_video="$output_dir/reaction_ff_off.avi"
comparison_video="$output_dir/reaction_ff_comparison.mp4"

render_clip() {
	local post_release_ff="$1"
	local destination="$2"
	echo "Rendering POSTREL_FF=$post_release_ff -> $destination"
	env \
		USE_NATIVE=1 \
		NUM_ARMS="$arm_count" \
		BALANCE_ONBOARD=1 \
		REACTION_FF=1 \
		POSTREL_FF="$post_release_ff" \
		RESPONSE_VIDEO=1 \
		RECORDING_AUTO_EXIT=1 \
		SENSOR_SEED=1 \
		"$godot_bin" \
		--path "$project_dir" \
		--write-movie "$destination" \
		--fixed-fps "$fps" \
		--disable-vsync \
		--audio-driver Dummy
}

# Both simulations are deterministic and use reaction FF during balance. Only
# POSTREL_FF changes, so their states match up to the arm-release event.
render_clip 1 "$ff_on_video"
render_clip 0 "$ff_off_video"

echo "Compositing synchronized post-release comparison -> $comparison_video"
ffmpeg -y \
	-ss "$trim_start" -i "$ff_on_video" \
	-ss "$trim_start" -i "$ff_off_video" \
	-filter_complex \
	"[0:v]scale=1280:720:flags=lanczos,drawbox=x=0:y=0:w=iw:h=52:color=black@0.65:t=fill,drawtext=text='Reaction feed-forward ON':x=(w-text_w)/2:y=12:fontsize=28:fontcolor=white[on];[1:v]scale=1280:720:flags=lanczos,drawbox=x=0:y=0:w=iw:h=52:color=black@0.65:t=fill,drawtext=text='Reaction feed-forward OFF':x=(w-text_w)/2:y=12:fontsize=28:fontcolor=white[off];[on][off]hstack=inputs=2[v]" \
	-map "[v]" \
	-t "$comparison_duration" \
	-r "$fps" \
	-an \
	-c:v libx264 \
	-crf 18 \
	-preset medium \
	-pix_fmt yuv420p \
	-movflags +faststart \
	"$comparison_video"

echo "Done: $comparison_video"

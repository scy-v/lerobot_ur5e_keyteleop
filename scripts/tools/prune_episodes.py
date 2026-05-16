from pathlib import Path
import yaml
from typing import List
import argparse
import logging
import lerobot.datasets.dataset_tools as dataset_tools
from lerobot.scripts.lerobot_edit_dataset import handle_delete_episodes, EditDatasetConfig, DeleteEpisodesConfig
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def _keep_episodes_from_video_with_av_modified(
    input_path: Path,
    output_path: Path,
    episodes_to_keep: list[tuple[float, float]],
    fps: float,
    vcodec: str = "libsvtav1",
    pix_fmt: str = "yuv420p",
) -> None:
    """Keep only specified episodes from a video file using PyAV.

    This function decodes frames from specified time ranges and re-encodes them with
    properly reset timestamps to ensure monotonic progression.

    Args:
        input_path: Source video file path.
        output_path: Destination video file path.
        episodes_to_keep: List of (start_time, end_time) tuples for episodes to keep.
        fps: Frame rate of the video.
        vcodec: Video codec to use for encoding.
        pix_fmt: Pixel format for output video.
    """
    from fractions import Fraction

    import av

    if not episodes_to_keep:
        raise ValueError("No episodes to keep")

    in_container = av.open(str(input_path))

    # Check if video stream exists.
    if not in_container.streams.video:
        raise ValueError(
            f"No video streams found in {input_path}. "
            "The video file may be corrupted or empty. "
            "Try re-downloading the dataset or checking the video file."
        )

    v_in = in_container.streams.video[0]

    out = av.open(str(output_path), mode="w")

    # Convert fps to Fraction for PyAV compatibility.
    fps_fraction = Fraction(fps).limit_denominator(1000)
    v_out = out.add_stream(vcodec, rate=fps_fraction)

    # PyAV type stubs don't distinguish video streams from audio/subtitle streams.
    v_out.width = v_in.codec_context.width
    v_out.height = v_in.codec_context.height
    v_out.pix_fmt = pix_fmt

    # Set time_base to match the frame rate for proper timestamp handling.
    v_out.time_base = Fraction(1, int(fps))

    out.start_encoding()

    # Create set of (start, end) ranges for fast lookup.
    # Convert to a sorted list for efficient checking.
    time_ranges = sorted(episodes_to_keep)

    # Track frame index for setting PTS and current range being processed.
    frame_count = 0
    range_idx = 0
    # Read through entire video once and filter frames.
    for packet in in_container.demux(v_in):
        try:
            frames = packet.decode()
        except Exception as e:
            approx_time = None
            if packet.pts is not None and v_in.time_base is not None:
                approx_time = float(packet.pts * v_in.time_base)

            logging.error(
                f"[DECODE ERROR - expected & skipped] file={input_path}, "
                f"time={approx_time}, pts={packet.pts}, "
                f"note=corrupted frames are intentionally skipped, "
                f"error={repr(e)}"
            )
            continue
        for frame in frames:
            if frame is None:
                continue

            # Get frame timestamp.
            frame_time = float(frame.pts * frame.time_base) if frame.pts is not None else 0.0

            # Check if frame is in any of our desired time ranges.
            # Skip ranges that have already passed.
            while range_idx < len(time_ranges) and frame_time >= time_ranges[range_idx][1]:
                range_idx += 1

            # If we've passed all ranges, stop processing.
            if range_idx >= len(time_ranges):
                break

            # Check if frame is in current range.
            start_ts, end_ts = time_ranges[range_idx]
            if frame_time < start_ts:
                continue

            # Frame is in range - create a new frame with reset timestamps.
            # We need to create a copy to avoid modifying the original.
            new_frame = frame.reformat(width=v_out.width, height=v_out.height, format=v_out.pix_fmt)
            new_frame.pts = frame_count
            new_frame.time_base = Fraction(1, int(fps))

            # Encode and mux the frame.
            for pkt in v_out.encode(new_frame):
                out.mux(pkt)

            frame_count += 1

    # Flush encoder.
    for pkt in v_out.encode():
        out.mux(pkt)

    out.close()
    in_container.close()

dataset_tools._keep_episodes_from_video_with_av = _keep_episodes_from_video_with_av_modified

def parse_episode_indices(indices_str: str) -> List[int]:
    """
    Convert a command-line string like "[22,23]" to a list of integers [22, 23]
    """
    indices_str = indices_str.strip()
    if indices_str.startswith("[") and indices_str.endswith("]"):
        indices_str = indices_str[1:-1]  # remove brackets
    else:
        raise ValueError("Input must be in format [22,23]")
    return [int(i.strip()) for i in indices_str.split(",") if i.strip()]

def main():
    parser = argparse.ArgumentParser(description="Prune dataset episodes")
    parser.add_argument("episode_indices", type=str, help='Episode indices to delete, e.g., "[22,23]"')
    args = parser.parse_args()

    try:
        episode_indices = parse_episode_indices(args.episode_indices)
    except Exception as e:
        print(f"Failed to parse episode_indices: {e}")
        return
    
    parent_path = Path(__file__).resolve().parent
    cfg_path = parent_path.parent / "config" / "cfg.yaml"

    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f)

    repo_id = cfg["prune_episodes"]["old_dataset_name"]
    new_repo_id = cfg["prune_episodes"]["new_dataset_name"]

    # Check if either repo_id or new_repo_id is None
    if repo_id is None or new_repo_id is None:
        print("Error: Source or target dataset name is not defined in the config. Aborting.")
        return

    print("\nDataset Pruning Confirmation")
    print("-" * 40)
    print(f"Source dataset (will be unchanged): {repo_id}")
    print(f"Target dataset (will be pruned): {new_repo_id}")
    print(f"Episodes to delete: {episode_indices}")
    print("-" * 40)

    confirm = input("Proceed with pruning? (y/n): ").strip().lower()

    if confirm != "y":
        print("Operation cancelled.")
        return
    
    cfg = EditDatasetConfig(
        repo_id=repo_id,
        new_repo_id=new_repo_id,
        operation=DeleteEpisodesConfig(
            type="delete_episodes",
            episode_indices=episode_indices
        )
    )
    handle_delete_episodes(cfg)
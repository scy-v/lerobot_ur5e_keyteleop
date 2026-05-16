import logging
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from pathlib import Path
import yaml
from tqdm import tqdm

logging.basicConfig(level=logging.INFO)


class EpisodeSampler(torch.utils.data.Sampler):
    """Iterate over frames of a single episode"""
    def __init__(self, dataset: LeRobotDataset, episode_index: int):
        from_idx = dataset.meta.episodes["dataset_from_index"][episode_index]
        to_idx = dataset.meta.episodes["dataset_to_index"][episode_index]
        self.frame_ids = range(from_idx, to_idx)

    def __iter__(self):
        return iter(self.frame_ids)

    def __len__(self):
        return len(self.frame_ids)


def check_dataset(repo_id: str):
    logging.info("Loading dataset metadata only")
    dataset = LeRobotDataset(repo_id, episodes=None)

    num_episodes = len(dataset.meta.episodes["dataset_from_index"])
    logging.info(f"Found {num_episodes} episodes in dataset {repo_id}")

    errors = []
    success_count = 0

    # outer progress bar (episodes)
    for episode_index in tqdm(range(num_episodes), desc="Episodes"):
        ep_idx = episode_index
        ep_display = episode_index + 1

        sampler = EpisodeSampler(dataset, episode_index)
        dataloader = torch.utils.data.DataLoader(
            dataset, batch_size=1, sampler=sampler
        )

        frame_idx = 0

        try:
            # inner progress bar (frames)
            for batch in tqdm(
                dataloader,
                total=len(sampler),
                desc=f"Episode {ep_display}/{num_episodes} | total frames {len(sampler)}",
                leave=False,  # do not keep per-episode bars
            ):
                frame_idx += 1

            success_count += 1

        except Exception as e:
            errors.append({
                "episode_idx": ep_idx,
                "episode_display": ep_display,
                "frame": frame_idx,
                "error": str(e),
            })

    # final summary
    print("\n" + "=" * 50)
    print("DATASET CHECK SUMMARY")
    print("=" * 50)
    print(f"Total episodes: {num_episodes}")
    print(f"Success: {success_count}")
    print(f"Failed: {len(errors)}")

    if errors:
        print("\nFailed episodes detail:")
        for err in errors:
            print(
                f"  - Episode {err['episode_display']} "
                f"(idx={err['episode_idx']}) | "
                f"Frame {err['frame']} | Error: {err['error']}"
            )


def main():
    parent_path = Path(__file__).resolve().parent
    cfg_path = parent_path.parent / "config" / "cfg.yaml"

    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f)

    repo_id = cfg["check_dataset"]["dataset_name"]
    check_dataset(repo_id)


if __name__ == "__main__":
    main()
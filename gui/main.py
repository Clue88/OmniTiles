import argparse

from app import App


def main() -> None:
    parser = argparse.ArgumentParser(description="OmniTiles GUI")
    parser.add_argument(
        "--fake",
        action="store_true",
        help="Run with in-process simulated tiles instead of scanning BLE.",
    )
    args = parser.parse_args()
    App(fake=args.fake).run()


if __name__ == "__main__":
    main()

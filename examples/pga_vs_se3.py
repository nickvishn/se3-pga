import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.viewers import pga_rigid_body_viewer, se3_rigid_body_viewer


def main():
    se3_rigid_body_viewer()
    pga_rigid_body_viewer()


if __name__ == "__main__":
    main()

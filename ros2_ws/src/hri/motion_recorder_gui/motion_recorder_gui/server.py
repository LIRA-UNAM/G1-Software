from getup_gui.server import run_server

from .bridge import MotionRecorderGuiBridge


def main(args=None):
    run_server(MotionRecorderGuiBridge, "motion_recorder_gui", args)


if __name__ == "__main__":
    main()

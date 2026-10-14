from getup_gui.server import run_server

from .bridge import PpowalkGuiBridge


def main(args=None):
    run_server(PpowalkGuiBridge, "ppowalk_gui", args)


if __name__ == "__main__":
    main()

#!/usr/bin/env python

import controlnode_sq
import threading
import signal

if __name__ == '__main__':
    node = controlnode_sq.ControlNode("controller")

    signal.signal(signal.SIGINT, node.shutdown)

    controller = threading.Thread(start=node.start)
    controller.start()

    while controller.run:
        signal.pause()

    controller.join()

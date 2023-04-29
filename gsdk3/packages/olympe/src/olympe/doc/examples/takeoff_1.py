# -*- coding: UTF-8 -*-

import olympe
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

DRONE_IP = "192.168.42.1"


def main():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    
    assert drone(TakeOff()).wait().success()
    #try:
    assert drone(
            moveBy(0.0,0.0,0.0,0.0)
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()
    time.sleep(5)
    
    assert drone(
          moveBy(-0.0,0.0,0.0,0.0)
          >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()
    
    assert drone(Landing()).wait().success()
    #except Exception:
    #  assert drone(Landing()).wait().success()
      
    drone.disconnect()


if __name__ == "__main__":
    main()

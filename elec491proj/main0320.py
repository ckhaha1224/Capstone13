# from tkinter import N
from unittest import runner
from utils.params import params
from model_processors.BaseProcessor import BaseProcessor
from model_processors.FaceDetectionProcessor import ModelProcessor as FaceDetectionProcessor
from model_processors.HandGestureProcessor import ModelProcessor as HandGestureProcessor
from atlas_utils.presenteragent import presenter_channel
from atlas_utils.acl_image import AclImage
import _thread

from atlas_utils.acl_resource import AclResource
import time
from enum import Enum
import cv2

import csv
import numpy as np
import math
import queue
import shlex
import subprocess
import tempfile
import traceback

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged

from utils.shared_variable import Shared
import threading
# from threading import Thread
from utils.runlive import PresenterServer
import openpose_tf

from atlas_utils.acl_resource import AclResource
import os
from utils.process_2 import process_2
from multiprocessing import Process, Pipe

class State(Enum):
    INITIAL = 1
    TAKEOFF_CONFIRM = 2
    TAKEOFF = 3
    FLOAT = 4
    LAND_CONFIRM = 5
    LAND = 6
    FOLLOW_ME_CONFIRM = 7
    FOLLOW_ME = 8
    STOP = 9
    TAKE_A_PICTURE_CONFIRM = 10
    TAKE_A_PICTURE = 11

def get_next_state(state, command):
    if state == State.INITIAL: 
        return State.TAKEOFF_CONFIRM if command == "1" else State.INITIAL
        # return State.INITIAL
    elif state == State.TAKEOFF_CONFIRM:
        return State.TAKEOFF if command == "2" else State.INITIAL
    elif state == State.TAKEOFF:
        return State.FLOAT
    elif state == State.FLOAT:
        if command == "1":
            return State.LAND_CONFIRM
            
        elif command == "3":
            return State.FOLLOW_ME_CONFIRM
        elif command == "4":
            return State.TAKE_A_PICTURE_CONFIRM
        else:
            return State.FLOAT
    elif state == State.LAND_CONFIRM:
        return State.LAND if command == "2" else State.LAND_CONFIRM
    elif state == State.LAND:
        return State.INITIAL
    elif state == State.FOLLOW_ME_CONFIRM:
        return State.FOLLOW_ME if command == "2" else State.FLOAT
    elif state == State.FOLLOW_ME:
        return State.STOP if command == "3" else State.FOLLOW_ME
    elif state == State.STOP:
        return State.FLOAT if command == "2" else State.FOLLOW_ME
    elif state == State.TAKE_A_PICTURE_CONFIRM:
        return State.TAKE_A_PICTURE if command == "2" else State.FLOAT
    elif state == State.TAKE_A_PICTURE:
        return State.FLOAT
    else:
        print(state)
        return state


olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

DRONE_IP = "10.202.0.1"


class StreamingExample(threading.Thread):

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(DRONE_IP)
        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
        print("Olympe streaming example output dir: {}".format(self.tempd))
        self.h264_frame_stats = []
        self.h264_stats_file = open(
            os.path.join(self.tempd, 'h264_stats.csv'), 'w+')
        self.h264_stats_writer = csv.DictWriter(
            self.h264_stats_file, ['fps', 'bitrate'])
        self.h264_stats_writer.writeheader()
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()
        super().__init__()
        super().start()

    def start(self):
        # Connect the the drone
        self.drone.connect()

        # You can record the video stream from the drone if you plan to do some
        # post processing.
        self.drone.set_streaming_output_files(
            h264_data_file=os.path.join(self.tempd, 'h264_data.264'),
            h264_meta_file=os.path.join(self.tempd, 'h264_metadata.json'),
            # Here, we don't record the (huge) raw YUV video stream
            # raw_data_file=os.path.join(self.tempd,'raw_data.bin'),
            # raw_meta_file=os.path.join(self.tempd,'raw_metadata.json'),
        )

        # Setup your callback functions to do some live video processing
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
        )
        # Start video streaming
        self.drone.start_video_streaming()

    def stop(self):
        # Properly stop the video stream and disconnect
        self.drone.stop_video_streaming()
        self.drone.disconnect()
        self.h264_stats_file.close()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.
            :type yuv_frame: olympe.VideoFrame
        """
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)

    def flush_cb(self):
        with self.flush_queue_lock:
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait().unref()
        return True

    def start_cb(self):
        pass

    def end_cb(self):
        pass

    def h264_frame_cb(self, h264_frame):
        """
        This function will be called by Olympe for each new h264 frame.
            :type yuv_frame: olympe.VideoFrame
        """

        # Get a ctypes pointer and size for this h264 frame
        frame_pointer, frame_size = h264_frame.as_ctypes_pointer()

        # For this example we will just compute some basic video stream stats
        # (bitrate and FPS) but we could choose to resend it over an another
        # interface or to decode it with our preferred hardware decoder..

        # Compute some stats and dump them in a csv file
        info = h264_frame.info()
        frame_ts = info["ntp_raw_timestamp"]
        if not bool(info["h264"]["is_sync"]):
            if len(self.h264_frame_stats) > 0:
                while True:
                    start_ts, _ = self.h264_frame_stats[0]
                    if (start_ts + 1e6) < frame_ts:
                        self.h264_frame_stats.pop(0)
                    else:
                        break
            self.h264_frame_stats.append((frame_ts, frame_size))
            h264_fps = len(self.h264_frame_stats)
            h264_bitrate = (
                8 * sum(map(lambda t: t[1], self.h264_frame_stats)))
            self.h264_stats_writer.writerow(
                {'fps': h264_fps, 'bitrate': h264_bitrate})

    def show_yuv_frame(self, window_name, yuv_frame):
        # the VideoFrame.info() dictionary contains some useful information
        # such as the video resolution
        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]

        # yuv_frame.vmeta() returns a dictionary that contains additional
        # metadata from the drone (GPS coordinates, battery percentage, ...)

        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]

        # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
        # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

        # Use OpenCV to convert the yuv frame to RGB
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
        # Use OpenCV to show this frame
        cv2.imshow(window_name, cv2frame)
        cv2.waitKey(1)  # please OpenCV for 1 ms...

    def convert_yuv_frame(self, yuv_frame):
        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]

        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]

        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
        return cv2frame


    def run(self):
        window_name = "Olympe Streaming Example"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        main_thread = next(
            filter(lambda t: t.name == "MainThread", threading.enumerate())
        )
        while main_thread.is_alive():
            with self.flush_queue_lock:
                try:
                    yuv_frame = self.frame_queue.get(timeout=0.01)
                except queue.Empty:
                    continue
                try:
                    self.show_yuv_frame(window_name, yuv_frame)
                except Exception:
                    # We have to continue popping frame from the queue even if
                    # we fail to show one frame
                    traceback.print_exc()
                finally:
                    # Don't forget to unref the yuv frame. We don't want to
                    # starve the video buffer pool
                    yuv_frame.unref()
        cv2.destroyWindow(window_name)

    def get_cv2_frame(self):
        main_thread = next(
            filter(lambda t: t.name == "MainThread", threading.enumerate())
        )
        while main_thread.is_alive():
            with self.flush_queue_lock:
                try:
                    yuv_frame = self.frame_queue.get(timeout=0.01)
                except queue.Empty:
                    continue
                try:
                    cv2_frame = self.convert_yuv_frame(yuv_frame)
                except Exception:
                    # We have to continue popping frame from the queue even if
                    # we fail to show one frame
                    traceback.print_exc()
                finally:
                    # Don't forget to unref the yuv frame. We don't want to
                    # starve the video buffer pool
                    yuv_frame.unref()
                    # Not sure if this it the right place to return without messing up threading
                    return cv2_frame

    def postprocessing(self):
        # Convert the raw .264 file into an .mp4 file
        h264_filepath = os.path.join(self.tempd, 'h264_data.264')
        mp4_filepath = os.path.join(self.tempd, 'h264_data.mp4')
        subprocess.run(
            shlex.split('ffmpeg -i {} -c:v copy -y {}'.format(
                h264_filepath, mp4_filepath)),
            check=True
        )


def takeoff(drone):
    assert drone(TakeOff()).wait().success()

def land(drone):
    assert drone(Landing()).wait().success()

following = False
def follow_me(drone, parent_conn):
    drone.takeoff()
    time.sleep(2)
    global following
    if not following:
        print("I am in follow me.")
        parent_conn.send("TRUE")
        following = True

def floating(_, parent_conn):
    global following
    if following:
        parent_conn.send("FALSE")
        following = False

def take_picture(tello, _):
    print("I am in taking pictures mode.")
    frame = tello.get_frame_read().frame
    cv2.imwrite("./picture.jpeg", frame)


state_to_func = {
    State.TAKEOFF: takeoff,
    State.LAND: land,
    State.FOLLOW_ME: follow_me,
    State.TAKE_A_PICTURE: take_picture,
    State.FLOAT: floating,
}

def runLive(p):
    p.main()

if __name__ == "__main__":

    damn = time.time()
    time.sleep(5)
    damn = time.time() - damn

    streaming_example = StreamingExample()
    streaming_example.start()


    parent_conn, child_conn = Pipe()
    p = Process(target=process_2, args=(streaming_example.drone, child_conn,))
    p.start()

    # the rest:
    p = PresenterServer(streaming_example.drone)

    t2 = threading.Thread(target=runLive, args=(p,))
    t2.start()

    print("t2 start")
    state = State.FOLLOW_ME
    is_confirm = False
    confirm_timeout = None

    # print("wait pid to do its stuff, 10 sec")
    # time.sleep(10)
    openpose_tf.init(openpose_tf.MODEL_PATH)
    # time.sleep(5)

    try:
        while True:
            func = state_to_func.get(state)
            if func is not None:
                print("Executing function related to state ", state)
                func(streaming_example.drone, parent_conn)
            frame = streaming_example.get_cv2_frame
            command = openpose_tf.get_pose(frame)
            if len(command) > 0:
                command = str(command[0].value)
            else:
                command = "0"

            state = get_next_state(state, command)
            print(state, command)

            if state in [State.TAKEOFF_CONFIRM, State.FOLLOW_ME_CONFIRM, State.LAND_CONFIRM, State.TAKE_A_PICTURE_CONFIRM]:
                confirm_timeout = time.time() + damn
                # command = openpose_tf.get_pose(frame)
                while time.time() < confirm_timeout:
                    frame = streaming_example.get_cv2_frame
                    command = openpose_tf.get_pose(frame)
                    if len(command) > 0:
                        command = str(command[0].value)
                    else:
                        command = "0"
                    print(state, command)
                    if command == "2":
                        state = get_next_state(state, "2")
                        break
    except KeyboardInterrupt:
        land(streaming_example.drone)


    


# if __name__ == "__main__":
    
#     # for i in reversed(range(0, 15)):
#     #     print(f"Starting drone in {i} seconds")
#     #     time.sleep(1)

#     damn = time.time()
#     time.sleep(5)
#     damn = time.time() - damn
    
#     tello = Tello()
#     tello.connect()
#     print(tello.get_battery(), "\n\n\n")
#     tello.streamon()
#     frame= tello.get_frame_read().frame
#     print(frame)


#     parent_conn, child_conn = Pipe()
#     p = Process(target=process_2, args=(tello, child_conn,))
#     p.start()

#     # the rest:
#     p = PresenterServer(tello)

#     t2 = threading.Thread(target=runLive, args=(p,))
#     t2.start()

#     print("t2 start")
#     state = State.FOLLOW_ME
#     is_confirm = False
#     confirm_timeout = None

#     # print("wait pid to do its stuff, 10 sec")
#     # time.sleep(10)
#     openpose_tf.init(openpose_tf.MODEL_PATH)
#     # time.sleep(5)

#     try:
#         while True:
#             func = state_to_func.get(state)
#             if func is not None:
#                 print("Executing function related to state ", state)
#                 func(tello, parent_conn)
#             frame= tello.get_frame_read().frame
#             command = openpose_tf.get_pose(frame)
#             if len(command) > 0:
#                 command = str(command[0].value)
#             else:
#                 command = "0"

#             state = get_next_state(state, command)
#             print(state, command)

#             if state in [State.TAKEOFF_CONFIRM, State.FOLLOW_ME_CONFIRM, State.LAND_CONFIRM, State.TAKE_A_PICTURE_CONFIRM]:
#                 confirm_timeout = time.time() + damn
#                 # command = openpose_tf.get_pose(frame)
#                 while time.time() < confirm_timeout:
#                     frame = tello.get_frame_read().frame
#                     command = openpose_tf.get_pose(frame)
#                     if len(command) > 0:
#                         command = str(command[0].value)
#                     else:
#                         command = "0"
#                     print(state, command)
#                     if command == "2":
#                         state = get_next_state(state, "2")
#                         break
                    
                    
                    
            
                


#             # if not is_confirm and next_state in [State.TAKEOFF_CONFIRM, State.FOLLOW_ME_CONFIRM, State.LAND_CONFIRM, State.TAKE_A_PICTURE_CONFIRM]:
#             #     is_confirm = True
#             #     confirm_timeout = time.time() + damn
#             #     continue

#             # if is_confirm and time.time() < confirm_timeout and command != "2":
#             #     pass
#             # else:
#             #     state = next_state
#             #     is_confirm = False
#             # print(state)
            
#     except KeyboardInterrupt:
#         tello.land()
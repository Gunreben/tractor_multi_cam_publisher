#!/usr/bin/env python3
"""
RTSP Timestamp Checker - Verifies if RTSP streams contain timestamp information
"""

import sys
import time
import argparse
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class RTSPTimestampChecker:
    def __init__(self, rtsp_url, timeout=10):
        self.rtsp_url = rtsp_url
        self.timeout = timeout
        self.mainloop = GLib.MainLoop()
        self.pipeline = None
        self.bus = None
        self.has_timestamps = False
        self.frames_checked = 0
        self.start_time = 0
        
    def build_pipeline(self):
        # Create pipeline that extracts frames but doesn't decode fully
        pipeline_str = (
            f"rtspsrc location={self.rtsp_url} latency=0 ! "
            "rtph264depay ! h264parse ! "
            "identity silent=false signal-handoffs=true name=identity ! "
            "fakesink sync=false"
        )
        
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message", self.on_message)
        
        # Connect to handoff signal to analyze buffers
        identity = self.pipeline.get_by_name("identity")
        identity.connect("handoff", self.on_handoff_signal)
        
    def on_handoff_signal(self, element, buffer):
        # Check if this buffer has timestamp info
        pts_valid = buffer.pts != Gst.CLOCK_TIME_NONE
        dts_valid = buffer.dts != Gst.CLOCK_TIME_NONE
        duration_valid = buffer.duration != Gst.CLOCK_TIME_NONE
        
        self.frames_checked += 1
        
        # Print timestamp info
        if pts_valid or dts_valid or duration_valid:
            self.has_timestamps = True
            print(f"Frame {self.frames_checked}:")
            if pts_valid:
                print(f" - PTS: {buffer.pts / Gst.SECOND:.6f} seconds")
            if dts_valid:
                print(f" - DTS: {buffer.dts / Gst.SECOND:.6f} seconds")
            if duration_valid:
                print(f" - Duration: {buffer.duration / Gst.SECOND:.6f} seconds")
            print()
        
        # After checking enough frames or on timeout, stop
        elapsed_time = time.time() - self.start_time
        if self.frames_checked >= 10 or elapsed_time > self.timeout:
            print(f"Frames checked: {self.frames_checked}")
            print(f"Has timestamps: {self.has_timestamps}")
            print(f"Time elapsed: {elapsed_time:.2f} seconds")
            self.mainloop.quit()
            
        return Gst.PadProbeReturn.OK
        
    def on_message(self, bus, message):
        t = message.type
        
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err.message}")
            self.mainloop.quit()
        
        elif t == Gst.MessageType.EOS:
            print("End of stream")
            self.mainloop.quit()
            
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending_state = message.parse_state_changed()
                if new_state == Gst.State.PLAYING:
                    print(f"Pipeline is now playing for {self.rtsp_url}")
                    self.start_time = time.time()
        
        return True
        
    def run(self):
        # Initialize GStreamer
        Gst.init(None)
        
        # Build and start pipeline
        self.build_pipeline()
        self.pipeline.set_state(Gst.State.PLAYING)
        
        try:
            # Run until we've checked enough frames or timeout
            self.mainloop.run()
        except KeyboardInterrupt:
            print("Interrupted by user")
        finally:
            # Clean up
            self.pipeline.set_state(Gst.State.NULL)
            
        # Return whether timestamps were found
        return self.has_timestamps

def main():
    parser = argparse.ArgumentParser(description="Check if RTSP stream contains timestamp information")
    parser.add_argument("rtsp_url", help="RTSP URL to check (e.g., rtsp://192.168.26.70:8554/h264)")
    parser.add_argument("-t", "--timeout", type=int, default=10, help="Timeout in seconds (default: 10)")
    args = parser.parse_args()
    
    checker = RTSPTimestampChecker(args.rtsp_url, args.timeout)
    has_timestamps = checker.run()
    
    # Final summary
    if has_timestamps:
        print("\nResult: The RTSP stream contains timestamps.")
        print("You can proceed with timestamp extraction in your ROS publisher.")
    else:
        print("\nResult: No timestamps detected in the RTSP stream.")
        print("You may need to use local timestamps or another synchronization method.")
    
    return 0 if has_timestamps else 1

if __name__ == "__main__":
    sys.exit(main())
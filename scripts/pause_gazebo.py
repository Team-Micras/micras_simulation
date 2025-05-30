import gdb
import os
# gz service -s /world/maze/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --req 'pause: true'

def set_gazebo_paused(pause):
    pause_command = "true" if pause else "false"
    command = f"gz service -s /world/maze/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --req 'pause: {pause_command}'"
    print(f"Executing command: {command}")
    os.system(command)

def stop(ev):
    print("Stopping Gazebo...")
    set_gazebo_paused(True)

def cont(ev):
    print("Continuing Gazebo...")
    set_gazebo_paused(False)

gdb.events.stop.connect(stop)
gdb.events.cont.connect(cont)

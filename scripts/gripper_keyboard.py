#!/usr/bin/env python
"""
Gripper Example: keyboard
"""
import argparse

import rospy


from robotiq_urcap_control.gripper import RobotiqGripper
import getch
import numpy as np

def map_keyboard():

    def activate():
        gripper.activate()

    def open_gripper():
        gripper.open()
    
    def close_gripper():
        gripper.close()

    def status():
        print("Current position", gripper.get_current_position())

    def update_delta(delta, increment):
        if delta == 'speed':
            global delta_speed
            delta_speed += increment
            print("speed", delta_speed)
        if delta == 'force':
            global delta_force
            delta_force += increment
            print("force", delta_force)

    def move(open):
        sign = 1 if open else -1
        pos = gripper.get_current_position()
        target = pos + 25 * sign
        gripper.move_and_wait_for_pos(target, delta_speed, delta_force)


    global delta_speed
    global delta_force
    delta_speed = 64
    delta_force = 1

    bindings = {
        #'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        #   key: (function, args, description)
        'a': (activate, [], "Activate gripper"),
        'o': (open_gripper, [], "Open"),
        'c': (close_gripper, [], "Close"),
        'p': (status, [], "Print status"),
        
        # Increase or decrease delta
        'f': (update_delta, ['force', 1.0], "force increase"),
        'r': (update_delta, ['force', -1.0], "force decrease"),
        's': (update_delta, ['speed', 1.0], "speed increase"),
        'w': (update_delta, ['speed', -1.0], "speed decrease"),
        
        'v': (move, [True], "slight open"),
        'b': (move, [False], "slight close"),

    }
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = getch.getch()
        if c:
            # catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c in bindings:
                cmd = bindings[c]
                cmd[0](*cmd[1])
                print(("command: %s" % (cmd[2], )))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(
                        list(bindings.items()), key=lambda x: x[1][2]):
                    print(("  %s: %s" % (key, val[2])))


def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on one of Baxter's arms. Each arm is represented
    by one side of the keyboard and inner/outer key pairings
    on each row for each joint.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(
        formatter_class=arg_fmt, description=main.__doc__, epilog=epilog)
    parser.add_argument(
        '--ip', required=True, type=str, help='robot IP')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("joint_position_keyboard")

    global gripper
    gripper = RobotiqGripper(args.ip)
    gripper.connect()

    map_keyboard()
    print("Done.")


if __name__ == '__main__':
    main()

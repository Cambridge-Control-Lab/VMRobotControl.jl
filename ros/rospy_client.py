#! /usr/bin/env python3
# coding: utf-8
import argparse
import errno
import ipaddress
import socket
import struct
import sys, os
import time

####################################################################################################
# ROS stuff

import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import JointState
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


class ROSManager:
    subscriber_topic = None
    publisher_topic = None
    joint_command_size = None
    joint_state_size = None
    publisher = None
    joint_command_message = None
    rate = None
    new_msg = None

    def __init__(self, subscriber_topic, publisher_topic, joint_command_size, joint_state_size, rate):
        rospy.init_node('vmc_control')
        self.subscriber_topic = subscriber_topic
        self.publisher_topic = publisher_topic
        self.joint_command_size = joint_command_size
        self.joint_state_size = joint_state_size
        rospy.Subscriber(subscriber_topic, JointState, self._subscriber_callback)
        self.publisher = rospy.Publisher(publisher_topic, Float64MultiArray, queue_size=1)
        self.joint_command_message = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.size = joint_command_size
        dim.stride = 1
        dim.label = "joint_effort"
        self.joint_command_message.layout.dim.append(dim)
        self.joint_command_message.data = [0.0] * joint_command_size
        self.rate = rospy.Rate(rate)

    def _subscriber_callback(self, msg):
        assert self.joint_state_size%2 == 0 
        assert len(msg.position) == self.joint_state_size//2
        assert len(msg.velocity) == self.joint_state_size//2
        self.new_msg = msg

        
class JointCommand:
    def __init__(self, sequence_number, timestamp, torques):
        self.sequence_number = sequence_number
        self.timestamp = timestamp
        self.torques = torques

####################################################################################################
# IPC to communicate with Julia

STATE_WAITING = 0
STATE_WARMUP = 1
STATE_ACTIVE = 2
STATE_STOPPED = 3

class IPCManager:
    # Inputs
    listen_ip = None
    listen_port = None
    joint_command_size = None
    joint_state_size = None
    # Constants
    torque_fmt = None
    state_fmt = None
    # State
    command_socket = None
    command_stream = None
    data_socket = None
    send_data_socket = None
    state = None
    sequence_number = None

    def __init__(self, listen_ip, listen_port, joint_command_size, joint_state_size):
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.joint_command_size = joint_command_size
        self.joint_state_size = joint_state_size
        # ! indicates network endianness, Q for unsigned 64 bit integer, d for 64 bit float
        self.torque_fmt = '!QQ' + 'd' * joint_command_size
        self.state_fmt = '!QQ' + 'd' * joint_state_size

    def __enter__(self):
        print("Waiting for connection")
        self.command_socket = self._wait_for_connection()
        # Setup data socket
        (bound_ip, bound_port) = self.command_socket.getsockname()
        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        self.data_socket.bind((bound_ip, bound_port))
        self.data_socket.setblocking(False)        
        
        self.send_data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        self.send_data_socket.setblocking(False)
        (remote_addr, remote_port) = self.command_socket.getpeername()
        self.send_data_socket.connect((remote_addr, remote_port))
        self.command_stream = self.command_socket.makefile('rw')

        self.state = STATE_WAITING
        self.sequence_number = 1

    def _wait_for_connection(self):
        address_family = socket.AF_INET if listen_ip.version == 4 else socket.AF_INET6
        tcp_server = socket.socket(address_family, socket.SOCK_STREAM) # TCP
        tcp_server.bind((listen_ip.exploded, listen_port))
        tcp_server.listen()
        tcp_server.settimeout(5.0)
        while not rospy.is_shutdown():
            try:
                time.sleep(0.0)
                command_socket, (remote_addr, remote_port) = tcp_server.accept()
                print(f"Connection from {remote_addr}:{remote_port}")
                time.sleep(0.5)
                break
            except socket.timeout:
                print("Timeout waiting for connection, retrying.")
        if rospy.is_shutdown():
            raise Exception("ROS was shut down.")
        command_socket.setblocking(False) # Readline will return an empty string if no data is available
        tcp_server.close() # Only allow one connection at a time
        return command_socket


    def __exit__(self, exc_type, exc_value, traceback):
        print("Closing connection")
        self.command_stream.write("STOP\n")
        self.command_stream.flush()
        time.sleep(1.0)
        self.command_stream.close()
        self.command_socket.close()
        self.data_socket.close()
        
        self.command_socket = None
        self.command_stream = None
        self.data_socket = None
        self.send_data_socket = None
        self.state = None

    def recv_joint_command(self):
        n_bytes = struct.calcsize(self.torque_fmt)
        try:
            data = self.data_socket.recv(n_bytes)
            data_unpacked = struct.unpack(self.torque_fmt, data)
            sequence_number = data_unpacked[0]
            timestamp = data_unpacked[1]
            torques = data_unpacked[2:2+self.joint_command_size]
            assert len(torques) == self.joint_command_size
            command = JointCommand(sequence_number, timestamp, torques)
        except socket.error as e:
            if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                print(f"Failed to recv: {e}")
                raise e
            return None
        return command

    def _send_robot_state(self, timestamp, sequence_number, joint_state_vector):
        message = struct.pack(self.state_fmt, 
            timestamp, 
            sequence_number, 
            *joint_state_vector
        )
        (remote_addr, remote_port) = self.command_socket.getpeername()
        result = self.send_data_socket.sendto(message, (remote_addr, remote_port))
        # Check if the entire packet was sent
        if result != len(message):
            print(f"Failed to send: Sent {result} bytes, expected {len(message)} bytes.")
            exit(1)

    def send_robot_state(self, timestamp, joint_state_vector):
        # print(f"Sending packet with sequence number {self.sequence_number}")
        self._send_robot_state(timestamp, self.sequence_number, joint_state_vector)
        self.sequence_number += 1


####################################################################################################

def loop_waiting(socket_manager):
    print("State: WAITING")
    while not rospy.is_shutdown():
        command = socket_manager.command_stream.readline()
        if command == "START\n":
            return STATE_WARMUP
        elif command == "":
            # Send state to julia every 100ms
            if ros_manager.new_msg is not None: # New msg received from ROS subscriber
                forward_state_to_julia(socket_manager, ros_manager)
                print("Sent initial state")
            else:
                print("No state message received from ROS yet... retrying")
            time.sleep(0.5)
        else:
            print(f"Unexpected command in state WAITING: \"{command}\".")
            return STATE_STOPPED

def forward_state_to_julia(socket_manager, ros_manager):
    msg_vec =  []
    msg_vec.extend(ros_manager.new_msg.position)
    msg_vec.extend(ros_manager.new_msg.velocity)
    ros_manager.new_msg = None # Clear the message
    socket_manager.send_robot_state(time.time_ns(), msg_vec) # Send to julia

def send_recv_send_recv_wait(socket_manager, ros_manager, set_zero=False):
    if ros_manager.new_msg is not None: # New msg received from ROS subscriber
        forward_state_to_julia(socket_manager, ros_manager)
    command = socket_manager.recv_joint_command() 
    if command is not None: # New torque command received from julia
        if set_zero:
            ros_manager.joint_command_message.data = 0 * command.torques
        else:
            ros_manager.joint_command_message.data = command.torques
        ros_manager.publisher.publish(ros_manager.joint_command_message) # Publish via ROS
    ros_manager.rate.sleep()

def loop_warmup(socket_manager, ros_manager):
    print("State: WARMUP")
    while not rospy.is_shutdown():
        command = socket_manager.command_stream.readline()
        if command == "":
            # Send zero torques only during warmup
            send_recv_send_recv_wait(socket_manager, ros_manager, set_zero=True)
        elif command == "WARMUP_DONE\n":
            return STATE_ACTIVE
        elif command == "STOP\n":
            return STATE_STOPPED
        else:
            print(f"Unexpected command in state WARMUP: \"{command}\".")
            return STATE_STOPPED

def loop_active(socket_manager, ros_manager):
    print("State: ACTIVE")
    while not rospy.is_shutdown():
        command = socket_manager.command_stream.readline()
        if command == "":
            send_recv_send_recv_wait(socket_manager, ros_manager)
        elif command == "STOP\n":
            return STATE_STOPPED
        else:
            print(f"Unexpected command in state ACTIVE: \"{command}\".")
            return STATE_STOPPED

####################################################################################################
# Main

if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("joint_command_size", type=int)
    parser.add_argument("joint_commands_topic", type=str)
    parser.add_argument("joint_state_topic", type=str)
    parser.add_argument("--rate", type=float, default=1000)
    parser.add_argument("--joint_state_size", default=None)
    parser.add_argument("--listen_port", type=int, default=25342)
    parser.add_argument("--listen_ip", type=str, default="127.0.0.1")
    parser.add_argument("--auto-restart", type=bool, default=True)

    args = parser.parse_args()
    joint_command_size = args.joint_command_size
    joint_state_size = args.joint_state_size if args.joint_state_size is not None else 2 * joint_command_size 
    joint_commands_topic = args.joint_commands_topic
    joint_state_topic = args.joint_state_topic
    listen_port = args.listen_port
    listen_ip = ipaddress.ip_address(args.listen_ip)

    # Setup ROS
    # rospy.init_node('vmc_control')
    ros_manager = ROSManager(
        joint_state_topic,
        joint_commands_topic,
        joint_command_size,
        joint_state_size,
        args.rate
    )

    # Communication with Julia
    socket_manager = IPCManager(
        listen_ip,
        listen_port,
        joint_command_size,
        joint_state_size    
    )

    while not rospy.is_shutdown():
        with socket_manager:
            state = STATE_WAITING
            while not rospy.is_shutdown():
                try:
                    if state == STATE_WAITING:
                        state = loop_waiting(socket_manager)
                    elif state == STATE_WARMUP:
                        state = loop_warmup(socket_manager, ros_manager)
                    elif state == STATE_ACTIVE:
                        state = loop_active(socket_manager, ros_manager)
                    elif state == STATE_STOPPED:
                        break # Loop back to waiting for a new connection
                    else:
                        raise Exception(f"Invalid state: {state}")
                except Exception as e:
                    print(f"Unhandled Exception: {e}")
                if not args.auto_restart:
                    break
        if not args.auto_restart:
            break


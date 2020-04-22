from socket import *
import threading

from utils import *


class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.l_socket = socket(AF_INET, SOCK_STREAM)

        self.threads = {}

    def start(self):
        # Bind the socket and listen
        try:
            self.l_socket.bind((self.host, self.port))
        except OSError:
            self.port = 9090
            self.l_socket.bind((self.host, self.port))
        self.l_socket.listen()

        # As soon as a client connected - set the socket timeout and create a new thread for it
        while True:
            print("Waiting for connection on port", self.port, "...")
            c_socket, address = self.l_socket.accept()
            print("-------------------------------------------------------")
            print("Connected to", address)
            c_socket.settimeout(TIMEOUT)

            self.threads[c_socket] = threading.Thread(target=handle_connection, args=(c_socket,))
            self.threads[c_socket].start()


def handle_connection(connection):
    robot = Robot()

    # There are three phases of maintaining the connection:
    # authorization, moving to the goal initial searching position and searching the final message
    try:
        authorize(connection, robot)
        move_to_goal_init_pos(connection, robot)
        search_goal_zone(connection, robot)

        raise CloseConnection("No final message found")

    # Some error occurred -> print what caused it and close the connection
    except CloseConnection as ex:
        print(*ex.args)
    # Timeout
    except timeout:
        print("Connection timeout")
    # The end: final message found
    except MessageFound as mf:
        connection.sendall(SERVER_LOGOUT)
        print("Message \"", *mf.args, "\" found!", sep="")

    connection.close()
    print("Connection closed")
    print("-------------------------------------------------------")


def authorize(connection, robot):
    # Extract username and compute the robot's login hash
    message = extract_message(connection, robot, "USERNAME")
    robot.hash = (sum(bytearray(message, "ascii")) * 1000) % 65536
    hash_to_send = bytearray(str((robot.hash + SERVER_KEY) % 65536), "ascii") + b"\a\b"

    # print("Sending:", hash_to_send)
    connection.sendall(hash_to_send)

    # Confirm or recharging expected - estimate the message with greater length
    message = extract_message(connection, robot, "RECHARGING")
    login_hash = check_confirmation_message(connection, robot, message)

    # The login hashes do not match
    if login_hash != robot.hash:
        connection.sendall(SERVER_LOGIN_FAILED)
        raise CloseConnection("Login failed")

    connection.sendall(SERVER_OK)


def move_to_goal_init_pos(connection, robot):
    # Sending an initial move command - cannot be ignored by not moving
    # print("Sending initial MOVE")
    connection.sendall(SERVER_MOVE)
    message = extract_message(connection, robot, "OK")
    robot.x, robot.y = check_ok_message(connection, robot, message)

    # Sending the second move command that can be once ignored
    x, y = send_move(connection, robot)

    # Computing initial direction
    robot.direction = Direction(x - robot.x, y - robot.y)
    assert robot.direction != (0, 0)
    robot.move()

    # Moving to the initial goal zone searching position - the bottom left corner
    turn_and_move(True, connection, robot, (GOAL_ZONE_LEFT, GOAL_ZONE_BOTTOM))  # Moving along the x axis
    turn_and_move(False, connection, robot, (GOAL_ZONE_LEFT, GOAL_ZONE_BOTTOM))  # Moving along the y axis


def search_goal_zone(connection, robot):
    # Check if the robot really is in the goal zone initial searching position
    assert (robot.x, robot.y) == (GOAL_ZONE_LEFT, GOAL_ZONE_BOTTOM)

    x_shift = GOAL_ZONE_RIGHT - GOAL_ZONE_LEFT
    y_shift = GOAL_ZONE_TOP - GOAL_ZONE_BOTTOM

    # Move by a spiral path to the center
    while x_shift > 0 and y_shift > 0:
        # Move along the x axis to the bottom right corner
        turn_and_move(True, connection, robot, (robot.x + x_shift, robot.y))
        # Move along the y axis to the top right corner
        turn_and_move(False, connection, robot, (robot.x, robot.y + y_shift))
        # Move along the x axis to the top left corner
        turn_and_move(True, connection, robot, (robot.x - x_shift, robot.y))
        # Move along the y axis to the bottom left corner
        turn_and_move(False, connection, robot, (robot.x, robot.y - y_shift + 1))

        # Move one position to the right to start a new circle
        turn_and_move(True, connection, robot, (robot.x + 1, robot.y))

        x_shift -= 2
        y_shift -= 2


if __name__ == "__main__":
    server = Server("127.0.0.1", 8888)
    server.start()
from collections import namedtuple
import re

SERVER_MOVE = b"102 MOVE\a\b"
SERVER_TURN_LEFT = b"103 TURN LEFT\a\b"
SERVER_TURN_RIGHT = b"104 TURN RIGHT\a\b"
SERVER_PICK_UP = b"105 GET MESSAGE\a\b"
SERVER_LOGOUT = b"106 LOGOUT\a\b"
SERVER_OK = b"200 OK\a\b"
SERVER_LOGIN_FAILED = b"300 LOGIN FAILED\a\b"
SERVER_SYNTAX_ERROR = b"301 SYNTAX ERROR\a\b"
SERVER_LOGIC_ERROR = b"302 LOGIC ERROR\a\b"

TIMEOUT = 1
TIMEOUT_RECHARGING = 5

SERVER_KEY = 54621
CLIENT_KEY = 45328

GOAL_ZONE_LEFT = -2
GOAL_ZONE_RIGHT = 2
GOAL_ZONE_TOP = 2
GOAL_ZONE_BOTTOM = -2

CLIENT_MSG_LENGTHS = {"USERNAME": 12, "CONFIRM": 7, "OK": 12, "RECHARGING": 12, "FULL POWER": 12, "MESSAGE": 100}


Direction = namedtuple("direction", "x y")


class CloseConnection(Exception):
    pass


class InvalidMessageError(Exception):
    pass


class MessageFound(Exception):
    pass


class Robot:
    def __init__(self):
        self.buffer = ""
        self.hash = None

        self.x = None
        self.y = None
        self.direction = None

    def move(self):
        self.x += self.direction.x
        self.y += self.direction.y

    def turn_left(self):
        self.direction = Direction(-self.direction.y, self.direction.x)
        # print("Direction now:", self.direction)

    def turn_right(self):
        self.direction = Direction(self.direction.y, -self.direction.x)
        # print("Direction now:", self.direction)


def extract_message(connection, robot, msg_type):
    assert msg_type in CLIENT_MSG_LENGTHS.keys()
    msg_len = CLIENT_MSG_LENGTHS[msg_type]

    separator = robot.buffer.find("\a\b")
    if separator != -1:  # A full message from this robot is already in its buffer
        message = robot.buffer[:separator]
        robot.buffer = robot.buffer[separator + 2:]

        if len(message) > msg_len:
            connection.sendall(SERVER_SYNTAX_ERROR)
            raise CloseConnection("Invalid format of estimated message in the buffer")

        return message

    # No full message yet -> read data until either data len limit exceeded or full message received
    while True:
        data = connection.recv(msg_len).decode("ascii")
        # print("Received \"", bytes(data, "ascii"), "\"", sep="")

        if data == "":
            raise CloseConnection("Empty data received")

        robot.buffer += data
        separator = robot.buffer.find("\a\b")

        # Continue receiving if the ending sequence is not found yet
        # and the length of data is less than maximum for this type
        if separator == -1:
            if len(data) < msg_len:
                continue
            else:
                connection.sendall(SERVER_SYNTAX_ERROR)
                raise CloseConnection("Invalid format of received message")

        # Extract the message from the buffer without the ending sequence and cut the buffer
        message = robot.buffer[:separator]
        robot.buffer = robot.buffer[separator + 2:]

        return message


def turn_and_move(x_or_y, connection, robot, goal_coordinates):
    if (robot.x, robot.y) == goal_coordinates:
        return

    x_dir_to_move = 1 if robot.x < goal_coordinates[0] else (-1 if robot.x > goal_coordinates[0] else 0)
    y_dir_to_move = 1 if robot.y < goal_coordinates[1] else (-1 if robot.y > goal_coordinates[1] else 0)
    init_coordinates = (robot.x, robot.y)

    # Turn the robot so that it's ready to move forward to the goal coordinates along the given axis (x_or_y flag)
    if     (x_or_y and robot.direction == (0, x_dir_to_move)) \
    or (not x_or_y and robot.direction == (-y_dir_to_move, 0)):
        # Turn 1x to the right
        send_turn(connection, robot, SERVER_TURN_RIGHT)

    elif (x_or_y and x_dir_to_move != 0) or (not x_or_y and y_dir_to_move != 0):
        # Turn 1x or 2x to the left
        while  (x_or_y and robot.direction != (x_dir_to_move, 0)) \
        or (not x_or_y and robot.direction != (0, y_dir_to_move)):
            send_turn(connection, robot, SERVER_TURN_LEFT)

    # The position must not have been changed because the robot only rotated
    assert (robot.x, robot.y) == init_coordinates

    # Move along the flagged axis until the robot is in the goal position
    while  (x_or_y and robot.x != goal_coordinates[0]) \
    or (not x_or_y and robot.y != goal_coordinates[1]):

        x, y = send_move(connection, robot)
        robot.move()
        if robot.x != x or robot.y != y:
            raise CloseConnection("The computed and sent coordinates do not match")

        # Try to pick up a message after every move if at goal zone
        if  robot.x in range(GOAL_ZONE_LEFT, GOAL_ZONE_RIGHT + 1) \
        and robot.y in range(GOAL_ZONE_BOTTOM, GOAL_ZONE_TOP + 1):
            send_pick_up(connection, robot)

    # Check if really in the goal position
    assert robot.x, robot.y == goal_coordinates


def send_move(connection, robot):
    # print("Sending:", SERVER_MOVE)
    connection.sendall(SERVER_MOVE)
    message = extract_message(connection, robot, "OK")
    x, y = check_ok_message(connection, robot, message)

    if (x, y) == (robot.x, robot.y):  # Send again
        # print("Sending:", SERVER_MOVE, "again")
        connection.sendall(SERVER_MOVE)
        message = extract_message(connection, robot, "OK")
        x, y = check_ok_message(connection, robot, message)

    return x, y


def send_turn(connection, robot, to_send):
    assert to_send in [SERVER_TURN_LEFT, SERVER_TURN_RIGHT]

    # print("Sending:", to_send)
    connection.sendall(to_send)
    message = extract_message(connection, robot, "OK")
    x, y = check_ok_message(connection, robot, message)

    if to_send == SERVER_TURN_LEFT:
        robot.turn_left()
    else:
        robot.turn_right()

    if robot.x != x or robot.y != y:
        raise CloseConnection("The computed and sent coordinates do not match")


def send_pick_up(connection, robot):
    # print("Sending:", SERVER_PICK_UP)
    connection.sendall(SERVER_PICK_UP)
    message = extract_message(connection, robot, "MESSAGE")

    if message == "RECHARGING":
        handle_recharging(connection, robot)
        message = extract_message(connection, robot, "MESSAGE")
    if message != "":
        raise MessageFound(message)


def check_confirmation_message(connection, robot, message):
    if message == "RECHARGING":
        handle_recharging(connection, robot)
        message = extract_message(connection, robot, "CONFIRM")

    pattern = re.compile("^\d{1,5}$")

    if pattern.match(message) is None:
        connection.sendall(SERVER_SYNTAX_ERROR)
        raise CloseConnection("Invalid CONFIRMATION message")

    return (int(message) - CLIENT_KEY) % 65536


def check_ok_message(connection, robot, message):
    if message == "RECHARGING":
        handle_recharging(connection, robot)
        message = extract_message(connection, robot, "OK")

    pattern = re.compile("^OK -?\d+ -?\d+$")

    if len(message) > 10 or pattern.match(message) is None:
        connection.sendall(SERVER_SYNTAX_ERROR)
        raise CloseConnection("Invalid OK message")

    return int(message.split()[1]), int(message.split()[2])


def handle_recharging(connection, robot):
    connection.settimeout(TIMEOUT_RECHARGING)
    message = extract_message(connection, robot, "FULL POWER")
    if message != "FULL POWER":
        connection.sendall(SERVER_LOGIC_ERROR)
        raise CloseConnection("Received something else than FULL POWER")
    connection.settimeout(TIMEOUT)

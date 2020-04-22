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

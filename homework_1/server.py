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

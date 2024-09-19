import socket

# Server address
HOST = '127.0.0.1'
PORT = 8080
BUFFER_SIZE = 4096


def call_server(message):
    # Create a TCP/IP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # Connect to server and send data
        s.connect((HOST, PORT))
        s.sendall(message)

        # Receive response from the server
        response = s.recv(BUFFER_SIZE)
    return response.decode()

# call_server(b"""
# set instance burger robot
#
# set instance entrance zone
# set instance table_01 zone
# set instance table_02 zone
# set instance table_03 zone
# set instance table_04 zone
# set instance table_05 zone
# set instance cabinet_01 zone
# set instance cabinet_02 zone
# set instance cabinet_03 zone
# set instance recharge_zone zone
# set instance init_zone zone
# set instance trash zone
#
# set instance hammer piece
#
# set predicate (piece_at hammer table_02)
# set predicate (robot_available burger)
# set predicate (robot_at burger init_zone)
# set predicate (is_recharge_zone recharge_zone)
# get problem predicates
# """)
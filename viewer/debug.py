"""
pip install erlang-py
"""
import socket
import struct
import erlang


def atom_to_str(value):
    """
    Convertit les atomes Erlang décodés par erlang-py en string Python.
    Exemple: OtpErlangAtom(b'nav2') -> 'nav2'
    """
    if hasattr(value, "value"):
        raw = value.value
        if isinstance(raw, bytes):
            return raw.decode()
        return str(raw)

    text = str(value)

    # fallback si l'objet s'affiche comme OtpErlangAtom(b'nav2')
    if "OtpErlangAtom" in text and "b'" in text:
        try:
            return text.split("b'")[1].split("'")[0]
        except Exception:
            return text

    return text


MCAST_GRP = "224.0.2.15"
MCAST_PORT = 62476
PC_IP = "172.20.10.3"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", MCAST_PORT))

mreq = struct.pack(
    "4s4s",
    socket.inet_aton(MCAST_GRP),
    socket.inet_aton(PC_IP)
)

sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

print("Listening Hera multicast...")

while True:
    packet, addr = sock.recvfrom(4096)

    try:
        msg = erlang.binary_to_term(packet)
    except Exception as e:
        print("decode error:", e)
        print(packet[:40])
        continue

    print("From", addr)
    # for i in range(len(msg[3:][1])):
    #     msg[3:][1][i] = round(msg[3:][1][i], 3)
    # print(atom_to_str(msg[0]), atom_to_str(msg[1]), atom_to_str(msg[2]), msg[3:][1][2:6])
    # print(atom_to_str(msg[0]), atom_to_str(msg[1]), atom_to_str(msg[2]), msg[3:])
    print(atom_to_str(msg[1]), atom_to_str(msg[2]), msg[3:])
    print()
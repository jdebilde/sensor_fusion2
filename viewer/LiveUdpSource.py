import threading
import socket
import struct
import erlang

from HeraSample import make_sample
from utils import atom_to_str
from config import MCAST_GRP, MCAST_PORT


class LiveUdpSource(threading.Thread):
    def __init__(self, out_queue, pc_ip, measure_filter, node_filter, format_name):
        super().__init__(daemon=True)
        self.out_queue = out_queue
        self.pc_ip = pc_ip
        self.measure_filter = measure_filter
        self.node_filter = node_filter
        self.format_name = format_name
        self.running = False
        self.sock = None

    def stop(self):
        self.running = False

        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass

            self.sock = None

    def run(self):
        self.running = True

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
            socket.IPPROTO_UDP
        )

        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            self.sock.bind(("", MCAST_PORT))

            mreq = struct.pack(
                "4s4s",
                socket.inet_aton(MCAST_GRP),
                socket.inet_aton(self.pc_ip)
            )

            self.sock.setsockopt(
                socket.IPPROTO_IP,
                socket.IP_ADD_MEMBERSHIP,
                mreq
            )

            self.sock.settimeout(0.5)

        except Exception as e:
            self.out_queue.put(("error", f"Erreur UDP: {e}"))
            self.stop()
            return

        while self.running:
            try:
                packet, addr = self.sock.recvfrom(4096)

            except socket.timeout:
                continue

            except OSError:
                break

            except Exception as e:
                self.out_queue.put(("error", f"Erreur réception UDP: {e}"))
                break

            try:
                msg = erlang.binary_to_term(packet)

            except Exception as e:
                self.out_queue.put(("warning", f"Paquet non décodable: {e}"))
                continue

            try:
                tag, measure, node, seq, values = msg

            except Exception:
                self.out_queue.put(("warning", f"Format inattendu: {msg}"))
                continue

            tag = atom_to_str(tag)
            measure = atom_to_str(measure)
            node = atom_to_str(node)

            if tag != "hera_data":
                continue

            if measure != self.measure_filter:
                continue

            if node != self.node_filter:
                continue

            try:
                seq = int(seq)
                values = [float(v) for v in values]

            except Exception:
                self.out_queue.put(("warning", f"Values invalides: {values}"))
                continue

            try:
                # print("[UDP]", self.format_name, "measure=", measure, "node=", node, "seq=", seq, "values=", values)
                sample = make_sample(
                    measure=measure,
                    node=node,
                    seq=seq,
                    values=values,
                    format_name=self.format_name,
                    hera_timestamp=None
                )

            except Exception as e:
                self.out_queue.put(("warning", f"Sample invalide: {e}"))
                continue

            self.out_queue.put(("sample", sample))

        self.stop()

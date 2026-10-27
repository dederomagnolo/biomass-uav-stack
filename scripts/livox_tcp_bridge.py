#!/usr/bin/env python3
"""Recreate Livox UDP datagrams received over a local TCP stream."""

import socket
import struct


LISTEN_ADDRESS = ("0.0.0.0", 47000)
UDP_ADDRESS = "127.0.0.1"


def receive_exact(connection: socket.socket, length: int) -> bytes | None:
    chunks = bytearray()
    while len(chunks) < length:
        chunk = connection.recv(length - len(chunks))
        if not chunk:
            return None
        chunks.extend(chunk)
    return bytes(chunks)


def serve_connection(connection: socket.socket) -> None:
    udp_sockets: dict[int, socket.socket] = {}
    try:
        while True:
            header = receive_exact(connection, 6)
            if header is None:
                return
            destination_port, payload_length = struct.unpack("!HI", header)
            if payload_length > 65535:
                raise ValueError(f"invalid UDP payload length: {payload_length}")
            payload = receive_exact(connection, payload_length)
            if payload is None:
                return
            udp_socket = udp_sockets.setdefault(destination_port, socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            udp_socket.sendto(payload, (UDP_ADDRESS, destination_port))
    finally:
        for udp_socket in udp_sockets.values():
            udp_socket.close()


def main() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(LISTEN_ADDRESS)
        server.listen(1)
        print(f"Livox TCP bridge listening on {LISTEN_ADDRESS[0]}:{LISTEN_ADDRESS[1]}", flush=True)
        while True:
            connection, peer = server.accept()
            print(f"Livox TCP relay connected from {peer[0]}:{peer[1]}", flush=True)
            with connection:
                try:
                    serve_connection(connection)
                except (ConnectionError, OSError, ValueError) as error:
                    print(f"Livox TCP relay disconnected: {error}", flush=True)


if __name__ == "__main__":
    main()

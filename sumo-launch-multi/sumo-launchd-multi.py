#!/usr/bin/env python3

"""
veins_launchd_multiclient.py

Modified veins_launchd that supports multi-client SUMO (--num-clients 2).
- OMNeT++ connects on the launchd port (default 9999) via the standard
  veins_launchd protocol (sends launch XML, gets proxied to SUMO).
- A second client (your Python script) connects directly to a second
  port (default 9998) that is proxied to SUMO as client 2.

IMPORTANT: Both proxies are started simultaneously only AFTER both
clients have connected. This ensures both can call setOrder before
any simulation steps proceed.

Based on veins_launchd by Christoph Sommer <christoph.sommer@uibk.ac.at>
SPDX-License-Identifier: GPL-2.0-or-later
"""

import os
import sys
import tempfile
import shutil
import socket
import struct
import subprocess
import time
import signal
import xml.dom.minidom
import select
import logging
import threading
from optparse import OptionParser
from _thread import allocate_lock, start_new_thread

_API_VERSION = 1
_LAUNCHD_VERSION = b'veins-launchd-multiclient 1.00'
_CMD_GET_VERSION = 0x00
_CMD_FILE_SEND = 0x75


class UnusedPortLock:
    lock = allocate_lock()

    def __init__(self):
        self.acquired = False

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, *args):
        self.release()

    def acquire(self):
        if not self.acquired:
            UnusedPortLock.lock.acquire()
            self.acquired = True

    def release(self):
        if self.acquired:
            UnusedPortLock.lock.release()
            self.acquired = False


def find_unused_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
    sock.bind(('127.0.0.1', 0))
    sock.listen(socket.SOMAXCONN)
    _, port = sock.getsockname()
    sock.close()
    return port


def forward_connection(client_socket, server_socket, stop_event):
    """Proxy TCP traffic bidirectionally until one side closes or stop is set."""
    client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    while not stop_event.is_set():
        r, _, e = select.select(
            [client_socket, server_socket], [],
            [client_socket, server_socket], 1
        )
        if client_socket in e or server_socket in e:
            break
        if client_socket in r:
            try:
                data = client_socket.recv(65535)
                if not data:
                    break
                server_socket.sendall(data)
            except Exception:
                break
        if server_socket in r:
            try:
                data = server_socket.recv(65535)
                if not data:
                    break
                client_socket.sendall(data)
            except Exception:
                break

    stop_event.set()


def parse_launch_configuration(launch_xml_string):
    p = xml.dom.minidom.parseString(launch_xml_string)
    launch_node = p.documentElement
    if launch_node.tagName != "launch":
        raise RuntimeError("Root element not <launch>")

    basedir = ""
    basedir_nodes = [x for x in launch_node.getElementsByTagName("basedir")
                     if x.parentNode == launch_node]
    if len(basedir_nodes) == 1:
        basedir = basedir_nodes[0].getAttribute("path")

    seed = 23423
    seed_nodes = [x for x in launch_node.getElementsByTagName("seed")
                  if x.parentNode == launch_node]
    if len(seed_nodes) == 1:
        seed = int(seed_nodes[0].getAttribute("value"))

    copy_nodes = [x for x in launch_node.getElementsByTagName("copy")
                  if x.parentNode == launch_node]

    return basedir, copy_nodes, seed


def set_sumoconfig_option(config_parser, config_xml, section, key, value):
    key_nodes = config_xml.getElementsByTagName(key)
    if len(key_nodes) < 1:
        key_node = config_parser.createElement(key)
        key_node.setAttribute("value", str(value))
        config_xml.appendChild(key_node)
    else:
        key_node = key_nodes[0]
        key_node.setAttribute("value", str(value))


def copy_and_modify_files(basedir, copy_nodes, runpath, remote_port, seed,
                          num_clients):
    """Copy files and inject --remote-port, --seed, and --num-clients."""
    config_file_name = None
    for copy_node in copy_nodes:
        file_src_name = None
        file_dst_name = None
        file_contents = None

        if copy_node.hasAttribute("file"):
            file_src_name = copy_node.getAttribute("file")
            file_src_path = os.path.join(basedir, file_src_name)
            if not os.path.exists(file_src_path):
                raise RuntimeError(f'File "{file_src_path}" does not exist')
            with open(file_src_path, 'rb') as fh:
                file_contents = fh.read()

        if copy_node.hasAttribute("name"):
            file_dst_name = copy_node.getAttribute("name")
        elif file_src_name:
            file_dst_name = file_src_name

        if copy_node.getAttribute("type") == "config":
            config_file_name = file_dst_name
            config_parser = xml.dom.minidom.parseString(file_contents)
            config_xml = config_parser.documentElement
            set_sumoconfig_option(config_parser, config_xml,
                                  "traci_server", "remote-port", remote_port)
            set_sumoconfig_option(config_parser, config_xml,
                                  "random_number", "seed", seed)
            set_sumoconfig_option(config_parser, config_xml,
                                  "random_number", "random", "false")
            # Inject num-clients into the sumocfg
            set_sumoconfig_option(config_parser, config_xml,
                                  "num-clients", "num-clients", num_clients)
            file_contents = config_xml.toxml('utf-8')

        file_dst_path = os.path.join(runpath, file_dst_name)
        with open(file_dst_path, "wb") as fh:
            fh.write(file_contents)

    if not config_file_name:
        raise RuntimeError('No <copy> node with type="config"')
    return config_file_name


def connect_to_sumo(remote_port, label, max_attempts=30):
    """Try to connect a TCP socket to SUMO, retrying until success."""
    for attempt in range(max_attempts):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(('127.0.0.1', remote_port))
            logging.info(f"Connected to SUMO as {label}")
            return s
        except socket.error:
            time.sleep(0.5)
    raise RuntimeError(f"Could not connect to SUMO as {label}")


def run_sumo_multiclient(runpath, sumo_command, shlex_mode, config_file_name,
                         remote_port, seed, omnet_client_socket,
                         unused_port_lock, keep_temp, python_port):
    """
    Launch SUMO with --num-clients 2.
    
    Connection sequence (order matters!):
    1. Launch SUMO
    2. Connect proxy socket #1 to SUMO (will carry OMNeT++ traffic)
    3. Connect proxy socket #2 to SUMO (will carry Python traffic)
    4. Wait for Python script to connect to python_port
    5. Start BOTH proxies simultaneously so both clients can setOrder
       before any simulationStep proceeds
    """
    sumo_log_out = open(os.path.join(runpath, 'sumo-launchd.out.log'), 'w')
    sumo_log_err = open(os.path.join(runpath, 'sumo-launchd.err.log'), 'w')

    try:
        if shlex_mode:
            import shlex
            cmd = shlex.split(
                sumo_command.replace('{}', '-c ' + config_file_name))
        else:
            cmd = [sumo_command, "-c", config_file_name]

        logging.info("Starting SUMO (%s) on port %d, seed %d, num-clients=2"
                     % (" ".join(cmd), remote_port, seed))
        sumo = subprocess.Popen(cmd, cwd=runpath, stdin=None,
                                stdout=sumo_log_out, stderr=sumo_log_err)

        # --- Connect BOTH proxy sockets to SUMO before starting proxies ---
        # Client 1 connection (for OMNeT++)
        omnet_sumo_socket = connect_to_sumo(remote_port, "client 1 (OMNeT++)")
        # Client 2 connection (for Python)
        python_sumo_socket = connect_to_sumo(remote_port, "client 2 (Python)")

        unused_port_lock.release()

        # --- Wait for Python script to connect ---
        py_listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        py_listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        py_listener.bind(('127.0.0.1', python_port))
        py_listener.listen(1)

        print("=" * 60)
        print(f"  SUMO launched on internal port {remote_port}")
        print(f"  OMNeT++ proxied via launchd protocol (port 9999)")
        print(f"  >>> Now run your Python script with:")
        print(f"  >>> traci.init(port={python_port})")
        print("=" * 60)

        py_client_socket, py_addr = py_listener.accept()
        logging.info(f"Python client connected from {py_addr}")
        py_listener.close()

        print("Both clients connected! Starting simulation proxies...")

        # --- Start BOTH proxies at the same time ---
        # This is critical: both must be active so both clients
        # can send setOrder before any simulationStep is processed
        stop_event = threading.Event()

        omnet_thread = threading.Thread(
            target=forward_connection,
            args=(omnet_client_socket, omnet_sumo_socket, stop_event),
            daemon=True
        )
        python_thread = threading.Thread(
            target=forward_connection,
            args=(py_client_socket, python_sumo_socket, stop_event),
            daemon=True
        )

        omnet_thread.start()
        python_thread.start()

        # Wait for either proxy to finish (simulation ended)
        omnet_thread.join()
        python_thread.join()

        # Cleanup sockets
        for s in (omnet_client_socket, omnet_sumo_socket,
                  py_client_socket, python_sumo_socket):
            try:
                s.close()
            except Exception:
                pass

        logging.debug("Proxies done, killing SUMO")
        start_new_thread(subprocess.Popen.wait, (sumo,))
        time.sleep(0.5)
        if sumo.returncode is None:
            os.kill(sumo.pid, signal.SIGTERM)
            time.sleep(0.5)
        if sumo.returncode is None:
            os.kill(sumo.pid, signal.SIGKILL)

        logging.info("SUMO terminated. Done.")

    except Exception as e:
        logging.error(f"Error: {e}")
        import traceback
        traceback.print_exc()

    sumo_log_out.close()
    sumo_log_err.close()


def handle_get_version(conn):
    response = struct.pack(
        "!iBBBiBBii",
        4 + 1 + 1 + 1 + 4 + 1 + 1 + 4 + 4 + len(_LAUNCHD_VERSION),
        1 + 1 + 1 + 4, _CMD_GET_VERSION, 0x00, 0x00,
        1 + 1 + 4 + 4 + len(_LAUNCHD_VERSION),
        _CMD_GET_VERSION, _API_VERSION, len(_LAUNCHD_VERSION)
    )
    response += _LAUNCHD_VERSION
    conn.send(response)


def read_launch_config(conn):
    msg_len_buf = b""
    while len(msg_len_buf) < 4:
        msg_len_buf += conn.recv(4 - len(msg_len_buf))
    msg_len = struct.unpack("!i", msg_len_buf)[0] - 4

    cmd_len_buf = conn.recv(1)
    cmd_len = struct.unpack("!B", cmd_len_buf)[0] - 1
    cmd_len_buf2 = b""
    if cmd_len == -1:
        while len(cmd_len_buf2) < 4:
            cmd_len_buf2 += conn.recv(4 - len(cmd_len_buf2))
        cmd_len = struct.unpack("!i", cmd_len_buf2)[0] - 5

    cmd_id_buf = conn.recv(1)
    cmd_id = struct.unpack("!B", cmd_id_buf)[0]

    if cmd_id == _CMD_GET_VERSION:
        handle_get_version(conn)
        return read_launch_config(conn)
    elif cmd_id != _CMD_FILE_SEND:
        raise RuntimeError("Expected CMD_FILE_SEND (0x%x), got 0x%x"
                           % (_CMD_FILE_SEND, cmd_id))

    fname_len_buf = b""
    while len(fname_len_buf) < 4:
        fname_len_buf += conn.recv(4 - len(fname_len_buf))
    fname_len = struct.unpack("!i", fname_len_buf)[0]
    fname = conn.recv(fname_len)

    data_len_buf = b""
    while len(data_len_buf) < 4:
        data_len_buf += conn.recv(4 - len(data_len_buf))
    data_len = struct.unpack("!i", data_len_buf)[0]
    data = conn.recv(data_len)

    response = struct.pack("!iBBBi", 4 + 1 + 1 + 1 + 4,
                           1 + 1 + 1 + 4, _CMD_FILE_SEND, 0x00, 0x00)
    conn.send(response)

    return data


def handle_connection(sumo_command, shlex_mode, conn, addr,
                      keep_temp, python_port):
    logging.debug("Handling connection from %s:%d" % addr)
    try:
        data = read_launch_config(conn)

        runpath = tempfile.mkdtemp(prefix="sumo-launchd-tmp-")
        unused_port_lock = UnusedPortLock()

        try:
            basedir, copy_nodes, seed = parse_launch_configuration(data)
            unused_port_lock.acquire()
            remote_port = find_unused_port()

            config_file_name = copy_and_modify_files(
                basedir, copy_nodes, runpath, remote_port, seed,
                num_clients=2
            )

            run_sumo_multiclient(
                runpath, sumo_command, shlex_mode, config_file_name,
                remote_port, seed, conn, unused_port_lock, keep_temp,
                python_port
            )
        finally:
            unused_port_lock.release()
            if not keep_temp:
                shutil.rmtree(runpath, ignore_errors=True)

    except Exception as e:
        logging.error("Error: %s" % e)
        import traceback
        traceback.print_exc()
    finally:
        try:
            conn.close()
        except Exception:
            pass


def main():
    parser = OptionParser()
    parser.add_option("-c", "--command", dest="command", default="sumo",
                      help="SUMO command [default: %default]")
    parser.add_option("-s", "--shlex", dest="shlex", default=False,
                      action="store_true")
    parser.add_option("-p", "--port", dest="port", type="int", default=9999,
                      help="OMNeT++ listen port [default: %default]")
    parser.add_option("--python-port", dest="python_port", type="int",
                      default=9998,
                      help="Port for Python TraCI client [default: %default]")
    parser.add_option("-b", "--bind", dest="bind", default="127.0.0.1")
    parser.add_option("-L", "--logfile", dest="logfile",
                      default=os.path.join(tempfile.gettempdir(),
                                           "sumo-launchd.log"))
    parser.add_option("-v", "--verbose", dest="count_verbose", default=0,
                      action="count")
    parser.add_option("-t", "--keep-temp", dest="keep_temp", default=False,
                      action="store_true")
    options, _ = parser.parse_args()

    _LOGLEVELS = (logging.ERROR, logging.WARN, logging.INFO, logging.DEBUG)
    loglevel = _LOGLEVELS[min(1 + options.count_verbose, len(_LOGLEVELS) - 1)]
    logging.basicConfig(filename=options.logfile, level=loglevel)
    logging.getLogger().addHandler(logging.StreamHandler())

    signal.signal(signal.SIGTERM, lambda s, f: sys.exit(1))

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((options.bind, options.port))
    listener.listen(5)

    print(f"veins_launchd_multiclient listening:")
    print(f"  OMNeT++ port: {options.port}")
    print(f"  Python  port: {options.python_port}")

    try:
        while True:
            conn, addr = listener.accept()
            start_new_thread(handle_connection,
                             (options.command, options.shlex, conn, addr,
                              options.keep_temp, options.python_port))
    except KeyboardInterrupt:
        logging.warning("Keyboard interrupt")
    finally:
        listener.close()


if __name__ == '__main__':
    main()
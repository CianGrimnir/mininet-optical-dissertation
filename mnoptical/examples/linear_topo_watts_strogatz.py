#!/usr/bin/env python3

"""
ringtopo.py: unidirectional ring network with 1-degree ROADMs
            and bidirectional Terminal<->ROADM links
"""

from mnoptical.dataplane import (OpticalLink as OLink,
                                 UnidirectionalOpticalLink as ULink,
                                 ROADM, Terminal,
                                 OpticalNet as Mininet,
                                 km, m, dB, dBm)

from mnoptical.rest import RestServer

from mnoptical.ofcdemo.demolib import OpticalCLI, cleanup
from mnoptical.examples.singleroadm import plotNet

from mininet.topo import Topo
from mininet.examples.linuxrouter import LinuxRouter
from mininet.log import setLogLevel, info, debug
from mininet.clean import cleanup

from functools import partial
from sys import argv
import numpy as np
import networkx as nx
from utils import Queue
from utils import NodeInformation

connection_detail = []


class LinearTopo(Topo):
    """Parametrized unidirectional ROADM ring network
       ROADM r{i} port numbering:
       - 1: line in
       - 2: line out
       - 3..N+2: add/drop ports from/to terminal
       Terminal t{i} port numbering:
       - 1..N: transceiver uplink/downlink ports (bidirectional)
               to/from ROADM
       - N+1..2*N ethernet ports (bidirectional) to/from router
       Router s{i} port numbering (N==nodecount==txcount):
       - 1..N: uplinks/downlinks to/from terminal
       - N+1: local port, connected to host
    """

    def build(self, power=0 * dBm, N=3, k=4, p=0, start_node=1, end_node=2):
        """Create a 1-degree ROADM ring network, with the specified
           operational power and N ROADM/Terminal/Router/Host nodes"""
        self.N = N
        halfk = k // 2
        links = {}
        neigh_list = {}
        neigh_metadata = {}
        neigh_graph = nx.Graph()
        seed = np.random.RandomState(42)
        nodes = list(range(1, N + 1))
        # Nodes/POPs: ROADM/Terminal/Router/Host
        rparams = {'monitor_mode': 'in'}
        transceivers = tuple((f'tx{ch}', power) for ch in range(1, N + 1))
        tparams = {'transceivers': transceivers, 'monitor_mode': 'in'}
        for i in range(1, N + 1):
            self.addSwitch(f'r{i}', cls=ROADM, **rparams)
            self.addSwitch(f't{i}', cls=Terminal, **tparams)
            self.addNode(f's{i}', cls=LinuxRouter)
            self.addHost(f'h{i}')

        # Optical WAN link parameters
        boost = ('boost', {'target_gain': 17 * dB})
        aparams = {'target_gain': 25 * km * .22}
        spans = [25 * km, ('amp1', aparams), 25 * km, ('amp2', aparams), 25 * km, ('amp3', aparams), 25 * km, ('amp4', aparams)]
        # Optical and packet links
        for i in range(1, N + 1):
            # Unidirectional roadm->roadm optical links
            for j in range(1, halfk + 1):
                neig_node = i % N + j
                neig_node = neig_node if neig_node <= N else neig_node - N
                if neig_node == 1 and i == N:
                    print(f"last node connection {i} {neig_node}")
                    continue
                debug(f'r{i}', f'r{neig_node}\n')
                neigh_node = self.watts_strogatz_calc(i, neig_node, nodes, p, seed)
                links[f'r{i}'] = links.get(f'r{i}', {'linein': 1, 'lineout': 2})
                links[f'r{neigh_node}'] = links.get(f'r{neigh_node}', {'linein': 1, 'lineout': 2})
                lineout = links[f'r{i}']['lineout']
                linein = links[f'r{neigh_node}']['linein']
                debug(f'new r{i} r{neigh_node} {lineout} {linein}\n')
                connected_node = NodeInformation(f'r{neigh_node}', f'r{i}', linein, lineout)
                neigh_list.setdefault(f'r{i}', []).append(f'r{neigh_node}')
                neigh_metadata.setdefault(f'r{i}', []).append({f'r{neigh_node}': connected_node})
                neigh_graph.add_edge(f'r{i}', f'r{neigh_node}')
                self.addLink(f'r{i}', f'r{neigh_node}',
                             port1=lineout, port2=linein,
                             boost=boost, spans=spans, cls=OLink)  # ULink
                links[f'r{i}']['lineout'] += 2
                links[f'r{neigh_node}']['linein'] += 2
            lineout = links[f'r{i}']['lineout']
            for port in range(1, N + 1):
                # Bidirectional terminal <-> roadm optical links
                debug(f't{i} r{i} {port+2} {lineout+port+2}\n')
                self.addLink(f't{i}', f'r{i}',
                             port1=port+2, port2=lineout + port+2,
                             spans=[1 * m], cls=OLink)
                # Terminal<->router ethernet links
                self.addLink(f's{i}', f't{i}', port1=port, port2=N + port)
            # Host-switch ethernet link
            self.addLink(f'h{i}', f's{i}', port2=N + 1)
        debug(f"links details - {links}\n")
        debug(f"neighbour nodes - \n {neigh_list}\n")
        debug(f"neighbour metadata - \n {neigh_metadata} \n")
        get_path = self.bfs(neigh_graph, f'r{start_node}', f'r{end_node}')
        global connection_detail
        connection_detail = self.get_connection_detail(get_path, neigh_list, neigh_metadata, f'r{start_node}')

    @staticmethod
    def get_connection_detail(shortest_path, neigh_list, neigh_metadata, initial_node):
        debug(f"path from {initial_node} {shortest_path}\n")
        connection = []
        for path in shortest_path:
            debug(f'initial - {path} {initial_node} {neigh_list}\n')
            if path in neigh_list[initial_node]:
                debug(f'properflow - {initial_node} {path}\n')
                neigh_node = neigh_metadata[initial_node]
                reverse = False
                find_node, node = path, initial_node
            elif initial_node in neigh_list[path]:
                debug(f'reverseflow - {path} {initial_node}\n')
                neigh_node = neigh_metadata[path]
                reverse = True
                find_node, node = initial_node, path
            else:
                debug(f"something a miss {initial_node} {path}\n")
                break
            debug(f"neigh_list - {neigh_node}")
            debug(f"node neighbour - {node} {find_node}\n")
            neigh_obj = [list(k.values())[0] for k in neigh_node if list(k.keys())[0] == find_node][0]
            lineout, linein = neigh_obj.get_link()
            if reverse:
                find_node, node = node, find_node
            info(f"connection link - {node} {find_node} {lineout} {linein} \n")
            node_info = NodeInformation(node, find_node, linein, lineout, reverse)
            connection.append(node_info)
            initial_node = path
        info(f"Connection {[i.__dict__ for i in connection]} \n")
        return connection

    @staticmethod
    def watts_strogatz_calc(curr_node, neigh_node, nodes, p, seed):
        debug(f"watts -> {curr_node} {neigh_node}\n")
        if seed.random() < p:
            # to avoid loop connection
            choices = [e for e in nodes if e not in (curr_node, neigh_node)]
            new_neigh_node = seed.choice(choices)
            debug(f"watts new connection -> {curr_node} {new_neigh_node}\n")
            return new_neigh_node
        else:
            debug(f"no change for {curr_node}")
            return neigh_node

    @staticmethod
    def bfs(graph, node, target):
        visited = []
        actions = []
        queue = Queue()
        start_node = node
        if start_node == target:
            return []
        queue.push((start_node, actions))
        while not queue.isEmpty():
            node, action = queue.pop()
            if node not in visited:
                visited.append(node)
                if node == target:
                    return action
                for neighbour in graph[node]:
                    new_action = action + [neighbour]
                    queue.push((neighbour, new_action))
        return []


# Helper functions

linein, lineout = 1, 2


def fwd(roadm, channels):
    info(roadm, 'fwd', channels, '\n')
    roadm.connect(linein, lineout, channels)


def drop(roadm, dst, channel):
    info(roadm, 'drop', channel, '\n')
    roadm.connect(linein, lineout + dst, [channel])


def add(roadm, src, channel):
    info(roadm, 'add', channel, '\n')
    roadm.connect(lineout + src, lineout, [channel])


# Configuration (for testing, etc.) using internal/native control API
def configNet(net, connection, start, end):
    """
    Configure connection between ROADMs and Terminals for ring topology
    """
    info("Configuring network...\n")
    N = net.topo.N
    channels = [1, 3, 5, 7]
    defaultEthPort = 20
    defaultWDMPort = 2
    # Terminal hostport<->(uplink,downlink)
    for ch in channels:
        ethPort = defaultEthPort + ch
        wdmPort = defaultWDMPort + ch
        info(f"t{start} t{end} line terminal connection - {ethPort} {wdmPort} \n")
        net[f't{start}'].connect(ethPort=ethPort, wdmPort=wdmPort, channel=ch)
        net[f't{end}'].connect(ethPort=ethPort, wdmPort=wdmPort, channel=ch)

    # Configuring ROADM to forward ch1 from t1 to t2"
    for index, conn in enumerate(connection):
        info(f"connection config - {conn.__dict__}\n")
        for ch in channels:
            terminal_port = neigh_forward_port = ch + 8
            node = conn.node_id
            neigh_node = conn.neigh_id
            default_linein = conn.linein
            default_lineout = conn.lineout
            if conn.reverse:
                default_lineout, default_linein = default_linein, default_lineout
            if index != 0:
                if (conn.reverse and connection[index - 1].reverse) or (not conn.reverse and connection[index - 1].reverse):
                    terminal_port = connection[index - 1].lineout
                else:
                    terminal_port = connection[index - 1].linein
            if index != (len(connection)-1):
                if conn.reverse and connection[index + 1].reverse:
                    neigh_forward_port = connection[index + 1].linein
                elif connection[index + 1].reverse:
                    if conn.reverse:
                        neigh_forward_port = connection[index + 1].lineout
                    else:
                        neigh_forward_port = connection[index + 1].linein
                else:
                    neigh_forward_port = connection[index+1].lineout
            info(f'neighbour - {neigh_node} start_node - {node}\n')
            info(f'roadm term - {terminal_port} neigh term - {neigh_forward_port} lineout - {default_lineout} linein - {default_linein}\n')
            net[node].connect(terminal_port, default_lineout, channels=[ch])
            net[neigh_node].connect(default_linein, neigh_forward_port, channels=[ch])
    # Power up transceivers
    info('*** Turning on transceivers... \n')
    net[f't{start}'].turn_on()
    net[f't{end}'].turn_on()


def configOpticalNet(net):
    """Configure ring of ROADMS and Terminals.
       We connect a full mesh of t{1..N}<->t{1..N}"""
    info("*** Configuring network...\n")
    N = net.topo.N
    # Allocate channel for each pair of nodes
    channels, ch = {}, 1
    for i in range(1, N + 1):
        for j in range(i + 1, N + 1):
            channels[i, j] = channels[j, i] = ch
            ch += 1
    allchannels = set(channels.values())
    # Configure devices
    for i in range(1, N + 1):
        # 1-degree/unidirectional ROADMs:
        # Add and drop local channels and pass others
        roadm = net[f'r{i}']
        localchannels = set()
        for j in range(1, N + 1):
            if i == j: continue
            addch, dropch = channels[i, j], channels[j, i]
            add(roadm, j, addch)
            drop(roadm, j, dropch)
            localchannels.update({addch, dropch})
        fwd(roadm, allchannels - localchannels)
        # Terminals (bidirectional)
        # Pass Ethernet ports through to WDM ports
        # on the appropriate channel
        terminal = net[f't{i}']
        for j in range(1, N + 1):
            if i == j: continue
            ethPort, wdmPort = j + N, j
            debug(f'*** {terminal}-eth{ethPort} <->'
                  f' {terminal}-wdm{wdmPort}\n')
            terminal.connect(ethPort=ethPort, wdmPort=wdmPort,
                             channel=channels[i, j])
    # Turn on Terminals/transceivers
    for i in range(1, N + 1):
        net[f't{i}'].turn_on()


def config(net):
    """Configure optical and packet network"""
    configOpticalNet(net)


class CLI(OpticalCLI):
    """CLI with config command"""

    def do_config(self, _line):
        config(self.mn)


def test(net):
    """Run script in test mode"""
    config(net)
    assert net.pingAll() == 0  # 0% loss


if __name__ == '__main__':
    cleanup()  # Just in case!
    setLogLevel('info')
    if 'clean' in argv: exit(0)
    start = 1
    end = 14
    topo = LinearTopo(N=20, p=0.15, start_node=start, end_node=end)
    net = Mininet(topo=topo)
    print("starting model --- ")
    restServer = RestServer(net)
    net.start()
    restServer.start()
    plotNet(net, outfile='linear_topo-watts_plot.png', directed=True)
    configNet(net, connection_detail, start, end)
    if 'test' in argv:
        test(net)
    else:
        CLI(net)
    restServer.stop()
    net.stop()
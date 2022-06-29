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


class RingTopo(Topo):
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

    def build(self, power=0 * dBm, N=3, k=4):
        """Create a 1-degree ROADM ring network, with the specified
           operational power and N ROADM/Terminal/Router/Host nodes"""
        self.N = N
        halfk = k//2
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
        aparams = {'target_gain': 50 * km * .22}
        spans = [50 * km, ('amp1', aparams), 50 * km, ('amp2', aparams), 50 * km, ('amp3', aparams), 50 * km, ('amp4', aparams)]
        # outer_linein, outer_lineout = 3, 4
        # Optical and packet links
        for i in range(1, N + 1):
            # Unidirectional roadm->roadm optical links
            # outer_node = i + 2
            # outer_node = outer_node if outer_node <= N else outer_node - N
            # print(f"outer node - {i} - {outer_node}")
            linein, lineout = 1, 2
            for j in range(1, halfk + 1):
                neig_node = i % N + j
                neig_node = neig_node if neig_node <= N else neig_node - N
                print(f'r{i}', f'r{neig_node}', lineout, linein)
                self.addLink(f'r{i}', f'r{neig_node}',
                             port1=lineout, port2=linein,
                             boost=boost, spans=spans, cls=OLink)  # ULink
                linein += 2
                lineout += 2
            for port in range(1, N + 1):
                # Bidirectional terminal <-> roadm optical links
                self.addLink(f't{i}', f'r{i}',
                             port1=port, port2=lineout + port,
                             spans=[1 * m], cls=OLink)
                # Terminal<->router ethernet links
                self.addLink(f's{i}', f't{i}', port1=port, port2=N + port)
            # Host-switch ethernet link
            self.addLink(f'h{i}', f's{i}', port2=N + 1)


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
def configNet(net):
    """
    Configure connection between ROADMs and Terminals for ring topology
    """
    info("Configuring network...\n")
    N = net.topo.N
    channels = [1, 3, 5, 7]
    defaultEthPort = 20
    defaultWDMPort = 1
    # Terminal hostport<->(uplink,downlink)
    for ch in channels:
        ethPort = defaultEthPort + ch
        wdmPort = defaultWDMPort + ch
        net['t1'].connect(ethPort=ethPort, wdmPort=wdmPort, channel=ch)
        net['t2'].connect(ethPort=ethPort, wdmPort=wdmPort, channel=ch)
        print(ethPort, wdmPort)
    # Configuring ROADM to forward ch1 from t1 to t2"
    default_lineout = 2
    default_linein = 1
    for ch in channels:
        terminal_port = ch + 7
        print(f'roadm {terminal_port} {default_lineout} {default_linein}')
        net['r1'].connect(terminal_port, default_lineout, channels=[ch])
        net['r2'].connect(default_linein, terminal_port, channels=[ch])
    # Power up transceivers
    info('*** Turning on transceivers... \n')
    net[f't1'].turn_on()
    net[f't2'].turn_on()


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
    topo = RingTopo(N=20)
    net = Mininet(topo=topo)
    restServer = RestServer(net)
    net.start()
    restServer.start()
    plotNet(net, outfile='ringtopo-watts_new.png', directed=True)
    configNet(net)
    if 'test' in argv:
        test(net)
    else:
        CLI(net)
    restServer.stop()
    net.stop()

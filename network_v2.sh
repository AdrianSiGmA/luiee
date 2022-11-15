#!/bin/bash

###############################################################################
#
# Script Name:   network.sh
# Description:   configures system to simulate Remote Control
# Args:          1st nic name; 2nd nic name; groupnumber; optional internet_nic
# Usage:         ./network.sh eno1 eno2 17 eno3
# Author:        Jérôme Schürmann, HSLU
# Created:       17.01.2020, sco
# Revison(1.1):  24.01.2020, gep
# Revison(2.0):  23.06.2021, juj & gep
# Version:       2.0
#
###############################################################################

#show message if arguments missing
echo ${1:?"first network card name"} > /dev/null;
echo ${2:?"second network card name"} > /dev/null;
echo ${3:?"groupnumber (e.g. 15)"} > /dev/null;
ControlRoom=$1
TestBed=$2
net=$(echo $3 | sed 's/^0*//')

#optional internet access over third network adapter
internet_uplink=${4: }

#Enable ip-forwarding
echo 'net.ipv4.ip_forward=1' > /etc/sysctl.conf

#previous config cleanup
rm /etc/network/interfaces.d/*

if [ -n "$internet_uplink" ]; then

echo "system with internet access - internet access over $internet_uplink adapter"
cat <<EOF > /etc/network/interfaces.d/$internet_uplink
auto $internet_uplink
iface $internet_uplink inet dhcp
pre-up iptables-restore < /etc/network/iptables.rules
EOF

if [ ! -f "/etc/network/iptables.rules.bkp" ]; then
  iptables-save > /etc/network/iptables.rules.bkp
  echo "created iptables backup (iptables.rules.bkp)"
fi

iptables -t nat -A POSTROUTING -o $internet_uplink -j MASQUERADE
iptables -A FORWARD -i $internet_uplink -j ACCEPT

iptables-save > /etc/network/iptables.rules
fi


interface_bkp_config=/etc/network/interfaces.bkp

if [ ! -f "$interface_bkp_config" ]; then
    cp /etc/network/interfaces{,.bkp}
    echo "create interface config backup (interfaces.bkp)"
fi

#Delete default network configuration

cat <<EOF > /etc/network/interfaces.d/$ControlRoom

#$ControlRoom
auto $ControlRoom
iface $ControlRoom inet static
address 172.20.$net.1
netmask 255.255.255.0
mtu 1300
post-up ifconfig $ControlRoom -multicast
EOF

cat <<EOF > /etc/network/interfaces.d/$TestBed
#$TestBed
auto $TestBed
iface $TestBed inet static
address 172.30.$net.1
netmask 255.255.255.0
mtu 1300
post-up ifconfig $TestBed -multicast
EOF


#rm /etc/network/interfaces
cat <<EOF > /etc/network/interfaces
#default config
source /etc/network/interfaces.d/*

#The Loopback network interface
auto lo
iface lo inet loopback
EOF

echo "new interface config written"

#Restart networking service, to save the new interface configuration permanent
service networking stop
echo "networking service stopped"
service networking start
echo "networking services started"
echo "result form networking service"
network_status=$(service networking status | grep -o 'Active.*')
tput setaf 3 #yellow
echo "--> ${network_status}"
tput setaf 7

#Start the interfaces
ifconfig $ControlRoom up
ifconfig $TestBed up
echo "interfaces $ControlRoom, $TestBed brought up"

dhcp_bkp_config=/etc/default/isc-dhcp-server.bkp
if [ ! -f "$dhcp_bkp_config" ] ; then
    cp /etc/default/isc-dhcp-server{,.bkp}
    echo "create dhcp config backup (isc-dhcp-server.bkp)"
fi

#DHCP define Interfaces in /etc/default/isc-dhcp-server
  > /etc/default/isc-dhcp-server
echo 'INTERFACES="'$ControlRoom' '$TestBed'"' > /etc/default/isc-dhcp-server

dhcpd_bkp_config=/etc/dhcp/dhcpd.conf

if [ ! -f "$dhcpd_bkp_config" ] ; then
    cp /etc/dhcp/dhcpd.conf{.,bkp}
    echo "create dhcpd config backup (dhcpd.conf)"
fi

#rm /etc/dhcp/dhcpd.conf
cat <<EOF > /etc/dhcp/dhcpd.conf
authoritative;
#$TestBed adapter
subnet 172.30.$net.0 netmask 255.255.255.0 {
  default-lease-time 600;
  max-lease-time 7200;
  range 172.30.$net.111 172.30.$net.199;
  option subnet-mask 255.255.255.0;
  option routers 172.30.$net.1;
  option domain-name-servers 8.8.8.8;
}
#$ControlRoom adapter
subnet 172.20.$net.0 netmask 255.255.255.0 {
  default-lease-time 600;
  max-lease-time 7200;
  range 172.20.$net.111 172.20.$net.199;
  option subnet-mask 255.255.255.0;
  option routers 172.20.$net.1;
  option domain-name-servers 8.8.8.8;
}
EOF

echo "new dhcp config written"

#Restart the DHCP Service to save the configuration
systemctl restart isc-dhcp-server
echo "dhcp server restarted"
echo "result from dhcp server:"
dhcp_status=$(service isc-dhcp-server status | grep -o 'Active.*')
tput setaf 3 #yellow
echo "--> ${dhcp_status}"
tput setaf 7 #white


#Create the delay.sh script
eth0=$ControlRoom
eth1=$TestBed

cat <<EOF > bwdelay.sh
#!/bin/bash
echo \${1:?"Set argument for your bandwidth in kbit/s (1500)"}
echo \${2:?"Set argument for your delay in milliseconds (500)"}
bw=\$1
delay=\$2
sudo tc qdisc repl dev $eth0 root handle 1: tbf rate \${bw}kbit burst \${bw}kbit latency 5000000
sudo tc qdisc repl dev $eth0 parent 1:1 handle 10: netem delay \${delay}ms
sudo tc qdisc repl dev $eth1 root handle 1: tbf rate \${bw}kbit burst \${bw}kbit latency 5000000
sudo tc qdisc repl dev $eth1 parent 1:1 handle 10: netem delay \${delay}ms
EOF

chmod +x bwdelay.sh

read -n 1 -s -r -p "Press any key to reboot"

reboot
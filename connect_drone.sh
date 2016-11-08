nmcli c up id ardrone2_072835
for i in {1..50}; do ping -c1 www.google.com &> /dev/null && break; done
echo "./data/wifi.sh" | telnet 192.168.1.1
nmcli c up id WASP_KTH_G2

sudo apt update
sudo apt install mosquitto mosquitto-clients
sudo systemctl start mosquitto


python3 -m virtualenv drone_env
source drone_env/bin/activate
pip3 install --upgrade pip
pip3 install paho-mqtt

desactivar el firewall de windows
netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=1883 connectaddress=172.29.106.73 connectport=1883
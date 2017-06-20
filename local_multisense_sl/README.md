# About
This ros package uses catkin tools: https://catkin-tools.readthedocs.io/en/latest/

The package is a wrapper that brings up the multisense with custom launch parameters.

# Install Pre-requisite Drivers
For Binary Installation
````
sudo apt-get install ros-indigo-multisense
````
For Source Installation or other ros versions see: http://docs.carnegierobotics.com/SL/install.html#install:indigo

# Setup Network Configuration
The network configuration is the toughest to get right.  These instructions are what worked for me after going through `http://docs.carnegierobotics.com/SL/compatibility.html#compatibility:network` 

Typically connecting the multisense to your motherboard via the ethernet port will connect to the network interface `eth0`
If not, be sure to change `eth0` to the proper name where it applies below.


## Reset MultiSense IP to factory settings
Probably the easiest way to do this is to first reset ROS Multisense Driver IP and Gateway to factory settings (10.66.171.21, 10.66.171.1)
````
# Connect the multisense to the ethernet port.
source /opt/ros/indigo/setup.bash # Source ROS
rosrun multisense_lib ChangeIpUtility -b eth0
# Power cycle the Multisense SL
````

For more useful command line utilities, see http://docs.carnegierobotics.com/SL/running.html#running:cli

# Configure Network
Turn off the network manager script: (This turned out to be not necessary if /etc/network/interfaces is set up properly. See below)
````
sudo stop network-manager
````

Run the network configuration script:
````
rosrun local_multisense_sl configureNetwork.sh
````
This is directly copied from the source code: `https://bitbucket.org/crl/multisense_ros/src`
Under `multisense_bringup`

# Retaining Internet connection (Optional)
In my case, my internet is provided via a cable. Because eth0 is now being used, you may not have internet anymore. 
I am able to retain internet connection by connecting a USB-ethernet cable where it uses `eth1`. 

My `/etc/network/interfaces` file was modified to assign a static ip to eth0:

````
# interfaces(5) file used by ifup(8) and ifdown(8)
auto lo
iface lo inet loopback

auto eth0
iface eth0 inet static
    address 10.66.171.20
````

For more hints consider looking at how Valkyrie robots get their multisense running:

https://github.com/openhumanoids/wiki/wiki/Valkyrie:-Running-the-Multisense-from-Zelda

# Launch Multisense
Check that your network is properly configured:
````
ifconfig eth0
````
Check that the IP is set to 10.66.171.20 

And we expect 16777215 from `configureNetwork.sh`:
````
cat /proc/sys/net/core/rmem_max # Should return 16777215
cat /proc/sys/net/core/wmem_max # Should return 16777215
````
Next ensure you can talk to the multisense:
````
ping 10.66.171.21 
````

Now you're ready to bringup the MultisenseSL:

````
# Source your catkin workspace then,
roslaunch local_multisense_sl multisense.launch
````
If you get an MTU error try:
````
roslaunch local_multisense_sl multisense.launch mtu:="7200"
````
My computer did not let me set my MTU to 7200 when running `configureNetwork.sh`, hence why the MTU in the launch file is defaulted to 1500.

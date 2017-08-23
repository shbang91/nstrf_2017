# Install the sound_play package
sudo apt-get install ros-indigo-sound-play


# Download and install the female voice
Instructions from: http://www.pirobot.org/blog/0022/
````
sudo apt-get install festlex-cmu
cd /usr/share/festival/voices/english/
sudo wget -c http://www.speech.cs.cmu.edu/cmu_arctic/packed/cmu_us_clb_arctic-0.95-release.tar.bz2
sudo tar jxf cmu_us_clb_arctic-0.95-release.tar.bz2 
sudo rm cmu_us_clb_arctic-0.95-release.tar.bz2
sudo ln -s cmu_us_clb_arctic cmu_us_clb_arctic_clunits
````
# Test the voices
## Run the sound_play node
````
rosrun sound_play soundplay_node.py 
````
## Run the python scripts
rosrun sound_play say.py "I am speaking with a female C M U voice" voice_cmu_us_clb_arctic_clunits

## Or use the python scripts in this directory:
python example.py
python say.py "I am speaking with a female C M U voice" voice_cmu_us_clb_arctic_clunits

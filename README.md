# ICRA BARN Navigation Challenge


## Requirements
If you run it on a local machine without containers:
* ROS version Noetic
* CMake version at least 3.0.2
* Python version at least 3.8
* Our program is compatible with ROS Noetic and can be executed directly on a local machine.

If you run it in Singularity containers:
* Go version at least 1.13
* Singularity version at least 3.6.3 and less than 4.02

The requirements above are just suggestions. If you run into any issue, please contact organizers for help (zfxu@utexas.edu).

## Installation
Follow the instructions below to run simulations on your local machines. (You can skip 1-6 if you only use Singularity container)

1. Install Conda and Create a Virtual Environment
```bash
# Download Miniconda installer for Linux (x86_64 architecture)
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh

# Run the installer (follow on-screen prompts, default settings are recommended for beginners)
bash miniconda.sh -b -p $HOME/miniconda

# Initialize Conda to make it available in the terminal
$HOME/miniconda/bin/conda init

# Create a Conda environment named "nav_challenge" with a specific Python version (e.g., 3.8)
# The -y flag auto-confirms all prompts to avoid manual input
conda create -n nav_challenge python=3.8 -y

# Activate the virtual environment (terminal prefix will show "nav_challenge" once activated)
conda activate nav_challenge
```


2. Install Python dependencies
```
pip3 install defusedxml rospkg netifaces numpy
pip3 install -r requirements.txt
```

3. Create ROS workspace
```
mkdir -p /<YOUR_HOME_DIR>/jackal_ws/src
cd /<YOUR_HOME_DIR>/jackal_ws/src
```

4. Clone this repo and required ros packages: (noetic)
```
git clone https://github.com/royalbegger/AAA.git
git clone https://github.com/jackal/jackal_simulator.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/jackal/jackal_desktop.git --branch <YOUR_ROS_VERSION>-devel
```

5. Install ROS package dependencies: (replace `<YOUR_ROS_VERSION>` with your own, noeitc)
```
cd ..
source /opt/ros/<YOUR_ROS_VERSION>/setup.bash
rosdep init; rosdep update
rosdep install -y --from-paths . --ignore-src --rosdistro=<YOUR_ROS_VERSION>
```

6. Build the workspace (if `catkin_make` fails, try changing `-std=c++11` to `-std=c++17` in `jackal_helper/CMakeLists.txt` line 3)
```
catkin_make
source devel/setup.bash
```


## Run Simulations
Navigate to the folder of this repo. 

If you run it on your local machines in world 0, and the result will be written in ``AAA/out.txt``
```
cd AAA
python3 run.py --world_idx 0 --gui
```

A successful run should print the episode status (collided/succeeded/timeout) and the time cost in second:
> \>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
>
> Navigation collided with time 27.2930 (s)

> \>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
>
> Navigation succeeded with time 29.4610 (s)


> \>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
>
>Navigation timeout with time 100.0000 (s)

## Test and compute score
In order to test our performance on the 300 static scenes in BARN, use
```
./test.sh
```

In order to test the final score, run it to calculate the average score over the 300 scenes.
```
python mean.py --file_name out.txt
```

## Acknowledgements
We would like to express our sincere gratitude to the **RRSL Lab** for open-sourcing their excellent works and providing valuable resources.

We also thank **zfxu@utexas.edu** for the patient and helpful responses to our questions and discussions throughout this project.
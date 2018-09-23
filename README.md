# Individual Submission

Name                   | Claus H. Rasmussen
---------------------- | --------------------
Udacity Account email  | chrasmussen(at)me.com

[img1]: ./imgs/final-project-ros-graph-v2.png "The System Architecture of Carla"

This project is the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

The course consisted of three terms, 3 moths each, lasting from oktober 2017 to september 2018.

Link to [Udacity's Self-driving Car Nano Degree ](https://www.udacity.com/drive).

## The project

![alt text][img1]

### System Architecture
The architecture consists of three main subsystems:
- Perception - detection of traffic lights and obstacles.
- Planning - updating the path, using waypoints with associated velocities.
- Control - actuation of throttle, steering and braking.

In this project I have developed a system, that integrates multiple components to drive a real self-driving car around a test track. The project presented here have so far only been tested on the Unity simulator, that Udacity provide to test the code.
Given the fact that my own Ubuntu 16.04 Xenial Xerus wasn't capable of providing enough computing power, the whole setup was uploaded to the 'Capstone Project Workspace', a GPU supported browser based workspace.

In short, the project consists of several ROS nodes, that is written in python. These nodes handles the load, follow and update functionality needed to provide the vehicle with waypoints ofr the track, that the vehicle should follow. Each waypoint holds a calculated value of the velocity, that the car should have at this point on the track. The car is controlled with drive-by-wire technology and calculate and publish data about throttle, steering and brake values.
A deep neural network has been trained to recognize the color of the traffic lights from images taken with the cars front camera. I used a ssd_inception_v2_coco pretrained model from [Goggle's Model Zoo](https://github.com/tensorflow/models/tree/master/research/object_detection/models). The images that I used for training and testing were kindly shared by other Udacity students, e.g. [Alex Lechner](https://github.com/alex-lechner/Traffic-Light-Classification).

### Code snippets

To install the tensorflow version 1.3 models from Google, the following code was used:
```
mkdir tensorflow
cd tensorflow
git clone https://github.com/tensorflow/models.git
cd models
git checkout edcd29f
cd research
protoc object_detection/protos/*.proto --python_out=.
export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim
python object_detection/builders/model_builder_test.py
```

To run Jupyter Notebook on Udacity's Capstone Project Workspace, in order to test the trained models, this code was used:
```
pip install jupyter
pip install matplotlib
jupyter notebook --ip=0.0.0.0 --allow-root
```

To create the .record files needed to train the models, using two different images libraries with different structures:
```
#to be run from /data

python create_tf_record.py --data_dir=tf_test_simulator/simulator_dataset_rgb/Green,tf_test_simulator/simulator_dataset_rgb/Red,tf_test_simulator/simulator_dataset_rgb/Yellow --annotations_dir=labels --output_path=tf_test_simulator/test_simulator.record --label_map_path=udacity_label_map.pbtxt
python create_tf_record_train_simulator.py --output_path tf_train_simulator/train_simulator.record
```

To train, visualize and 'freeze' the result, this code was used:
```
#to be run from CarND-Capstone

python train.py --logtostderr --train_dir=training_simulator/ --pipeline_config_path=training_simulator/ssd_inception_v2_coco_simulator.config

tensorboard --logdir='training_simulator'

python export_inference_graph.py --input_type image_tensor --pipeline_config_path=training_simulator/ssd_inception_v2_coco_simulator.config --trained_checkpoint_prefix=training_simulator/model.ckpt-20000 --output_directory=frozen_models/frozen_simulator_inception2
```

The frozen models is called in the 'light_classification/tl_classifier.py'

## The problems I had
I've experienced a lot of latency problems, which have all been solved apart from a stubborn one that allways appears halfway out on the track, just after the fifth traffic light. My theory is that it relates to issues with the Web Services used in the communication between the simulator and my ROS based python code. There are no anomalies in the data at the mentioned part of the path, apart from some distances between the provided waypoints, that are larger than the average. The distance problem has been dealt with in *waypoint_loader.py*, where extra waypoints have been added.
The loop-rates(in Hz) has been adjusted so that the simulator can be run withuot too much latency, but especially the rate for the 'twist_controller' has to be adjusted to 50 Hz, in order for the code to run properly on Carla, Udacity's autonomous car.

--------------
**The following text is from the original README from Udacity**

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

# Quadrotor path planning with RRT* and MPC 
This is the final project of the course Planning and Decision Making. The code in this github repository supports the paper that has been handed in for the project. 

- __Authors:__ M.H.J.L. Kokshoorn, P.A.R.M. Mignot, L.J.A. Peters, D.P. Siderius.
- __Date:__ 15-01-2024
- __Course code:__ RO47005
- __Department:__ Cognotive Robotics
- __Study:__ Master Roboyics 
- __University:__ TU Delft 

## Installation 

The following steps need to be followed in order to run the python script.

This project uses the [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones) repository for the drone environment. The steps to download this enviroment are shown below. 
``` 
git clone https://github.com/utiasDSL/gym-pybullet-drones.git
cd gym-pybullet-drones/

conda create -n drones python=3.10
conda activate drones

pip3 install --upgrade pip
pip3 install -e . # if needed, `sudo apt install build-essential` to install `gcc` and build `pybullet`
```
After this is done this github needs to be cloned. Go to the place where you want to do this on your computer. The cloning can be done with the following command:

``` 
git clone git@github.com:LucasPeters00/pdm_final.git
```
If everything went well now you can run the script

## Usage

If you want to see the quadrotor go through the static and dynamic columns. First you need to be in the cloned directory You can use the following command:

```
python3 main.py
```
A screen will pop up and you will see the following:

[![Watch the video](https://img.youtube.com/vi/TaFyCzv_xfk/maxresdefault.jpg)](https://youtu.be/TaFyCzv_xfk)


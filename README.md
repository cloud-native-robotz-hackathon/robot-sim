# Virtual Robots for the Robot Hackathon

This repository is hosting our attempts to use the Open Source robot simulator **Webots** (www.cyberbotics.com).

## Goal

To be able to run the robot hackathon virtually (or as a virtual/physical hybrid) we need a robot simulator with at least these abilities:

* Provide a robot 3D simulation and a model that is close to the GoPiGo3 we use in the carbon world.
* Provide the ability to control the virtual robot via a Rest API
* Ability to stream the simulation to a web browser to limit interaction of attendees with the simulation itself

Webots is providing all of this in principle but some customization is needed.

## Running Webots on Fedora (around) 33

Installation is well documented in the user guide (https://cyberbotics.com/doc/guide/installing-webots) but official packages are only provided for Ubuntu. On Fedora 32/33 Webots runs fine for me (YMMV) using the generic tarball installation method (https://cyberbotics.com/doc/guide/installation-procedure#installing-the-tarball-package).

## Webots Simulations in this repo

You'll find Webots projects in the *simulations* folder.

**hackathon_simulator** is going to be the production environment

**gopigo** is an *attempt* to build a custom robot modelling the GoPiGo3 robot in Webots.

## Testing the Hackathon Simulator

To test the feasibility of developing a Webots controller that provides a Rest API to control a Webots robot from outside of Webots I've written a Python controller called *epuck_remote_tornado.py* using the Python Tornado library. It's more like a PoC but it works to control the epuck robot model. You can find the code in the *hackathon_simulator* project folder inside the *controller* folder.

### Requirements

* Working Webots installation of course
* The Python Tornado library: `$ sudo yum install python3-tornado -y`

### Run it

* Clone this repository
* Start Webots
* Open the world: *File->Open World->simulations/hackathon_simulator/worlds/hackathon_simulator.wbt*
* Start the simulation

This will start the controller and will open the local port 3000/TCP. You can now use curl or whatever locally to control the robot. Try:

```
$ curl http://localhost:3000/distance
$ curl http://localhost:3000/forward
$ curl http://localhost:3000/90
$ curl http://localhost:3000/stop
```

And you should see the robot moving around in Webots.

`$ curl http://localhost:3000/reset` will reset the simulation but keep it running.

Look at the handler section in the controller code to see what API endpoints work already.

Stop the simulation using the "PAUSE" button and reset when you want to start over using the "REWIND" button. And read the user guide, there is tons of information and tutorials there.
# Boids

Please see this accompanying article to learn more about this algorithm and the code.
https://medium.com/better-programming/boids-simulating-birds-flock-behavior-in-python-9fff99375118

Note that the PySimpleGUI code will not be covered in the article as it's a new addition.

## What is Boids?
[Boids](https://en.wikipedia.org/wiki/Boids) is an artificial life program developed by Craig Reynolds which simultes the flocking behaviour of birds.As with most artificial life simulations, Boids is an example of emergent behavior; that is, the complexity of Boids arises from the interaction of individual agents (the boids, in this case) adhering to a set of simple rules. The rules applied in the simplest Boids world are as follows:

* separation: steer to avoid crowding local flockmates
* alignment: steer towards the average heading of local flockmates
* cohesion: steer to move towards the average position (center of mass) of local flockmates

## PySimpleGUI - the GUI engine
This fork of boid is a port to the PySimpleGUI framework.  It is currently running on the tkinter and the browser ports.  It alsmot ran on the Qt port (very close)

## About this repository (boids-PySimpleGUI)
This repository is a fork of the Boids project located here:
https://github.com/roholazandie/boids

The change to the code was removal of P5 as the graphic rendering engine and replaced with PySimpleGUI and PySimpleGUIWeb.  With the addition of the GUI came a slider that enables you to adjust, in real time, the number of birds.

## GUI Platforms - Desktop and Browser

By the magic of PySimpleGUI this code is running both on the desktop in the form of a tkinter window but also it runs in the browser.  The only required change to the program was the import statement.  That's ALL you have to change to move from the desktop to the browser.  Amazing.

### Tkinter version
`import PySimpleGUI as sg`

![Boids - Tk](https://user-images.githubusercontent.com/13696193/59565880-8ec5de80-9026-11e9-9832-ab7b05fd7b5c.gif)

### Browser based
`import PySimpleGUIWeb as sg`

![Boids - Web](https://user-images.githubusercontent.com/13696193/59565883-9a190a00-9026-11e9-80c9-1822b2cda6af.gif)


## Installation

 Run:
 ```
 sudo apt-get install libglfw3
 ```
 ```
pip install -r requirements.txt
```
For using fast boid:  (Not yet ported)
```
pip install ray
```

## A technical detail
The implemented boid is slow for large number of boids. The fast_boid is a try usign [ray](https://github.com/ray-project/ray) to make it faster. In current implementation, it doesn't make a huge difference though. If someone could figure out to make it faster just send me a push request.

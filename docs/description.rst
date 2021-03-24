SoMo is a light wrapper around pybullet that facilitates
the simulation of continuum manipulators.

SoMo (**So**\ ft\ **Mo**\ tion) is a framework to facilitate the
simulation of continuum manipulator (CM) motion in `pybullet`_. In SoMo,
continuum manipulators are approximated as a series of rigid links
connected by spring-loaded joints. SoMo makes it easy to create URDFs of
such approximated manipulators and load them into pybullet’s rigid body
simulator. With SoMo, environments with various continuum manipulators,
such as hands with soft fingers (xxx links), or snakes, can be created
and controlled with only a few lines of code.



Installation
------------

Requirements
~~~~~~~~~~~~

-  `Python 3.6`_\ +
-  Tested on:

   -  Ubuntu 16.04 and Ubuntu 18.04 with Python 3.6.9
   -  Ubuntu 20.04 with Python 3.6.9, 3.7.9 and 3.8.2
   -  Windows 10 with Python 3.7 through `Anaconda`_

-  Recommended: pip (``sudo apt-get install python3-pip``)
-  Recommended (for Ubuntu): `venv`_
   (``sudo apt-get install python3-venv``)

Setup
~~~~~

0. Make sure your system meets the requirements
1. Clone this repository
2. Set up a dedicated virtual environment using ``venv``
3. Activate virtual environment
4. Install requirements from this repo:
   ``$ pip install -r requirements.txt``
5. Install this module:

   -  either by cloning this repo to your machine and using
      ``$  pip install -e .`` from the repo root, or with
   -  ``$ pip install git+https://github.com/graulem/somo``

6. To upgrade to the newest version:
   ``$ pip install git+https://github.com/graulem/somo --upgrade``

Explore the examples
~~~~~~~~~~~~~~~~~~~~

-  run any of the files in the examples folder. xx is a great place to
   start xx should not be part of the installation

Contributing
~~~~~~~~~~~~

-  only through a new branch and reviewed PR (no pushes to master!)
-  always use `Black`_ for code formatting
-  always bump the version of your branch by increasing the version
   number listed in somo/_version.py

Using this framework
--------------------

This framework relies on two key components: a collection of methods
that facilitates the automated generation of urdf files (xx); and a
class that relies heavily on pybullet to define a CMClass object (xx
rephrase). A CMClass object can be instantiated from a json file (xx not
implemented yet) that specifies the properties of the continuum
manipulator in human-readable form, enabling the user to easily keep
track of and vary the properties of different CMs through a series of
experiments.

.. raw:: html

   <!--
   the continuum manipulator class, which relies heavily on pybullet, defines a CM object and provides an intuitive interface to 
   -->

Citation
--------
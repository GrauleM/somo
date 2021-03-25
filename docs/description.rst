SoMo is a light wrapper around pybullet that facilitates
the simulation of continuum manipulators.

SoMo (**So**\ ft\ **Mo**\ tion) is a framework to facilitate the
simulation of continuum manipulator motion in `PyBullet physics engine <https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet>`_. In SoMo,
continuum manipulators are approximated as a series of rigid links
connected by spring-loaded joints. SoMo makes it easy to create URDFs of
such approximated manipulators and load them into pybullet’s rigid body
simulator. With SoMo, environments with various continuum manipulators
(such as hands with soft fingers or snakes) can be created
and controlled with only a few lines of code.



.. image:: img/importance_of_palms.png



Quick Install
=============

.. note::
   Coming soon: pip install!




Explore the examples
====================

Run any of the files in the examples folder. "examples/basic" is a great place to start!




Links
=====

**Documentation:** `Read the Docs <https://somo.readthedocs.io/en/latest/>`_

**pip install:** `View on PyPi <https://pypi.org/project/somo/>`_ (*Not Launched Yet*)

**Source code:** `Github <https://github.com/graulem/somo>`_


Contact
=======

If you have questions, or if you've done something interesting with this package, get in touch with `Moritz Graule <mailto:graulem@g.harvard.edu>`_!

If you find a problem or want something added to the library, `open an issue on Github <https://github.com/graulem/somo/issues>`_.



Citation
=========

.. code-block:: console
  
   @inproceedings{graule2020somo,
      title={SoMo: Fast and Accurate Simulations of Continuum Robots in Complex Environments},
      author={Graule, Moritz A. and Teeple, Clark B and McCarthy, Thomas P and St. Louis, Randall C and Kim, Grace R and Wood, Robert J},
      booktitle={2021 IEEE International Conference on Intelligent Robots and Systems (IROS)},
      pages={In Review},
      year={2021},
      organization={IEEE}
   }

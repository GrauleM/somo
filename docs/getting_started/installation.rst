.. _installation:

=============
Installation
=============

Requirements
~~~~~~~~~~~~

-  `Python 3.6 <https://www.python.org/downloads/release/python-360/>`_ +
-  Tested on:

   -  Ubuntu 16.04 and Ubuntu 18.04 with Python 3.6.9
   -  Ubuntu 20.04 with Python 3.6.9, 3.7.9 and 3.8.2
   -  Windows 10 with Python 3.7 and 3.8 through `Anaconda <https://www.anaconda.com/products/individual#Downloads>`_

-  Recommended: pip (``sudo apt-get install python3-pip``)
-  Recommended (for Ubuntu): `venv <https://docs.python.org/3/library/venv.html>`_
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
      ``pip install -e .`` from the repo root, or with
   -  ``pip install git+https://github.com/graulem/somo``
   
6. To upgrade to the newest version:
   ``$ pip install git+https://github.com/graulem/somo --upgrade``
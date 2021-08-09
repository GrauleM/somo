from setuptools import setup

# a dummy setup.py used to reserve the name somo on pypi following
# https://stackoverflow.com/questions/47676721/register-an-internal-package-on-pypi

setup(
    name="somo",
    version=open("somo/_version.py").readlines()[-1].split()[-1].strip("\"'"),
    description="A light framework for the simulation of continuum manipulators",
    long_description="",
    url="git@github.com:GrauleM/somo",
    author="Moritz A. Graule",
    author_email="moritz@graule.ch",
    license="unlicense",
    # remember to add all additional submodules to this list
    packages=["somo", "somo.sweep", "somo.logger"],
    classifiers=["Development Status :: 1 - Planning"],
)

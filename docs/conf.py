# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
import codecs

sys.path.insert(
    0, os.path.abspath("..")
)  # Add the base path to to add things to autodoc


# -- Project information -----------------------------------------------------

project = "SoMo"
copyright = "2021, Moritz A. Graule, Harvard Microrobotics Lab"
author = "Moritz A. Graule, Harvard Microrobotics Lab"


# -- General configuration ---------------------------------------------------

# Set name of Master Doc
master_doc = "index"

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [  #'recommonmark',
    "sphinx_markdown_tables",
    "sphinx_copybutton",
    "sphinx.ext.autodoc",
    "sphinx.ext.coverage",
    "sphinx.ext.napoleon",
    "sphinx.ext.graphviz",
    "sphinx.ext.mathjax",
    "sphinxcontrib.bibtex",
    "sphinx_panels",
    "m2r2",
]


bibtex_bibfiles = ["refs.bib"]
bibtex_default_style = "bibtexlabels"
bibtex_reference_style = "author_year"


import pybtex.plugin
from pybtex.style.formatting.unsrt import Style as UnsrtStyle
from pybtex.style.labels import BaseLabelStyle
from pybtex.plugin import register_plugin


class MyLabelStyle(BaseLabelStyle):
    def format_labels(self, sorted_entries):
        for entry in sorted_entries:
            yield entry.key


class MyStyle(UnsrtStyle):
    default_label_style = MyLabelStyle


register_plugin("pybtex.style.formatting", "bibtexlabels", MyStyle)


source_suffix = [".rst", ".md"]

autodoc_mock_imports = ["pandas", "seaborn", "pathos", "yaml"]

# Read in all required packages and add them to a list
import requirements

with open("../requirements.txt", "r") as fd:
    for req in requirements.parse(fd):
        autodoc_mock_imports.append(req.name)

autodoc_mock_imports = autodoc_mock_imports


# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"

html_logo = "img/logo.png"
html_theme_options = {
    "logo_only": True,
    "collapse_navigation": False,
    "sticky_navigation": True,
    "style_nav_header_background": "#f0f0f0",
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]

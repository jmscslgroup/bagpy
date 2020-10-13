Installation
------------

bagpy can be used with Python 2.7 and 3.6 or later. We do not support ``bagpy`` on Windows. However, a user can use ``bagpy`` through Google Colab.

PyPI
^^^^

Install bagpy from PyPI_ using::

    pip install -U bagpy

``-U`` is short for ``--upgrade``.
If you get a ``Permission denied`` error, use ``pip install -U bagpy --user`` instead.


Development Version
^^^^^^^^^^^^^^^^^^^

To work with the latest development version, install from GitHub_ using::

    pip install git+https://github.com/jmscslgroup/bagpy

or::

    git clone https://github.com/jmscslgroup/bagpy
    pip install -e bagpy

``-e`` is short for ``--editable`` and links the package to the original cloned
location such that pulled changes are also reflected in the environment.


Dependencies
^^^^^^^^^^^^

- `numpy <https://docs.scipy.org/>`_, `scipy <https://docs.scipy.org/>`_, `pandas <https://pandas.pydata.org/>`_, `scikit-learn <https://scikit-learn.org/>`_, `matplotlib <https://matplotlib.org/>`_.


Software Requirements
^^^^^^^^^^^^^^^^^^^^^^

- Ubuntu 18.04 (not tested on any other version of Ubuntu, but might work)
- Python 3.6 or later, or Python 2.7 


Creating vritual environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
I recommending creating a python virtual environment for installing ``bagpy``.
You can use Anaconda to create virtual environment. Steps for doing so can be found elsewhere on internet.
Alternatively, you can also use ``virtualenv`` python package to do so.
    



Jupyter Notebook
^^^^^^^^^^^^^^^^

To run the tutorials in a notebook locally, please install::

   conda install notebook

and run ``jupyter notebook`` in the terminal. If you get the error ``Not a directory: 'xdg-settings'``,
use ``jupyter notebook --no-browser`` instead and open the url manually (or use this
`bugfix <https://github.com/jupyter/notebook/issues/3746#issuecomment-444957821>`_).


If you run into issues, do not hesitate to approach us or raise a `GitHub issue`_.

.. _Anaconda: https://docs.anaconda.com/anaconda/install/
.. _PyPI: https://pypi.org/project/bagpy
.. _Github: https://github.com/jmscslgroup/bagpy
.. _`Github issue`: https://github.com/jmscslgroup/bagpy/issues/new/choose
.. _comma.ai: https://comma.ai/

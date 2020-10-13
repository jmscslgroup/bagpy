import sys
import os
import inspect
import logging
from pathlib import Path
from datetime import datetime
from typing import Optional, Union, Mapping

from sphinx.application import Sphinx
from sphinx.ext import autosummary

# remove PyCharm’s old six module
if "six" in sys.modules:
    print(*sys.path, sep="\n")
    for pypath in list(sys.path):
        if any(p in pypath for p in ["PyCharm", "pycharm"]) and "helpers" in pypath:
            sys.path.remove(pypath)
    del sys.modules["six"]

import matplotlib 

matplotlib.use("agg")

HERE = Path(__file__).parent
sys.path.insert(0, f"{HERE.parent.parent}")
sys.path.insert(0, os.path.abspath("_ext"))
import strym

logger = logging.getLogger(__name__)


# -- Retrieve notebooks ------------------------------------------------

from urllib.request import urlretrieve

notebooks_url = "https://raw.githubusercontent.com/jmscslgroup/bagpy/master/notebook/"
notebooks = [
    "bagpy_example.ipynb",
    "Reading_bagfiles_from_cloud.ipynb"
]
for nb in notebooks:
    try:
        urlretrieve(notebooks_url + nb, nb)
    except:
        pass


# -- General configuration ------------------------------------------------

needs_sphinx = "1.7"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.doctest",
    "sphinx.ext.coverage",
    "sphinx.ext.mathjax",
    "sphinx.ext.autosummary",
    "sphinx.ext.napoleon",
    "sphinx.ext.intersphinx",
    "sphinx.ext.githubpages",
    "sphinx_autodoc_typehints",
    "nbsphinx",
    "edit_on_github",
    "ytsphinx.youtube",
    "m2r2"
]


# Generate the API documentation when building
autosummary_generate = True
napoleon_google_docstring = False
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = False
napoleon_use_rtype = False
napoleon_custom_sections = [("Params", "Parameters")]

intersphinx_mapping = dict(
    python=("https://docs.python.org/3", None)
)

templates_path = ["_templates"]
source_suffix = [".rst", ".ipynb"]
master_doc = "index"

# General information about the project.
project = "bagpy"
author = "Rahul Bhadani"
title = "bagpy: Reading rosbag files simplified"
copyright = f"{datetime.now():%Y}, {author}"

v = Path("../../bagpy/version").open(encoding = "utf-8").read().splitlines()
version = v[0].strip()

# The full version, including alpha/beta/rc tags.
release = version

exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]
pygments_style = "sphinx"
todo_include_todos = False

# Add notebooks prolog to Google Colab and nbviewer
nbsphinx_prolog = r"""
{% set docname = 'github/jmscslgroup/bagpy/blob/master/notebook/' + env.doc2path(env.docname, base=None) %}
.. raw:: html

    <div class="note">
      <a href="https://colab.research.google.com/{{ docname|e }}" target="_parent">
      <img src="https://colab.research.google.com/assets/colab-badge.svg" alt="Open In Colab"/></a>
      <a href="https://nbviewer.jupyter.org/{{ docname|e }}" target="_parent">
      <img src="_static/nbviewer-badge.svg" alt="Open In nbviewer"/></a>
    </div>
"""

# -- Options for HTML output ----------------------------------------------

html_theme = "sphinx_rtd_theme"
html_theme_options = dict(navigation_depth=1, titles_only=True)
github_repo = "bagpy"
github_nb_repo = "bagpy"
html_static_path = ["_static"]

html_logo = "favicon.png"

html_favicon = "favicon.ico"

def setup(app):
    app.add_css_file("custom.css")


# -- Options for other output ------------------------------------------

htmlhelp_basename = "bagpydoc"
title_doc = f"{project} documentation"

latex_documents = [(master_doc, f"{project}.tex", title_doc, author, "manual")]
man_pages = [(master_doc, project, title_doc, [author], 1)]
texinfo_documents = [
    (master_doc, project, title_doc, author, project, title, "Miscellaneous")
]


# -- GitHub URLs for class and method pages ------------------------------------------


def get_obj_module(qualname):
    """Get a module/class/attribute and its original module by qualname"""
    modname = qualname
    classname = None
    attrname = None
    while modname not in sys.modules:
        attrname = classname
        modname, classname = modname.rsplit(".", 1)

    # retrieve object and find original module name
    if classname:
        cls = getattr(sys.modules[modname], classname)
        modname = cls.__module__
        obj = getattr(cls, attrname) if attrname else cls
    else:
        obj = None

    return obj, sys.modules[modname]


def get_linenos(obj):
    """Get an object’s line numbers"""
    try:
        lines, start = inspect.getsourcelines(obj)
    except TypeError:
        return None, None
    else:
        return start, start + len(lines) - 1


# set project_dir: project/docs/source/conf.py/../../.. → project/
project_dir = Path(__file__).parent.parent.parent
github_url_strym = "https://github.com/jmscslgroup/bagpy/tree/master"
from pathlib import PurePosixPath


def modurl(qualname):
    """Get the full GitHub URL for some object’s qualname."""
    obj, module = get_obj_module(qualname)
    github_url = github_url_strym
    path = PurePosixPath(Path(module.__file__).resolve().relative_to(project_dir))
    start, end = get_linenos(obj)
    fragment = f"#L{start}-L{end}" if start and end else ""
    return f"{github_url}/{path}{fragment}"


def api_image(qualname: str) -> Optional[str]:
    path = Path(__file__).parent / f"{qualname}.png"
    print(path, path.is_file())
    return (
        f".. image:: {path.name}\n   :width: 200\n   :align: right"
        if path.is_file()
        else ""
    )


# modify the default filters
from jinja2.defaults import DEFAULT_FILTERS

DEFAULT_FILTERS.update(modurl=modurl, api_image=api_image)

# -- Override some classnames in autodoc --------------------------------------------

import sphinx_autodoc_typehints



# -- Prettier Param docs --------------------------------------------

from typing import Dict, List, Tuple
from docutils import nodes
from sphinx import addnodes
from sphinx.domains.python import PyTypedField, PyObject
from sphinx.environment import BuildEnvironment


class PrettyTypedField(PyTypedField):
    list_type = nodes.definition_list

    def make_field(
        self,
        types: Dict[str, List[nodes.Node]],
        domain: str,
        items: Tuple[str, List[nodes.inline]],
        env: BuildEnvironment = None,
    ) -> nodes.field:
        def makerefs(rolename, name, node):
            return self.make_xrefs(rolename, domain, name, node, env=env)

        def handle_item(
            fieldarg: str, content: List[nodes.inline]
        ) -> nodes.definition_list_item:
            head = nodes.term()
            head += makerefs(self.rolename, fieldarg, addnodes.literal_strong)
            fieldtype = types.pop(fieldarg, None)
            if fieldtype is not None:
                head += nodes.Text(" : ")
                if len(fieldtype) == 1 and isinstance(fieldtype[0], nodes.Text):
                    (text_node,) = fieldtype  # type: nodes.Text
                    head += makerefs(
                        self.typerolename, text_node.astext(), addnodes.literal_emphasis
                    )
                else:
                    head += fieldtype

            body_content = nodes.paragraph("", "", *content)
            body = nodes.definition("", body_content)

            return nodes.definition_list_item("", head, body)

        fieldname = nodes.field_name("", self.label)
        if len(items) == 1 and self.can_collapse:
            fieldarg, content = items[0]
            bodynode = handle_item(fieldarg, content)
        else:
            bodynode = self.list_type()
            for fieldarg, content in items:
                bodynode += handle_item(fieldarg, content)
        fieldbody = nodes.field_body("", bodynode)
        return nodes.field("", fieldname, fieldbody)


# replace matching field types with ours
PyObject.doc_field_types = [
    PrettyTypedField(
        ft.name,
        names=ft.names,
        typenames=ft.typenames,
        label=ft.label,
        rolename=ft.rolename,
        typerolename=ft.typerolename,
        can_collapse=ft.can_collapse,
    )
    if isinstance(ft, PyTypedField)
    else ft
    for ft in PyObject.doc_field_types
]

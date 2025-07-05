import os
import sys
sys.path.insert(0, os.path.abspath('../sw'))  # Adjust path if needed

# -- Project information -----------------------------------------------------
project = 'py_pidx'
copyright = '2025, Mehrab Mahdian'
author = 'Mehrab Mahdian'

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.autosectionlabel'
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------
html_theme = "alabaster"

html_theme_options = {
    'description': 'Advanced PID controller library for Python',
    'github_user': 'mehrabmahdian',
    'github_repo': 'py_pidx',
    'fixed_sidebar': True,
}

html_context = {
    "display_github": True,            # Show GitHub link
    "github_user": "mehrabmahdian",
    "github_repo": "py_pidx",
    "github_version": "main",
    "conf_py_path": "/docs/",
}

# Static files (CSS, JS, images)
# html_static_path = ['_static']

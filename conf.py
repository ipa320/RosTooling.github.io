# Sphinx configuration for the RosTooling documentation.
# Build locally with:  sphinx-build -W --keep-going -b html . _build/html

project = "RosTooling"
author = "Fraunhofer IPA"
copyright = "Fraunhofer IPA"

extensions = [
    "myst_parser",         # pages stay Markdown
    "sphinx_design",       # tabs, e.g. VS Code and Eclipse steps side by side
    "sphinx_reredirects",  # keeps the URLs of moved pages working
]

source_suffix = {".md": "markdown"}
root_doc = "index"

# Not part of the site: tooling and archived build output.
exclude_patterns = ["_build", ".github", "tools", ".venv", "venv", "README.md", "Thumbs.db", ".DS_Store"]

# MyST: anchors for headings up to level 3, so links such as page.md#section work.
myst_heading_anchors = 3
myst_enable_extensions = ["colon_fence", "deflist"]

# The inherited pages skip heading levels (for example H1 straight to H3). They are
# rewritten or updated in the later phases of the documentation update; until then
# this one warning is silenced so that every other warning still fails the build.
suppress_warnings = ["myst.header"]

html_theme = "sphinx_rtd_theme"
html_title = "RosTooling documentation"
html_theme_options = {
    "navigation_depth": 3,
    "collapse_navigation": False,
    "sticky_navigation": True,
    "titles_only": False,
}

# Old URL -> new URL, relative to the site root. A page that moved to the archive
# keeps its address by redirecting to the page that replaced it.
redirects = {
    "docu/CreateYourModel": "LearnRosModels.html",
    "docu/NewRosModel": "StaticCodeAnalyis.html",
    "docu/IntrospectionNode": "ros2model.html",
}

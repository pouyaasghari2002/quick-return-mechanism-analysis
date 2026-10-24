# Building the English report

Overleaf: New Project → Upload Project, upload the package ZIP, select `report/main.tex` as the main document, and choose pdfLaTeX. Include the enclosing folder in the path if Overleaf retains it.

For a smaller Overleaf project, retain `report/main.tex`, `report/figures/`, and the sibling `matlab/report_listing.m` with the same relative paths. Native CAD, simulation models, workspace, and videos are not needed for typesetting.

Local TeX Live or MiKTeX: from the repository root, change to `report/` and run `pdflatex -interaction=nonstopmode -halt-on-error main.tex` twice. The second pass resolves the contents and bookmarks.

The report uses standard packages, including amsmath, graphicx, adjustbox, listings, and Latin Modern fonts. No custom fonts, bibliography file, or simulation software are required. Compilation displays source code; it does not execute it.

# Requirements and reproduction

## MATLAB

1. Open a working copy of `matlab/QuickReturnAnalysis.mlx`.
2. Confirm Symbolic Math Toolbox availability and compatibility with the script's local-function placement.
3. Review the model inputs: 200 N applied load, 1 rad/s input speed, zero input angular acceleration, and 2000 samples over two revolutions.
4. Inspect the link-5 angular-velocity discrepancy and relevant equations before drawing conclusions from a new run.
5. Keep new results separate from the archived workspace and report figures.

`quick_return_extracted.m` retains executable Live Script blocks with prose headings turned into comments. It does not reproduce stored Live Editor output or embedded images. `report_listing.m` is extracted from the separate Word appendix; those two sources may differ. Neither is presented as corrected or execution-tested.

Inspection findings to check include the second-loop velocity RHS parentheses, the first-loop acceleration's radial term, use of aG4 rather than aG5 in member-5 dynamic equations, the hardcoded force-plot index, and the initialization/expansion of the unknowns matrix. The original files have not been modified to fix these issues.

The saved workspace is named `Workspcae.mat` in the archive; the spelling is retained. Loading a workspace is not a substitute for rerunning and validating the calculations.

## Adams/View

Extract `adams/native_models.zip` into a working folder. It contains the original `QuickReturn.bin` and `CAM.bin`, byte for byte. Open them with a compatible Adams/View installation. A recording is labeled Adams 2020, but native application compatibility was not verified.

Check initial crank angle, prescribed motion units, gravity, mass/inertia units, joint definitions, force directions, and cam constraints before comparing plots. The source describes MATLAB starting at zero degrees and Adams at 30 degrees. Correct phase and time conventions before assessing agreement.

The Excel cam worksheet and imported coordinate CSV are not separate archive files. Screenshots of them are preserved in the report; no replacement data have been invented.

## SolidWorks and SAM

Keep the complete `solidworks/` folder together. Open `Assem.SLDASM` with its referenced part files available. Inspect the feature tree and mates before rebuilding or changing geometry. Native rebuilds were not performed during this preparation.

Open `sam/QuickReturn.sam` in a compatible SAM application. Its presence in the archive does not establish a completed quantitative comparison; the detailed report comparisons focus on MATLAB and Adams.

## Video files

The included Adams recordings and SolidWorks animation remain unchanged. The original approximately 62.4 MiB MATLAB recording exceeds the 25 MiB GitHub browser-upload limit and remains in the original RAR archive. It is excluded from this package. It is not needed to compile the report.

## Additional code

Place any separately stored original MATLAB helpers or other analysis scripts in the relevant folder. Document their purpose, version, dependencies, exact run instructions, and input/output paths. Do not claim missing code is available or tested until it has been supplied and checked.

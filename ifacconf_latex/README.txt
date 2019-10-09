INSTALLING THE IFACCONF LATEX PACKAGE
=====================================

1. Unzip the ifacconf_latex.zip archive into a folder (directory) of your convenience.

2. Copy the ifaconf.cls file to a directory where latex can find it (see your local installation documentation for the details). 

3. Test the class file by processing the sample file twice:

pdflatex ifaconf.tex   
pdflatex ifaconf.tex  # run twice to solve cross-references

You should obtain as a result a file named ifaconf.pdf. It should look the same as the provided ifaconf_sample.pdf. This file also contains some guidelines about using the ifaconf.cls LaTeX class.

Alternatively, you can generate a PDF file using latex and ghostscript:

latex ifacconf.tex
latex ifacconf.tex    # run twice to solve cross-references
dvips -Ppdf -G0 -ta4 ifacconf
ps2pdf -dCompatibilityLevel=1.4 -dMaxSubsetPct=100 -dSubsetFonts=true \
       -dEmbedAllFonts=true -sPAPERSIZE=a4 ifacconf.ps ifaconf.pdf

4. In particular, check the ifacconf.pdf file you have produced for:

- Page size: A4 paper size is required for all IFAC event papers.
- Font usage: you should not have any type 3 fonts nor Asian fonts. All the fonts must be embedded in the PDF file.

The simplest way to check the PDF file is to use Acrobat Reader (freely downloadable from http://www.adobe.com) and open the File->Properties Window.

If the file size or fonts are incorrect, check your LaTeX configuration for  appropriate settings. 
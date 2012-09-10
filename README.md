vimjay
======

A graph based image processing and generation tool.

Nodes are processing blocks with inputs and outputs.  Currently inputs are not distinguished from outputs and all can be either, though in some nodes certain ios will be overwritten every update.

The current model is to only update nodes that are input ancestors to an active output, and that have dirty inputs.

Originally hosted on http://code.google.com/p/binarymillenium/source/browse/trunk/opencv/camthing on svn.

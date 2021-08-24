# Example rc-fmu-ap configurations.

Note, this is of course a work in progress, so the contents of these
configuration files are expected to evolve over time as the feature
set of rc-fmu-ap expands and matures.

    { "include": "file.json" }

It is important to note the additional feature on top of pure json
that enables a top level json file to include sub json files in
specific locations of the tree (a bit like a C preprocessor #include.)
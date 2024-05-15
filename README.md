### Running the Project

If the project is being run in the CLion terminal, make sure the option "Emulate terminal in output console" is enabled
in the Run/Debug configurations. This is necessary for capturing inputs for our program.

### Using custom Datasets

Custom Datasets can be used in the project. To use them follow these instructions:
1. Put the files in a folder named "graph/Custom/" in the main project directory according to the following rules:
    - The **Nodes** file should be named "nodes.csv"
    - The **Edges** file should be named "edges.csv"
    - The overall structure of the CSV files should be the same as the ones in the fully connected or real world datasets
2. When starting the program, select the option ```[4]``` in the menu to load a custom dataset

The structure should be like:
```
$ tree graph

  graph
    ├── Custom
    │      ├── edges.csv
    │      └── nodes.csv
    ├── Extra Fully Connected
    ├── Real World
    └── Toy
```
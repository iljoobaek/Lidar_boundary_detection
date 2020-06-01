

# Boundary detection virtualscan

The overall flow of the project is 

1. Use Vscan to detect points of interest
2. Further process the vscan results to obtain clusters(DBScan)
3. Divide the space into priors and run boundary detection
4. Merge the boundaries from the different priors where needed

Part 1 is done using the c++ implementation of vscan(main code is in main.cpp). Part 2,3 and 4 are done in python. 

The python code uses the vscan data prepared in the form of frame-wise csv files.(for example in this folder `2011_09_26_drive_0005_sync_vscan`)

We use the main.cpp file to generate these csv files. We need to redirect the stdout to a file(using the `>` operator). The log(stdout) of main.cpp will generate semicolon separated values for each vscan point in a frame(with a few debug statements between each frame). To convert the data to We need to manually remove a the debug lines from this log. The debug lines start with "Frame ". Once we remove this, every frame has semicolon separated coordinates. Each frame is separated by a comma. We use `log_to_csv.ipynb` to convert this formatted log file to frame wise csv files for the python code to process the data.

All the python code is in the `boundary_detector_python` folder. The jupyter notebooks were used for debugging and visualization for developing the algorithm. 

## How to run

1. install all the requirements from the requirements.txt file
    - `pip install -r requirements.txt`
2. run the `boundary_final_code.py` python script from the boundary_detector_python folder
    - `cd boundary_detector_python`
    - `python boundary_final_code.py`

## Algorithm for boundary detection

1. We divide the vscan data into 6 priors( also have an option of usign 2 priors)
2. For each prior, we add two helper points which will help us generate a convex hull such that it includes the boundary we need.
    - The way we hose the helper points is as follows
    - For a prior on the upper half, we use the upper 2 corners of the prior rectangle and for a prior in the lower half, we use the lower 2 corners of the prior rectangle
    - Once we have a convex hull using these helper points, we take the convex hull and remove the helper points. The remaining points form a line that will be our boundary.
    - We use the functions from scipy to generate the convex hull
3. The priors we use look like this

```

X1 = first limit
X2 = second limit
W = Max X 
0]> = Position of the Car

Priors 
    0    X1    X2     W
    -------------------
    |    |     |      |
    | 1  |  2  |  3   |
    |    |     |      |
    0]>--+-----+------+
    |    |     |      |
    | 4  |  5  |  6   |
    |    |     |      |
    -------------------
```

## Notes for the cpp code

### Note
The current implementation of recognizing whether the data is from kitti or not simply see if the root path
has "kitti" included as below.
```cpp
root_path.find("kitti") == std::string::npos ? false : true;
```
So make sure the root path is something like "/home/data_set/kitti_data/...".

## Run the program from command line 
```
./run.sh
```

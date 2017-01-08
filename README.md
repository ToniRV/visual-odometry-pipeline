<img src="resources/vo_pipeline.png" align="right" width="30%"/>
# README for VO pipeline

Monocular, visual odometry (VO) pipeline with the most essential features:

1. initialization of 3D landmarks
2. keypoint tracking between two frames
3. pose estimation using established 2D↔3D correspondences
4. triangulation of new landmarks.

## Installation
1. To install this source code, first clone the repo:
```
git clone git@github.com:ToniRV/visual-odometry-pipeline.git
```

2. Modify the paths to the datasets that you can download from [here](http://rpg.ifi.uzh.ch/docs/teaching/2016/kitti00.zip)
Copy this [file](visual-odometry-pipeline/src/Parameters/paths_example.txt) and rename it to paths.txt
Modify your paths accordingly.
Then store it in the Parameters folder.

3. Now you can head to matlab and open main. You can now press Run.


## Grading
1. **Qualitative evaluation** of VO pipeline given by the following points (sorted by decreasing order of importance):

  1. **Implementation**:
    * A 6 can only be achieved with a completely monocular pipeline.  
    * If your pipeline requires stereo frames for initialization only, your grade will be ≤ 5.
    * If your pipeline requires stereo frames in continuous operation, your grade will be ≤ 4.5.
  2. **Performance**: How far does the pipeline go without failing or deviating significantly from the ground truth.(Note that with the KITTI and Malaga sequences, scale drift will be difficult to avoid, we will take that into account when judging your VO pipelines).
  3. **Speed**: How fast does your code run (in particular for Matlab code, we will check whether you paid attention to using vectorized operations instead of for loops whenever possible).
  
2. The **report** that you will have to hand in:

  * Summarize the work that you did and specify _exactly_ what your VO pipeline does:
    * i.e. whether it is monocular or stereo
    * How well it performs on the provided datasets (**with plots**). 
    * Describe bonus features that you implemented, or the additional work you did that over the basic VO pipeline required, and how this impacts the quality of your VO pipeline.</br>
       Additional features which degrades the quality of your VO pipeline will be accepted and valued as long as:
       * the implemented feature is properly motivated and described
       * an analysis showing the effect of your additional feature is provided in the report.
  * A video of your working pipeline.

3. **Other Features**: Whether you implemented one or multiple bonus features, or did something in addition to the basic requirements (and explained it in the report). Note that no penalty is given if a bonus feature requires stereo frames. A (non-exhaustive) list of ideas is given down here:

## Reference output
https://www.youtube.com/watch?v=MQ1iDi3S-TU&list=PLI5XgAFFHqZiAg6yd0XKOTRgkCE4QTm54&index=3#t=32.749926

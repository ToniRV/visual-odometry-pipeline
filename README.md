<img src="vo_pipeline.png" align="right" width="30%"/>
# README for VO pipeline

Monocular, visual odometry (VO) pipeline with the most essential features:

1. initialization of 3D landmarks
2. keypoint tracking between two frames
3. pose estimation using established 2D↔3D correspondences
4. triangulation of new landmarks.

## Grading
1. The quality of the pose estimation: not a strict quantitative evaluation. Instead, we will pay a particular attention to the following points (sorted by decreasing order of importance):
  1. The features you implemented. 
    * A 6 can only be achieved with a completely monocular pipeline.  
    * If your pipeline requires stereo frames for initialization only, your grade will be ≤ 5.
    * If your pipeline requires stereo frames in continuous operation, your grade will be ≤ 4.5.
  2. How far does the pipeline go without failing or deviating significantly from the ground truth. Note that with the KITTI and Malaga sequences, scale drift will be difficult to avoid, we will take that into account when judging your VO pipelines).
  3. How fast does your code run (in particular for Matlab code, we will check whether you paid attention to using vectorized operations instead of for loops whenever possible).
  
2. The text report that you will have to hand in:
  * Summarize the work that you did and specify _exactly_ what your VO pipeline does:
    * i.e. whether it is monocular or stereo
    * How well it performs on the provided datasets (**with plots**). 
    * Describe bonus features that you implemented, or the additional work you did that over the basic VO pipeline required, and how this impacts the quality of your VO pipeline.</br>
       Additional features which degrades the quality of your VO pipeline will be accepted and valued as long as:
       * the implemented feature is properly motivated and described
       * an analysis showing the effect of your additional feature is provided in the report.
  * A video of your working pipeline.

3. Whether you implemented one or multiple bonus features, or did something in addition to the basic requirements (and explained it in the report). Note that no penalty is given if a bonus feature requires stereo frames. A (non-exhaustive) list of ideas is given down here:

## Installation

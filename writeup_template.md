## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.
In this project, I wrote code that recognized objects from a pointcloud (pr2_robot/scripts/project_template.py). 
For the first part of the project, I performed filtering and segmentation on the pointcloud in order to accurately
isolate the objects. After converting the ROS pointcloud into PCL pointcloud datatype, I used PCL to firstly perform
statistical outlier filtering, which removed irrelevant noise. This can be seen by comparing the original and filtered
pointclouds:

![original](rviz_original.png)
![filtered](rviz_filtered.png)

Next I performed voxel grid downsampling to downsample the pointcloud, improving computational efficiency for the later
clustering and detection steps. Next I performed passthrough filtering to isolate the table from the rest of the 
image. I filtered first on the z axis to remove the table leg, and then on the x axis to remove part of the dropboxes
that were present in the image. Finally, I used RANSAC segmentation to separate the table from the objects.
After downsampling and segmentation, the pointcloud only contained the objects of interest:

![segmented](rviz_segmented.png)

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  
In the second part of the project, I used clustering techniques to isolate individual objects from each other. This was
accomplished via PCL's Euclidean clustering functionality with a k-d tree to extract clusters of points close together
in space. Afterwards, each cluster was assigned an arbitrary color and the pointcloud appeared as:

![clustered](rviz_clustered.png)

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
In the final part of the perception pipeline, I used the individual objects, now isolated into different clusters, to
recognition which object they were based on the attributes of known objects. To do this, I first extracted color and 
normal histrogram features from 100 random orientations of each object (pr2_robot/scripts/capture_features.py). 
I then used these features to train an SVM classifier using sklearn (pr2_robot/scripts/train_svm.py). Finally, I used
the resultant model.sav files to perform real time classification of the clustered objects from the pointcloud, 
ultimately labeling each cluster.
  

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

In this part of the project, I situated my filtering, clustering, and recognition code with a ROS node in order to
perform real time 3D object recognition. I create a `perception` ROS node that subscribed to the pointcloud topic of 
the PR2 RGBD camera. Within the subscriber callback, `pcl_callback` I added all the abovementioned code, which ultimately generated
a list of recognized objects with labels and corresponding pointclouds. I passed this list into a separate function,
`pr2_mover()`, which first parsed the list of known objects for the given world, and then compared each object in that 
pick list to the recognized object. Once a match was found, the centroid of the object was computed and bundled with
other metadata including the label, dropbox place position. Finally, all this data was written out to a yaml file.

This process worked quite well...

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  




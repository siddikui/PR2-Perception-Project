## Project: Perception Pick & Place
### This is a write up for object recognition in a cluttered table top environment with an RGBD camera mounted on a PR2 robot. The project involves creating a perception pipeline with filtering, clustering, segmentation, object recognition using a Support Vector Machine and creating ROS messages for PR2 pick and place cycle.

---

See [Exercises](https://github.com/udacity/RoboND-Perception-Exercises) before setting up the project.
Look at the [Project Repository](https://github.com/udacity/RoboND-Perception-Project) to set up the project. 

#### Steps to complete the project:
1. Extracting features and training an SVM model for different objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models to identify). 
2. Writing a ROS node and subscribing to `/pr2/world/points` topic. This topic contains noisy point cloud data.
3. Using filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Applying Euclidean clustering to create separate clusters for individual items.
5. Performing object recognition on these objects and assigning them labels (markers in RViz).
6. Calculating the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Creating ROS messages containing the details of each object (name, pick_pose, etc.) and writing these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Correctly identifying 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.

[//]: # (Image References)

[image1]: ./misc/Original_Pt_Cld.png
[image2]: ./misc/Original_Cld_Top.png

[image3]: ./misc/Passthru_Cld_Top.png
[image4]: ./misc/Stats_Cld.png
[image5]: ./misc/Vox_Cld.png
[image6]: ./misc/Table_Cld.png
[image7]: ./misc/Object_Cld.png

[image8]: ./misc/Cluster_Cld.png

[image9]: ./misc/World3_Conf_Mat.png
[image10]: ./misc/World2_Conf_Mat.png
[image11]: ./misc/World1_Conf_Mat.png


[image12]: ./misc/Detection_World3.png
[image13]: ./misc/Detection_World2.png
[image14]: ./misc/Detection_World1.png
[image15]: ./misc/FinalShot.png

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.   

---

### Filtering and RANSAC Implementation

The original pointcloud from the RGBD camera is quite noisy and the goal is to extract and recognize the desired objects on the tabletop. Publishing the pointcloud in the Rviz and looking at it through the perception pipeline is a good way to tune the pipeline. Please see the pcl_callback() function (lines 52-90) in the /pr2_robot/scripts/project_template.py. 

![alt text][image1]

Viewed from the top, it can be seen there is a lot of unnecessary pointcloud data. Therefore it's a good idea to crop the point cloud first in the z and y dimensions.

![alt text][image2]

Applying a Passthrough filter will reduce the point cloud size and will make working further less computationaly intensive.

![alt text][image3]

A statistical outlier filter then helps get rid of noisy data and we get a nice looking clean tabletop environment.

![alt text][image4]

The pointcloud data is still very dense, correctly applying voxel grid sampling reduces the data points while still keeping track of the original object shapes.

![alt text][image5]
 
And now we want to extract only the objects and remove the tabletop. RANSAC plane fitting can find different types of shapes in a given data. Since the table is a plane, we use RANSAC algorithm to extract table indices (inliers). The outliers will therefore be the objects. So here's our tabletop. 

![alt text][image6]

And the objects.

![alt text][image7]

### Clustering for Segmentation  

As we have extracted the objects, we can now apply Euclidean Clustering to to group the objects into seperate clusters. Setting the cluster tolerance to 0.05, minimum cluster size to 50 and maximum cluster size to 2500 yeilds the desired results. Once again, visualising in Rviz gives a cool clustered pointcloud. Please see the pcl_callback() function (lines 92-118) in the /pr2_robot/scripts/project_template.py. 

![alt text][image8]

### Feature Extraction and SVM Training

We can now extract the color histogram features as well as surface normal histogram features to extract features for each object in random orientations (I used 50 orientations each object, gathering more data can increase the model performance). It's also better to use HSV color space instead of RGB, then combine together the color and normal features and use those for training an SVM. I ended up using 32 color bins with range (0, 256) and 32 surface normals with range (-1,1). Since the objects vary a lot in color, a color bin size of 16 and 8 should also do a fine job. Please see capture_features.py, features.py, train_svm.py and the pcl_callback() function (lines 140-172) in the /pr2_robot/scripts/project_template.py. We use 5 fold cross validation to train the SVM. The overall Accuracy Scores for World3, World2 and World1 are 0.90, 0.88, and 0.93 respectively. 

Confusion Matrices for World3: 

![alt text][image9]

Confusion Matrices for World2:

![alt text][image10]

Confusion Matrices for World1:

![alt text][image11]

Later during the object recognition test phase, the higher accuracy models didn't perform that well and were overftting with using larger feature vector sizes i.e more color and normal features. Collecting more random samples might help getting even better models specially for World3 where we have 8 different objects. 


### Object Recognition

Next we test our trained SVMs in the given world environments and publish our detected object list and object markers in the Rviz.

For world3, 7 out of 8 objects are being correctly classified hence an accuracy of 87.5. The 'book' has been misclassified as 'snacks'. Tuning the features and capturing more features would alleviate this problem.

![alt text][image12]

For world2, all objects are correctly classified i.e 5/5 with 100% accuracy.

![alt text][image13]

For world1, all objects are correctly classified i.e 3/3 with 100% accuracy.

![alt text][image14]

### Pick and Place Setup

Now for picking and placing up the objects, a proper ROS message format is needed which looks like this:

test_scene_num    std_msgs/Int32

object_name       std_msgs/String

arm_name          std_msgs/String

pick_pose         geometry_msgs/Pose

place_pose        geometry_msgs/Pose

Therefore we iterate in our detected object list, see if the object exists in the given pick list (object_name message), we compute the centroid of the object which makes the pick_pose message. We also extract the object group from the dropbox parameter, i.e. green or red and associate right and left hands respectively hence making up arm_name message. Next we use the object group to decide the dropbox, i.e. right or left. The dropbox position makes the place_pose message. The test_scene_num message depends on world used. We do this for all the detected objects and make a dictionary list of messages using the function make_yaml_dict() in the project_template.py. And lastly we create the output*.yaml files using send_to_yaml() function. The output*.yaml files exist in the /pr2_robot.scripts folder.

Please see the pr2_mover() function in the /pr2_robot/project_template.py.

And a last screenshot of PR2 picking up the smaller soap.

![alt text][image14]

The project can be extended in various ways such as: 

Complex object recognition i.e. bringing in more objects, gathering features other than color or surface normals. 

Trying out different classifiers such as Covnets instead of SVM. 

Tuning the pick and place cycle such that the dropped objects do not land on eachother. 

This pipeline would cause the robot arm to hit some of the objects in its way to pick up the desired object, therefore proper collision avoidance is also required.


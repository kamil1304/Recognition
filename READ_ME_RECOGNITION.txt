--Functionality
Main task of a node is detecting  object  which are seen by PrimeSens Sensor.  
Node require information form  the sensor, a PointCloud,  and compare it with  
default base of  objects. If program can find resemblance, then  surround an object
 by  Cube of certain color, otherwise, if detected object is unknown, it is surrounded by red cube.
 Amount of detected objects and their accuracy  strongly depends on  amount of default  object base. 


--Usage
Starting a node with ' roslaunch ' command  open  a node and retrive parameters  from  a paramater file.
 Node needs information from sensor about PointCloud- this is a subscribe role of a node. Node also  publish markers, 
 which are situated in a place of recognized objects. In Rviz You can dipsplay Markers by 'Add' MarkerArray and choosing topic '/vizualization_markers'.

-- File/directory Structure
* /launch/object_recognition.launch-- lauchfile which start a node and retrive information from  parameters.yaml
* /parameters.yaml -- all parameters used in a program
* /objects/list.txt -- all objects which can be detected
* /objects -- here are folders with '*.pcd' files which corespond to certian object
* /CmakeList.txt -- information about the node

* /src/ObjectRecognitionMain.cpp- Main class, here is created ROS node, advertiser & publisher. Calling this class,
   user can detect objects using PrimeSens sensor. Default objects are loaded in the begining of program by Iocloud and compared with  PointCloud sent by sensor.
*/src/ClasterDivision -- Class contains methods to cluster PointClouds
*/src/DescriptorsTypes -- virtual Class enables to create different types of descriptors for PointClouds,
*/src/DescriptorSpin -- useage of SpinImahe descriptor
*/src/Filtering --Class enables to filer  PointCloud using different methods. Main taks is do decrease amount of points which increase next operations on the PointCloud,
*/src/Histogramoper -- Class contain methods  which can operate on histograms
*/src/Iocloud --Class contain methods enables to input and output PointCloud such as save and load
*/src/Listener --  Listener enables to gather data from topic from ROS and operate on it,
*/src/Recognition --Class enables to find resemblance  between two PointClouds
*/src/Rospublish --Class create a markers and allow to publish them in certian position


--Workflow
 Actual parameters as set in .yaml to enable quick  detecting of an object. Specialy actuall settings of filters allow for that. Unfortunately  it makes  a detection not fully robust. 

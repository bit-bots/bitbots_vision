# represents a human pose in OpenPose format
# ordering is as follows (commented to send less data):

#uint8 NOSE=0
#uint8 NECK=1
#uint8 RShoulder=2
#uint8 RElbow=3
#uint8 RWrist=4
#uint8 LShoulder=5
#uint8 LElbow=6
#uint8 LWrist=7
#uint8 RHip=8
#uint8 RKnee=9
#uint8 RAnkle=10
#uint8 LHip=11
#uint8 LKnee=12
#uint8 LAnkle=13
#uint8 REye=14
#uint8 LEye=15
#uint8 REar=16
#uint8 LEar=17
#uint8 BACKGROUND=18

geometry_msgs/Point32[] keypoints
std_msgs/Float32 score
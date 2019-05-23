/*
Edited by the Japanese Turtle for automated parking
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <find_object_2d/ObjectsStamped.h>
#include <QtCore/QString>
#include <iostream>

class TfExample
{
public:
	TfExample() :
		mapFrameId_("/map"),
		objFramePrefix_("object")
	{
		ros::NodeHandle pnh("~");
		pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
		pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);


		nh.getParam("frame_offset", frame_offset);
		subs_ = nh.subscribe("objectsStamped", 1, &TfExample::objectsDetectedCallback, this);
	}

	// Here I synchronize with the ObjectsStamped topic to
	// know when the TF is ready and for which objects
	void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg)
	{
		if(msg->objects.data.size())
		{
			char multiSubId = 'b';
			int previousId = -1;
			for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
			{
				// get data
				int id = (int)msg->objects.data[i];

				QString multiSuffix;
				if(id == previousId)
				{
					multiSuffix = QString("_") + multiSubId++;
				}
				else
				{
					multiSubId = 'b';
				}
				previousId = id;

				// "object_1", "object_1_b", "object_1_c", "object_2"
				std::string objectFrameId = QString("%1_%2%3").arg(objFramePrefix_.c_str()).arg(id).arg(multiSuffix).toStdString();

				tf::StampedTransform pose;
				tf::StampedTransform poseCam;
				try
				{
					// Get transformation from "object_#" frame to target frame "map"
					// The timestamp matches the one sent over TF
					//get transform from map frame to dock frame
					tfListener_.lookupTransform(mapFrameId_, objectFrameId, msg->header.stamp, pose);
					tfListener_.lookupTransform(msg->header.frame_id, objectFrameId, msg->header.stamp, poseCam);
				}
				catch(tf::TransformException & ex)
				{
					ROS_WARN("%s",ex.what());
					continue;
				}

				// Here "pose" is the position of the object "id" in "/map" frame.
				ROS_INFO("%s [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
						objectFrameId.c_str(), mapFrameId_.c_str(),
						pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
						pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());
				ROS_INFO("%s [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
						objectFrameId.c_str(), msg->header.frame_id.c_str(),
						poseCam.getOrigin().x(), poseCam.getOrigin().y(), poseCam.getOrigin().z(),
						poseCam.getRotation().x(), poseCam.getRotation().y(), poseCam.getRotation().z(), poseCam.getRotation().w());

				//update phantom frame transformations
				tx=pose.getOrigin().x();
				ty=pose.getOrigin().y();
			  tz=pose.getOrigin().z();
        //update phantom frame rotations in quaternions
		    q_received=pose.getRotation();


			}
		}
	} //bracket for objectsDetectedCallback
       void pub_phantom_frame(){
       tf::Transform tf_send;
			 tf_send.setOrigin(tf::Vector3(tx,ty,tz));

			 tf::Quaternion q_send;
			 q_send=q_received;
			 tf_send.setRotation(q_send);

       br.sendTransform(tf::StampedTransform(tf_send,ros::Time::now(),mapFrameId_,"phantom_dock"));
			 pub_dock_goal();
		 }

		 void pub_dock_goal(){
		 tf::Transform tf_dock_goal;
		 //we would want to set the dock goal right in front of the dock.
		 tf_dock_goal.setOrigin(tf::Vector3(frame_offset,0.0,0.0));
		 tf::Quaternion q_send;
		 //we would want to rotate the dock goal frame by 180 degrees(from phantom frame)
     q_send.setRPY(0,0,3.14159);
		 tf_dock_goal.setRotation(q_send);

		 br.sendTransform(tf::StampedTransform( tf_dock_goal,ros::Time::now(),"phantom_dock","dock_goal"));

		 }

private:
	ros::NodeHandle nh;
	std::string mapFrameId_;
	std::string objFramePrefix_;
  ros::Subscriber subs_;
  tf::TransformListener tfListener_;
	tf::TransformBroadcaster br;
	double frame_offset;

  double tx,ty,tz;// transformation values.
  tf::Quaternion q_received;

};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "tf_example_node");
		TfExample sync;
    ros::Rate loop_rate(10); //we shall send our frames at 10hz

		while(ros::ok()){
    sync.pub_phantom_frame();


		ros::spinOnce();
		loop_rate.sleep();
}









return 0;}

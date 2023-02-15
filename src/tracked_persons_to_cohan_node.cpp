// Software License Agreement (BSD License)
//
//  Copyright (c) 2022, Jarosław Karwowski, Robot Programming and Machine Perception Group, Warsaw University of Technology
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//  * Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <ros/ros.h>
#include <people_msgs/People.h>
#include <people_msgs_utils/person.h>
#include <human_msgs/TrackedHumans.h>
#include <human_msgs/TrackedSegmentType.h>

class PeopleCohanPlannerInterface {
public:
	PeopleCohanPlannerInterface() {
		nh_.param("stopped_vel_xy_threshold", stopped_vel_xy_threshold_, 0.05);
		nh_.param("stopped_vel_th_threshold", stopped_vel_th_threshold_, 0.10);
		// according to CoHAN documentation, only TORSO segment is taken into consideration
		nh_.param("tracked_segment", cohan_tracked_segment_, static_cast<int>(human_msgs::TrackedSegmentType::TORSO));

		pub_ = nh_.advertise<human_msgs::TrackedHumans>("/tracked_humans", 5);
		sub_ = nh_.subscribe<people_msgs::People>(
			"/people",
			5,
			&PeopleCohanPlannerInterface::peopleCb,
			this
		);
	}
	virtual ~PeopleCohanPlannerInterface() = default;

protected:
	void peopleCb(const people_msgs::PeopleConstPtr& msg) {
		// prepare compound message containter
		human_msgs::TrackedHumans people_cohan;
		people_cohan.header = msg->header;

		for (const auto& person: msg->people) {
			people_msgs_utils::Person pstd(person);
			human_msgs::TrackedHuman pcohan;

			// check if person is moving
			bool moving =
				(std::hypot(pstd.getVelocityX(), pstd.getVelocityY()) >= stopped_vel_xy_threshold_)
				|| (std::abs(pstd.getVelocityTheta()) >= stopped_vel_th_threshold_);
			pcohan.state = moving ? human_msgs::TrackedHuman::MOVING : human_msgs::TrackedHuman::STOPPED;

			/*
			 * Conversion from track name to track ID.
			 *
			 * NOTE: for tracks obtained from SPENCER perception stack, this will be valid.
			 * However, with approaches that name people with, e.g., literals, this will produce a runtime error.
			 * E.g. UUIDv5 should be used then.
			 */
			pcohan.track_id = std::stoul(pstd.getName());

			// copy tracking data to a segment
			human_msgs::TrackedSegment segment;
			segment.pose.pose.position = pstd.getPosition();
			segment.pose.pose.orientation = pstd.getOrientation();
			segment.twist.twist.linear.x = pstd.getVelocityX();
			segment.twist.twist.linear.y = pstd.getVelocityY();
			segment.twist.twist.angular.z = pstd.getVelocityTheta();
			segment.type = static_cast<int8_t>(cohan_tracked_segment_);
			// update segments set
			pcohan.segments.push_back(segment);

			// update people container
			people_cohan.humans.push_back(pcohan);
		}

		// publish gathered data
		pub_.publish(people_cohan);
	}

	ros::NodeHandle nh_;
	ros::Publisher pub_;
	ros::Subscriber sub_;

	double stopped_vel_xy_threshold_;
	double stopped_vel_th_threshold_;
	int cohan_tracked_segment_;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "tracked_persons_to_cohan_node");
	PeopleCohanPlannerInterface interface;

	ros::spin();
	return 0;
}

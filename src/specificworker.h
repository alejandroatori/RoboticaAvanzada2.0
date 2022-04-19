/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include "/home/robocomp/robocomp/classes/abstract_graphic_viewer/abstract_graphic_viewer.h"
#include <QGraphicsPolygonItem>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "/home/robocomp/robocomp/classes/grid2d/grid.h"
#include "HumanCameraBody.h"
#include <jsoncpp/json/json.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>


class SpecificWorker : public GenericWorker
{
Q_OBJECT
    struct Target
    {
        bool active = false;
        QPointF dest;
    };
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute();
	int startup_check();
	void initialize(int period);
    void new_target_slot(QPointF);
    void new_tilt_value_slot(int);
    void sweep_button_slot(bool);
    void trace_button_slot(bool);

private:
	bool startup_check_flag;
    AbstractGraphicViewer *viewer;

    //robot
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsEllipseItem *laser_in_robot_polygon;
    QPointF last_point;
    std::vector<QGraphicsLineItem *> lines;

    //Guarda los pares de articulaciones
    std::vector<std::pair<int, int>> vector_articulaciones;




    void cameraSetUp (const RoboCompHumanCameraBody::PeopleData &people_data);
    void drawSkeleton (cv::Mat &image, const RoboCompHumanCameraBody::PeopleData &people_data);
    void setRobotSpeed(float speed, float rot);
    void posicionRobot (RoboCompFullPoseEstimation::FullPoseEuler bState);
    void drawPeopleMap (const RoboCompHumanCameraBody::PeopleData &people, RoboCompFullPoseEstimation::FullPoseEuler bState);
    QPointF robot_to_world(RoboCompFullPoseEstimation::FullPoseEuler state, Eigen::Vector2f TW);
    QPointF world_to_robot(RoboCompFullPoseEstimation::FullPoseEuler state);
    QPointF world_to_robot2(Eigen::Vector2f point, RoboCompFullPoseEstimation::FullPoseEuler bState);
    QPointF robot_to_world2(Eigen::Vector2f TW, RoboCompFullPoseEstimation::FullPoseEuler bState);


    ////////////////////////////////////////////////////////////////////////////////////////
    void draw_laser(const RoboCompLaser::TLaserData &ldata);
    void world_to_robot5(Eigen::Vector2f robot_eigen, Eigen::Vector2f target_eigen, RoboCompFullPoseEstimation::FullPoseEuler bState);
    float speed_multiplier(float rot, float dist);

    // grid
    int TILE_SIZE = 100;
    QRectF dimensions;
    Grid grid;

    //target
    int state;
    Target target;
    float dist;
    float beta;

    //matriz de rotacion
    //https://eigen.tuxfamily.org/dox/classEigen_1_1AngleAxis.html

    Eigen::AngleAxis<float> x_axis_rotation_matrix = Eigen::AngleAxisf (1.3, Eigen::Vector3f::UnitX());
//    Eigen::AngleAxis<float> z_axis_rotation_matrix = Eigen::AngleAxisf (servo_position, Eigen::Vector3f::UnitZ());

};

#endif

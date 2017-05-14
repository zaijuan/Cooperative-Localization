/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2015 Taihú Pire and Thomas Fischer
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire <tpire at dc dot uba dot ar>
 *           Thomas Fischer <tfischer at dc dot uba dot ar>
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */
#pragma once

#include "CameraPose.hpp"
#include "Measurement.hpp"
#include "Measurement_3d_2d.hpp"
#include <tf/LinearMath/Transform.h>
#include <g2o/core/sparse_optimizer.h>
#include <opencv2/core/eigen.hpp>
#include <g2o/types/sba/types_sba.h>
#include <g2o/core/base_binary_edge.h>

class tracker_g2o
{
public:
    tracker_g2o(
            const cv::Matx33d& intrinsicLeft,
            const cv::Matx33d& intrinsicRight,
            double stereo_baseline,
            const cv::Matx33d& intrinsic_hd
            );

    /**
     * RefineCameraPose is the main working part of the tracker:
     * call this for every frame.
     * This will estimate the new pose of the system
     * and return a potential CameraPose.
     */
    CameraPose RefineCameraPose(const CameraPose& estimatedCameraPose,
                                const std::map<MapPoint *, std::pair<Measurement, Measurement> > &measurementsStereo,
                                const std::map<MapPoint *, Measurement> &measurementsLeft,
                                const std::map<MapPoint *, Measurement> &measurementsRight,
                                const std::vector<Measurement_3d_2d> &measurements_arsys,
                                const CameraPose& ApolloCamPose, bool include_marker_info);

protected:
    Eigen::Vector2d focal_length_,focal_length_hd;
    Eigen::Vector2d principal_point_,principal_point_hd;

    // G2O optimizer
    g2o::SparseOptimizer optimizer_;
    double gainTerminateThreshold_;

    // Calibration
    cv::Matx33d intrinsic_hd;
    cv::Matx33d intrinsicLeft_;
    cv::Matx33d intrinsicRight_;

    double stereo_baseline_, stereo_baseline_n;

    double feature_error;     //  marker_error,  fixed_edge_error;

    tf::Transform T_B_camleft_B_backboard;

    //1-new   build Vertex of the object 3D point on the marker board
    g2o::OptimizableGraph::Vertex* BuildNewVertex(int vertex_id, const cv::Point3f& Point);
    //2-new   build Vertex of the relative transform from Boreas camera to one marker board
    g2o::OptimizableGraph::Vertex* BuildNewVertex(int vertex_id, cv::Vec4d orientation_cv,cv::Vec3d position_cv, const bool isFixed);
    //3-new   build the edge of the image projection of the marker board on the image plane of the boreas camera
    g2o::OptimizableGraph::Edge* BuildNewEdgeFromMarker(int pointId, int Brelative_ID, const cv::Point2d& projection);
    //4-new   build the almost fixed edge between the Boreas_world_frame and the marker_frame.
    g2o::OptimizableGraph::Edge* BuildNewEdgeFixed(int nMarkerPoint_BundleID, int nKF_BundleID, const cv::Point3d fixed_position_in_Boreas);

    g2o::OptimizableGraph::Vertex* BuildNewVertex(int vertex_id, const CameraPose& cameraPose, const bool isFixed);
    g2o::OptimizableGraph::Vertex* BuildNewVertex(int vertex_id, const MapPoint& mapPoint);
    g2o::OptimizableGraph::Edge* BuildNewMonoEdge(int pointId, int keyFrameId, const cv::Point2d& projection);
    g2o::OptimizableGraph::Edge* BuildNewMonoEdgeRight(int pointId, int keyFrameId, const cv::Point2d& projection);
    g2o::OptimizableGraph::Edge* BuildNewStereoEdge(int pointId, int keyFrameId, const cv::Point2d& projectionLeft, const cv::Point2d& projectionRight);

    CameraPose GetPose(int keyFrameId);

private:
    cv::Point3d convertToWorldFrame( cv::Point3d point, tf::Transform T_world_B_camleft);
    cv::Point3d convertToBoreasFrame( cv::Point3d point, tf::Transform T_B_camleft_B_backboard);

};



/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Stephan Kurzawe
 *********************************************************************/

#include <costmap_prohibition_layer/costmap_prohibition_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_prohibition_layer_namespace::CostmapProhibitionLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace costmap_prohibition_layer_namespace
{
CostmapProhibitionLayer::CostmapProhibitionLayer()
{
}

void CostmapProhibitionLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb =
      boost::bind(&CostmapProhibitionLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  // get a pointer to the layered costmap and save resolution
  costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
  _costmap_resolution = costmap->getResolution();

  // set initial bounds
  _min_x = _min_y = _max_x = _max_y = 0;
  
  _source_frame = "map";
  nh.param("source_frame", _source_frame, _source_frame);

  // reading the prohibition areas out of the namespace of this plugin!
  // e.g.: "move_base/global_costmap/prohibition_layer/prohibition_areas"
  std::string params = "prohibition_areas";
  if (!parseProhibitionListFromYaml(&nh, params))
    ROS_ERROR_STREAM("Reading prohibition areas from '" << nh.getNamespace() << "/" << params << "' failed!");
  
  // compute map bounds for the current set of prohibition areas.
  computeMapBounds();
  
  ROS_INFO("CostmapProhibitionLayer initialized.");
}

void CostmapProhibitionLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}


void CostmapProhibitionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, 
                                           double *min_x, double *min_y, double *max_x, double *max_y)
{
    if (!enabled_)
        return;
    
    if (_prohibition_points.empty() && _prohibition_polygons.empty())
        return;

    transformProhibitionAreas();
    computeMapBounds();
    
//     ROS_INFO_STREAM("robotx: " << robot_x << " y: " << robot_y << " yaw: " << robot_yaw);
//     ROS_INFO_STREAM("origin_x: " << layered_costmap_->getCostmap()->getOriginX() << " origin_y: " << layered_costmap_->getCostmap()->getOriginY());
//     ROS_INFO_STREAM("min_x: " << *min_x << "/" << _min_x << ", " << "min_y: " << *min_y << "/" << _min_y << ", " << "max_x: " << *max_x << "/" << _max_x << ", " << "max_y: " << *max_y << "/" << _max_y);
    *min_x = std::min(*min_x, _min_x);
    *min_y = std::min(*min_y, _min_y);
    *max_x = std::max(*max_x, _max_x);
    *max_y = std::max(*max_y, _max_y);

}

void CostmapProhibitionLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  // set costs of polygons
  for (int i = 0; i < _prohibition_polygons_global.size(); ++i)
  {
      setPolygonCost(master_grid, _prohibition_polygons_global[i], LETHAL_OBSTACLE, min_i, min_j, max_i, max_j);
  }
      
//       ROS_INFO_STREAM("mini: " << min_i << " minj: " << min_j << " maxi: " << max_i << " maxj: " << max_j);
  // set cost of points
  for (int i = 0; i < _prohibition_points_global.size(); ++i)
  {
    unsigned int mx;
    unsigned int my;
    if (master_grid.worldToMap(_prohibition_points_global[i].x, _prohibition_points_global[i].y, mx, my))
    {
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
//     else
//       ROS_ERROR_STREAM("Prohibition Layer: Point Cost couldn't be set!");
  }
}

void CostmapProhibitionLayer::computeMapBounds()
{
  // reset bounds
  _min_x = _min_y = _max_x = _max_y = 0;
    
  // iterate polygons
  for (int i = 0; i < _prohibition_polygons_global.size(); ++i)
  {
    for (int j=0; j < _prohibition_polygons_global.at(i).size(); ++j)
    {
      double px = _prohibition_polygons_global.at(i).at(j).x;
      double py = _prohibition_polygons_global.at(i).at(j).y;
      _min_x = std::min(px, _min_x);
      _min_y = std::min(py, _min_y);
      _max_x = std::max(px, _max_x);
      _max_y = std::max(py, _max_y);
    }
  }

  // iterate points
  for (int i = 0; i < _prohibition_points_global.size(); ++i)
  {
      double px = _prohibition_points_global.at(i).x;
      double py = _prohibition_points_global.at(i).y;
      _min_x = std::min(px, _min_x);
      _min_y = std::min(py, _min_y);
      _max_x = std::max(px, _max_x);
      _max_y = std::max(py, _max_y);
  }
}

bool CostmapProhibitionLayer::transformProhibitionAreas()
{
    tf::StampedTransform tf_source_to_global;
    try
    {
        tf_->waitForTransform(layered_costmap_->getGlobalFrameID(), _source_frame, ros::Time::now(), ros::Duration(3));
        tf_->lookupTransform(layered_costmap_->getGlobalFrameID(), _source_frame, ros::Time(0), tf_source_to_global);
    }
    catch(const tf::TransformException& ex)
    {
        ROS_ERROR_STREAM("CostmapProhibitionLayer: " << ex.what());
        return false;
    }
    
    // TODO: only renew if transformation has changed!   
    
    // iterate polygons
    _prohibition_polygons_global.resize(_prohibition_polygons.size());
    for (int i = 0; i < _prohibition_polygons.size(); ++i)
    {
        _prohibition_polygons_global[i].resize(_prohibition_polygons.at(i).size());
        for (int j=0; j < _prohibition_polygons.at(i).size(); ++j)
        {
            transformPoint(tf_source_to_global, _prohibition_polygons.at(i).at(j), _prohibition_polygons_global.at(i).at(j));
        }
    }

    // iterate points
    _prohibition_points_global.resize(_prohibition_points.size());
    for (int i = 0; i < _prohibition_points.size(); ++i)
    {
        transformPoint(tf_source_to_global, _prohibition_points[i], _prohibition_points_global[i]);
    }    
    return true;
}


void CostmapProhibitionLayer::transformPoint(const tf::StampedTransform& transform, const geometry_msgs::Point& pt_in, geometry_msgs::Point& pt_out)
{
    tf::Stamped<tf::Point> pin, pout;
    tf::pointMsgToTF(pt_in, pin);
    pout.setData(transform * pin);
    tf::pointTFToMsg(pout, pt_out);     
}

void CostmapProhibitionLayer::setPolygonCost(costmap_2d::Costmap2D &master_grid, const std::vector<geometry_msgs::Point>& polygon, unsigned char cost,
                                             int min_i, int min_j, int max_i, int max_j)
{
   std::vector<PointInt> map_polygon;
   for (unsigned int i = 0; i < polygon.size(); ++i)
   {
     PointInt loc;
     master_grid.worldToMapNoBounds(polygon[i].x, polygon[i].y, loc.x, loc.y);
     map_polygon.push_back(loc);
   }
  
    std::vector<PointInt> polygon_cells;
    
    // get the cells that fill the polygon
    polygonOutlineCells(map_polygon, polygon_cells);
    
   // set the cost of those cells
   for (unsigned int i = 0; i < polygon_cells.size(); ++i)
   {
       int mx = polygon_cells[i].x;
       int my = polygon_cells[i].y;
       // check if point is outside bounds
       if (mx < min_i || mx >= max_i)
           continue;
       if (my < min_j || my >= max_j)
           continue;
       master_grid.setCost(mx, my, cost);
   }
}


// load prohibition positions out of the rosparam server
bool CostmapProhibitionLayer::parseProhibitionListFromYaml(ros::NodeHandle *nhandle, const std::string &param)
{
  std::lock_guard<std::mutex> l(_parse_mutex);
  std::unordered_map<std::string, geometry_msgs::Pose> map_out;

  XmlRpc::XmlRpcValue param_yaml;

  bool ret_val = true;

  if (nhandle->getParam(param, param_yaml))
  {
    if (param_yaml.getType() == XmlRpc::XmlRpcValue::TypeArray)  // list of goals
    {
      for (int i = 0; i < param_yaml.size(); ++i)
      {
        if (param_yaml[i].getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
          std::vector<geometry_msgs::Point> vector_to_add;

          /* **************************************
           * differ between points and polygons
           * lines get to a polygon with the resolution
           * of the costmap
           **************************************** */

          // add a point
          if (param_yaml[i].size() == 1)
          {
            geometry_msgs::Point point;
            ret_val = getPoint(param_yaml[i][0], point);
            _prohibition_points.push_back(point);
          }
          // add a line
          else if (param_yaml[i].size() == 2)
          {
            if (param_yaml[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
	      param_yaml[i][0].getType() == XmlRpc::XmlRpcValue::TypeInt)
            {
              // add a lonely point
              geometry_msgs::Point point;
              ret_val = getPoint(param_yaml[i], point);
              _prohibition_points.push_back(point);
            }
            else
            {
              // add a line!
              geometry_msgs::Point point_A;
              ret_val = getPoint(param_yaml[i][0], point_A);
              vector_to_add.push_back(point_A);

              geometry_msgs::Point point_B;
              ret_val = getPoint(param_yaml[i][1], point_B);
              vector_to_add.push_back(point_B);

              // calculate the normal vector for AB
              geometry_msgs::Point point_N;
              point_N.x = point_B.y - point_A.y;
              point_N.y = point_A.x - point_B.x;

              // get the absolute value of N to normalize and get
              // it to the length of the costmap resolution
              double abs_N = sqrt(pow(point_N.x, 2) + pow(point_N.y, 2));
              point_N.x = point_N.x / abs_N * _costmap_resolution;
              point_N.y = point_N.y / abs_N * _costmap_resolution;

              // calculate the new points to get a polygon which can be filled
              geometry_msgs::Point point;
              point.x = point_A.x + point_N.x;
              point.y = point_A.y + point_N.y;
              vector_to_add.push_back(point);

              point.x = point_B.x + point_N.x;
              point.y = point_B.y + point_N.y;
              vector_to_add.push_back(point);

              _prohibition_polygons.push_back(vector_to_add);
            }
          }
          // add a point or add a polygon
          else if (param_yaml[i].size() >= 3)
          {
            // add a polygon with any number of points
            for (int j = 0; j < param_yaml[i].size(); ++j)
            {
              geometry_msgs::Point point;
              ret_val = getPoint(param_yaml[i][j], point);
              vector_to_add.push_back(point);
            }
            _prohibition_polygons.push_back(vector_to_add);
          }
        }
        else
        {
          ROS_ERROR_STREAM("Prohibition Layer:" << param << " with index " << i << " is not correct.");
          ret_val = false;
        }
      }
    }
    else
    {
      ROS_ERROR_STREAM("Prohibition Layer: " << param << "struct is not correct.");
      ret_val = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Prohibition Layer: Cannot read " << param << " from parameter server");
    ret_val = false;
  }
  return ret_val;
}

// get a point out of the XML Type into a geometry_msgs::Point
bool CostmapProhibitionLayer::getPoint(XmlRpc::XmlRpcValue &val, geometry_msgs::Point &point)
{
  try
  {
    // check if there a two values for the coordinate
    if (val.getType() == XmlRpc::XmlRpcValue::TypeArray && val.size() == 2)
    {
      auto convDouble = [](XmlRpc::XmlRpcValue &val) -> double
      {
        if (val.getType() == XmlRpc::XmlRpcValue::TypeInt)  // XmlRpc cannot cast int to double
          return int(val);
        return val;  // if not double, an exception is thrown;
      };

      point.x = convDouble(val[0]);
      point.y = convDouble(val[1]);
      point.z = 0.0;
      return true;
    }
    else
    {
      ROS_ERROR_STREAM("Prohibition_Layer: A point has to consist two values!");
      return false;
    }
  }
  catch (const XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR_STREAM("Prohibition Layer: Cannot add current point: " << ex.getMessage());
    return false;
  }
}

}  // end namespace

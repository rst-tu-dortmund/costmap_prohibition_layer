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
  costmap_2d::Costmap2D *layered_costmap = layered_costmap_->getCostmap();
  _costmap_resolution = layered_costmap->getResolution();

  // reading the prohibition areas out of the namespace of this plugin!
  // e.g.: "move_base/global_costmap/prohibition_layer/prohibition_areas"
  std::string params = "prohibition_areas";
  if (!parseProhibitionListFromYaml(&nh, params))
      ROS_ERROR_STREAM("Reading all prohibition areas failed!");
}

void CostmapProhibitionLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void CostmapProhibitionLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  // set costs of polygons
  for (int i = 0; i < _prohibition_polygons.size(); i++)
  {
    if (!master_grid.setConvexPolygonCost(_prohibition_polygons[i], LETHAL_OBSTACLE))
      ROS_ERROR_STREAM("Prohibition Layer: Polygon Cost couldn't be filled!");
  }

  // set cost of points
  for (int i = 0; i < _prohibition_points.size(); i++)
  {
    unsigned int mx;
    unsigned int my;
    if (master_grid.worldToMap(_prohibition_points[i].x, _prohibition_points[i].y, mx, my))
    {
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
    else
      ROS_ERROR_STREAM("Prohibition Layer: Point Cost couldn't be set!");
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
            if (param_yaml[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble)
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
      ROS_INFO_STREAM("Type: " << val.getType() << " Size: " << val.size());
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

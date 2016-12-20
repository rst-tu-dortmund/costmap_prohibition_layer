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
#ifndef COSTMAP_PROHIBITION_LAYER_H_
#define COSTMAP_PROHIBITION_LAYER_H_

#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <tf/transform_datatypes.h>
#include <mutex>
#include <geometry_msgs/PoseArray.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <unordered_map>

namespace costmap_prohibition_layer_namespace
{
    
// point with integer coordinates  
struct PointInt
{
    int x;
    int y;
};
    
class CostmapProhibitionLayer : public costmap_2d::Layer
{
public:
  /**
   * unused constructor
   */
  CostmapProhibitionLayer();

  /**
 * function which get called at initializing the costmap
 * define the reconfige callback, get the reoslution
 * and read the prohibitions from the ros-parameter server
 */
  virtual void onInitialize();

  /**
 * This is called by the LayeredCostmap to poll this plugin
 * as to how much of the costmap it needs to update.
 * Each layer can increase the size of this bounds. 
 */
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y);
  
    /**
 * function which get called at every cost updating procdure
 * of the overlayed costmap. The before readed costs will get
 * filled
 */
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  /**
   * overlayed reconfigure callback function
    */
  void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level);

    
  void computeMapBounds();
  
  bool transformProhibitionAreas();
  void transformPoint(const tf::StampedTransform& transform, const geometry_msgs::Point& pt_in, geometry_msgs::Point& pt_out);
  
  void setPolygonCost(costmap_2d::Costmap2D &master_grid, const std::vector<geometry_msgs::Point>& polygon, unsigned char cost,
                      int min_i, int min_j, int max_i, int max_j);
  
  void polygonOutlineCells(const std::vector<PointInt>& polygon, std::vector<PointInt>& polygon_cells)
  {
     for (unsigned int i = 0; i < polygon.size() - 1; ++i)
     {
       raytrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, polygon_cells);
     }
     if (!polygon.empty())
     {
       unsigned int last_index = polygon.size() - 1;
       // we also need to close the polygon by going from the last point to the first
       raytrace(polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y, polygon_cells);
     }
  }
  
  void raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt>& cells)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    PointInt pt;
    pt.x = x0;
    pt.y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;
    
    for (; n > 0; --n)
    {
         cells.push_back(pt);

        if (error > 0)
        {
            pt.x += x_inc;
            error -= dy;
        }
        else
        {
            pt.y += y_inc;
            error += dx;
        }
    }
}

  
  /**
   * read the prohibition areas in YAML-Format from the
   * ROS parameter server in the namespace of this
   * plugin
   * e.g. /move_base/global_costmap/prohibition_layer/param
   *
   * @param nhandle     pointer to the ros-Node handle
   * @param param       name of the parameter where the
   *                    prohibition areas saved in YAML format
   *
   * @return bool       true if the parsing was successful
   *                    false if it wasn't
    */
  bool parseProhibitionListFromYaml(ros::NodeHandle* nhandle, const std::string& param);

  /**
 * get a geometry_msgs::Point from a YAML-Array
 * The z-coordinate get always written to zero!
 *
 * @param val         YAML-array with to point-coordinates (x and y)
 * @param point       variable where the determined point get saved
 *
 * @return bool       true if the determining was successful
 *                    false if it wasn't
  */
  bool getPoint(XmlRpc::XmlRpcValue& val, geometry_msgs::Point& point);

  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;          //!< dynamic_reconfigure server for the costmap
  std::mutex _parse_mutex;                                                      //!< mutex for the YAML Import
  double _costmap_resolution;                                                   //!< resolution of the overlayed costmap to create the thinnest line out of two points
  std::string _source_frame;                                                    //!< coordinate frame in which the prohibition points and polygons are defined
  std::vector<geometry_msgs::Point> _prohibition_points;                        //!< vector to save the lonely points in source coordinates
  std::vector<geometry_msgs::Point> _prohibition_points_global;                 //!< vector to save the lonely points in global map coordinates
  std::vector<std::vector<geometry_msgs::Point>> _prohibition_polygons;         //!< vector to save the polygons (including lines) in source coordinates
  std::vector<std::vector<geometry_msgs::Point>> _prohibition_polygons_global;  //!< vector to save the polygons (including lines) in global coordinates
  double _min_x, _min_y, _max_x, _max_y;                                        //!< cached map bounds
};
}
#endif

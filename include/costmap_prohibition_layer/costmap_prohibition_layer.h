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
   * function which get called at every cost updating procdure
   * of the overlayed costmap. The before readed costs will get
   * filled
   */
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  /** 
   * overlayed reconfigure callback function
    */
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level); 
  
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

  

  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;          // dynamic_reconfigure server for the costmap
  std::mutex                                        _parse_mutex;               // mutex for the YAML Import  
  double                                            _costmap_resolution;        // resolution of the overlayed costmap to create the thinnest line out of two points
  std::vector<geometry_msgs::Point>                 _prohibition_points;        // vector to save the lonely points
  std::vector<std::vector<geometry_msgs::Point>>    _prohibition_polygons;      // vecotr to save the polygons (including lines)
  
};
}
#endif

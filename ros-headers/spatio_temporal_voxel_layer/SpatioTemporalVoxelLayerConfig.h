//#line 2 "/opt/ros/kinetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"
// *********************************************************
//
// File autogenerated for the spatio_temporal_voxel_layer package
// by the dynamic_reconfigure package.
// Please do not edit.
//
// ********************************************************/

#ifndef __spatio_temporal_voxel_layer__SPATIOTEMPORALVOXELLAYERCONFIG_H__
#define __spatio_temporal_voxel_layer__SPATIOTEMPORALVOXELLAYERCONFIG_H__

#if __cplusplus >= 201103L
#define DYNAMIC_RECONFIGURE_FINAL final
#else
#define DYNAMIC_RECONFIGURE_FINAL
#endif

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace spatio_temporal_voxel_layer
{
  class SpatioTemporalVoxelLayerConfigStatics;

  class SpatioTemporalVoxelLayerConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l,
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }

      virtual void clamp(SpatioTemporalVoxelLayerConfig &config, const SpatioTemporalVoxelLayerConfig &max, const SpatioTemporalVoxelLayerConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const SpatioTemporalVoxelLayerConfig &config1, const SpatioTemporalVoxelLayerConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, SpatioTemporalVoxelLayerConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const SpatioTemporalVoxelLayerConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, SpatioTemporalVoxelLayerConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const SpatioTemporalVoxelLayerConfig &config) const = 0;
      virtual void getValue(const SpatioTemporalVoxelLayerConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template <class T>
    class ParamDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string a_name, std::string a_type, uint32_t a_level,
          std::string a_description, std::string a_edit_method, T SpatioTemporalVoxelLayerConfig::* a_f) :
        AbstractParamDescription(a_name, a_type, a_level, a_description, a_edit_method),
        field(a_f)
      {}

      T (SpatioTemporalVoxelLayerConfig::* field);

      virtual void clamp(SpatioTemporalVoxelLayerConfig &config, const SpatioTemporalVoxelLayerConfig &max, const SpatioTemporalVoxelLayerConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;

        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const SpatioTemporalVoxelLayerConfig &config1, const SpatioTemporalVoxelLayerConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, SpatioTemporalVoxelLayerConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const SpatioTemporalVoxelLayerConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, SpatioTemporalVoxelLayerConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const SpatioTemporalVoxelLayerConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const SpatioTemporalVoxelLayerConfig &config, boost::any &val) const
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, SpatioTemporalVoxelLayerConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template<class T, class PT>
    class GroupDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string a_name, std::string a_type, int a_parent, int a_id, bool a_s, T PT::* a_f) : AbstractGroupDescription(a_name, a_type, a_parent, a_id, a_s), field(a_f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, SpatioTemporalVoxelLayerConfig &top) const
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T (PT::* field);
      std::vector<SpatioTemporalVoxelLayerConfig::AbstractGroupDescriptionConstPtr> groups;
    };

class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(SpatioTemporalVoxelLayerConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("enabled"==(*_i)->name){enabled = boost::any_cast<bool>(val);}
        if("publish_voxel_map"==(*_i)->name){publish_voxel_map = boost::any_cast<bool>(val);}
        if("voxel_size"==(*_i)->name){voxel_size = boost::any_cast<double>(val);}
        if("combination_method"==(*_i)->name){combination_method = boost::any_cast<int>(val);}
        if("mark_threshold"==(*_i)->name){mark_threshold = boost::any_cast<double>(val);}
        if("update_footprint_enabled"==(*_i)->name){update_footprint_enabled = boost::any_cast<bool>(val);}
        if("track_unknown_space"==(*_i)->name){track_unknown_space = boost::any_cast<bool>(val);}
        if("decay_model"==(*_i)->name){decay_model = boost::any_cast<int>(val);}
        if("voxel_decay"==(*_i)->name){voxel_decay = boost::any_cast<double>(val);}
        if("mapping_mode"==(*_i)->name){mapping_mode = boost::any_cast<bool>(val);}
        if("map_save_duration"==(*_i)->name){map_save_duration = boost::any_cast<double>(val);}
      }
    }

    bool enabled;
bool publish_voxel_map;
double voxel_size;
int combination_method;
double mark_threshold;
bool update_footprint_enabled;
bool track_unknown_space;
int decay_model;
double voxel_decay;
bool mapping_mode;
double map_save_duration;

    bool state;
    std::string name;

    
}groups;



//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool enabled;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool publish_voxel_map;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double voxel_size;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      int combination_method;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double mark_threshold;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool update_footprint_enabled;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool track_unknown_space;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      int decay_model;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double voxel_decay;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool mapping_mode;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double map_save_duration;
//#line 228 "/opt/ros/kinetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("SpatioTemporalVoxelLayerConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }

    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }

    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const SpatioTemporalVoxelLayerConfig &__max__ = __getMax__();
      const SpatioTemporalVoxelLayerConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const SpatioTemporalVoxelLayerConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }

    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const SpatioTemporalVoxelLayerConfig &__getDefault__();
    static const SpatioTemporalVoxelLayerConfig &__getMax__();
    static const SpatioTemporalVoxelLayerConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();

  private:
    static const SpatioTemporalVoxelLayerConfigStatics *__get_statics__();
  };

  template <> // Max and min are ignored for strings.
  inline void SpatioTemporalVoxelLayerConfig::ParamDescription<std::string>::clamp(SpatioTemporalVoxelLayerConfig &config, const SpatioTemporalVoxelLayerConfig &max, const SpatioTemporalVoxelLayerConfig &min) const
  {
    (void) config;
    (void) min;
    (void) max;
    return;
  }

  class SpatioTemporalVoxelLayerConfigStatics
  {
    friend class SpatioTemporalVoxelLayerConfig;

    SpatioTemporalVoxelLayerConfigStatics()
    {
SpatioTemporalVoxelLayerConfig::GroupDescription<SpatioTemporalVoxelLayerConfig::DEFAULT, SpatioTemporalVoxelLayerConfig> Default("Default", "", 0, 0, true, &SpatioTemporalVoxelLayerConfig::groups);
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.enabled = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.enabled = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.enabled = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<bool>("enabled", "bool", 0, "Enabled or not", "", &SpatioTemporalVoxelLayerConfig::enabled)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<bool>("enabled", "bool", 0, "Enabled or not", "", &SpatioTemporalVoxelLayerConfig::enabled)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.publish_voxel_map = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.publish_voxel_map = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.publish_voxel_map = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<bool>("publish_voxel_map", "bool", 0, "Publishes the voxel map", "", &SpatioTemporalVoxelLayerConfig::publish_voxel_map)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<bool>("publish_voxel_map", "bool", 0, "Publishes the voxel map", "", &SpatioTemporalVoxelLayerConfig::publish_voxel_map)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.voxel_size = 0.001;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.voxel_size = 10.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.voxel_size = 0.05;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<double>("voxel_size", "double", 0, "Size of the voxel in meters", "", &SpatioTemporalVoxelLayerConfig::voxel_size)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<double>("voxel_size", "double", 0, "Size of the voxel in meters", "", &SpatioTemporalVoxelLayerConfig::voxel_size)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.combination_method = -2147483648;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.combination_method = 2147483647;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.combination_method = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<int>("combination_method", "int", 0, "Combination methods", "{'enum_description': 'Combination methods', 'enum': [{'srcline': 7, 'description': 'uses the highest of the layers in the stack', 'srcfile': '/tmp/binarydeb/ros-kinetic-spatio-temporal-voxel-layer-1.2.1/cfg/SpatioTemporalVoxelLayer.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'largest'}, {'srcline': 8, 'description': 'overrides lower layers than this with these values', 'srcfile': '/tmp/binarydeb/ros-kinetic-spatio-temporal-voxel-layer-1.2.1/cfg/SpatioTemporalVoxelLayer.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'overwrite'}]}", &SpatioTemporalVoxelLayerConfig::combination_method)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<int>("combination_method", "int", 0, "Combination methods", "{'enum_description': 'Combination methods', 'enum': [{'srcline': 7, 'description': 'uses the highest of the layers in the stack', 'srcfile': '/tmp/binarydeb/ros-kinetic-spatio-temporal-voxel-layer-1.2.1/cfg/SpatioTemporalVoxelLayer.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'largest'}, {'srcline': 8, 'description': 'overrides lower layers than this with these values', 'srcfile': '/tmp/binarydeb/ros-kinetic-spatio-temporal-voxel-layer-1.2.1/cfg/SpatioTemporalVoxelLayer.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'overwrite'}]}", &SpatioTemporalVoxelLayerConfig::combination_method)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.mark_threshold = 0.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.mark_threshold = 200.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.mark_threshold = 0.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<double>("mark_threshold", "double", 0, "Over this threshold, the measurement will be marked", "", &SpatioTemporalVoxelLayerConfig::mark_threshold)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<double>("mark_threshold", "double", 0, "Over this threshold, the measurement will be marked", "", &SpatioTemporalVoxelLayerConfig::mark_threshold)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.update_footprint_enabled = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.update_footprint_enabled = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.update_footprint_enabled = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<bool>("update_footprint_enabled", "bool", 0, "Cleans where the robot's footprint is", "", &SpatioTemporalVoxelLayerConfig::update_footprint_enabled)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<bool>("update_footprint_enabled", "bool", 0, "Cleans where the robot's footprint is", "", &SpatioTemporalVoxelLayerConfig::update_footprint_enabled)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.track_unknown_space = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.track_unknown_space = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.track_unknown_space = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<bool>("track_unknown_space", "bool", 0, "If true, marks will be UNKNOWN (255) otherwise, FREE (0)", "", &SpatioTemporalVoxelLayerConfig::track_unknown_space)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<bool>("track_unknown_space", "bool", 0, "If true, marks will be UNKNOWN (255) otherwise, FREE (0)", "", &SpatioTemporalVoxelLayerConfig::track_unknown_space)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.decay_model = -2147483648;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.decay_model = 2147483647;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.decay_model = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<int>("decay_model", "int", 0, "Decay models", "{'enum_description': 'Decay models', 'enum': [{'srcline': 12, 'description': 'Linear', 'srcfile': '/tmp/binarydeb/ros-kinetic-spatio-temporal-voxel-layer-1.2.1/cfg/SpatioTemporalVoxelLayer.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'Linear'}, {'srcline': 13, 'description': 'Exponential', 'srcfile': '/tmp/binarydeb/ros-kinetic-spatio-temporal-voxel-layer-1.2.1/cfg/SpatioTemporalVoxelLayer.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'Exponential'}, {'srcline': 14, 'description': 'Permanent', 'srcfile': '/tmp/binarydeb/ros-kinetic-spatio-temporal-voxel-layer-1.2.1/cfg/SpatioTemporalVoxelLayer.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'Permanent'}]}", &SpatioTemporalVoxelLayerConfig::decay_model)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<int>("decay_model", "int", 0, "Decay models", "{'enum_description': 'Decay models', 'enum': [{'srcline': 12, 'description': 'Linear', 'srcfile': '/tmp/binarydeb/ros-kinetic-spatio-temporal-voxel-layer-1.2.1/cfg/SpatioTemporalVoxelLayer.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'Linear'}, {'srcline': 13, 'description': 'Exponential', 'srcfile': '/tmp/binarydeb/ros-kinetic-spatio-temporal-voxel-layer-1.2.1/cfg/SpatioTemporalVoxelLayer.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'Exponential'}, {'srcline': 14, 'description': 'Permanent', 'srcfile': '/tmp/binarydeb/ros-kinetic-spatio-temporal-voxel-layer-1.2.1/cfg/SpatioTemporalVoxelLayer.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'Permanent'}]}", &SpatioTemporalVoxelLayerConfig::decay_model)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.voxel_decay = -1.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.voxel_decay = 200.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.voxel_decay = 10.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<double>("voxel_decay", "double", 0, "Seconds if linear, e^n if exponential", "", &SpatioTemporalVoxelLayerConfig::voxel_decay)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<double>("voxel_decay", "double", 0, "Seconds if linear, e^n if exponential", "", &SpatioTemporalVoxelLayerConfig::voxel_decay)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.mapping_mode = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.mapping_mode = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.mapping_mode = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<bool>("mapping_mode", "bool", 0, "Mapping mode", "", &SpatioTemporalVoxelLayerConfig::mapping_mode)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<bool>("mapping_mode", "bool", 0, "Mapping mode", "", &SpatioTemporalVoxelLayerConfig::mapping_mode)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.map_save_duration = 1.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.map_save_duration = 2000.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.map_save_duration = 60.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<double>("map_save_duration", "double", 60, "f mapping, how often to save a map for safety", "", &SpatioTemporalVoxelLayerConfig::map_save_duration)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::ParamDescription<double>("map_save_duration", "double", 60, "f mapping, how often to save a map for safety", "", &SpatioTemporalVoxelLayerConfig::map_save_duration)));
//#line 245 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.convertParams();
//#line 245 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __group_descriptions__.push_back(SpatioTemporalVoxelLayerConfig::AbstractGroupDescriptionConstPtr(new SpatioTemporalVoxelLayerConfig::GroupDescription<SpatioTemporalVoxelLayerConfig::DEFAULT, SpatioTemporalVoxelLayerConfig>(Default)));
//#line 366 "/opt/ros/kinetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

      for (std::vector<SpatioTemporalVoxelLayerConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__);
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__);
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__);
    }
    std::vector<SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<SpatioTemporalVoxelLayerConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    SpatioTemporalVoxelLayerConfig __max__;
    SpatioTemporalVoxelLayerConfig __min__;
    SpatioTemporalVoxelLayerConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const SpatioTemporalVoxelLayerConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static SpatioTemporalVoxelLayerConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &SpatioTemporalVoxelLayerConfig::__getDescriptionMessage__()
  {
    return __get_statics__()->__description_message__;
  }

  inline const SpatioTemporalVoxelLayerConfig &SpatioTemporalVoxelLayerConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }

  inline const SpatioTemporalVoxelLayerConfig &SpatioTemporalVoxelLayerConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }

  inline const SpatioTemporalVoxelLayerConfig &SpatioTemporalVoxelLayerConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }

  inline const std::vector<SpatioTemporalVoxelLayerConfig::AbstractParamDescriptionConstPtr> &SpatioTemporalVoxelLayerConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<SpatioTemporalVoxelLayerConfig::AbstractGroupDescriptionConstPtr> &SpatioTemporalVoxelLayerConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const SpatioTemporalVoxelLayerConfigStatics *SpatioTemporalVoxelLayerConfig::__get_statics__()
  {
    const static SpatioTemporalVoxelLayerConfigStatics *statics;

    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = SpatioTemporalVoxelLayerConfigStatics::get_instance();

    return statics;
  }

//#line 7 "/tmp/binarydeb/ros-kinetic-spatio-temporal-voxel-layer-1.2.1/cfg/SpatioTemporalVoxelLayer.cfg"
      const int SpatioTemporalVoxelLayer_largest = 0;
//#line 8 "/tmp/binarydeb/ros-kinetic-spatio-temporal-voxel-layer-1.2.1/cfg/SpatioTemporalVoxelLayer.cfg"
      const int SpatioTemporalVoxelLayer_overwrite = 1;
//#line 12 "/tmp/binarydeb/ros-kinetic-spatio-temporal-voxel-layer-1.2.1/cfg/SpatioTemporalVoxelLayer.cfg"
      const int SpatioTemporalVoxelLayer_Linear = 0;
//#line 13 "/tmp/binarydeb/ros-kinetic-spatio-temporal-voxel-layer-1.2.1/cfg/SpatioTemporalVoxelLayer.cfg"
      const int SpatioTemporalVoxelLayer_Exponential = 1;
//#line 14 "/tmp/binarydeb/ros-kinetic-spatio-temporal-voxel-layer-1.2.1/cfg/SpatioTemporalVoxelLayer.cfg"
      const int SpatioTemporalVoxelLayer_Permanent = 2;
}

#undef DYNAMIC_RECONFIGURE_FINAL

#endif // __SPATIOTEMPORALVOXELLAYERRECONFIGURATOR_H__

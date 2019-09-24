#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include <gazebo/gazebo.hh>
#include <gazebo/test/ServerFixture.hh>
#include <iostream>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <string>
#include <fstream>
#include <streambuf>
#include <memory>
#include <thread>
#include <boost/algorithm/string.hpp>
#include <algorithm>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{

public:
  WorldPluginTutorial() : WorldPlugin()
  {
    printf("Hello World BULAT!\n");
  }

public:
  ~WorldPluginTutorial()
  {
    printf("Finished:(\n");
    if (this->rosQueueThread.joinable())
      this->rosQueueThread.join();
  }

public:
  void GenerateSDF(sdf::SDF &inputSDF, std::string modelName, ignition::math::Pose3d &pose3d)
  {
    sdf::ElementPtr model = inputSDF.Root()->GetElement("model");
    model->GetAttribute("name")->SetFromString(modelName);
    if (!model->HasElement("pose"))
      model->AddElement("pose")->Set<ignition::math::Pose3d>(pose3d);
    else
      model->GetElement("pose")->Set<ignition::math::Pose3d>(pose3d);
  }

public:
  void TestLoad(physics::WorldPtr &_parent)
  {
    sdf::SDF sphereSDF;
    sphereSDF.SetFromString(
       "<sdf version ='1.6'>\
       <world name='default'>\
       <model name='y_mesh'><static>true</static><link name='body'><visual name='visual'>\
        <pose frame=''>0 0 0 0 0 0</pose>\
        <geometry>\
        <mesh><uri>file:///home/optio32/test/map2.dae</uri></mesh>\
        </geometry>\
        </visual><collision name='collision1'>\
        <geometry>\
        <mesh><uri>file:///home/optio32/test/map2.dae</uri></mesh>\
        </geometry>\
        </collision>\
        </link>\
        </model>\
        </world>\
        </sdf>");
    _parent->InsertModelSDF(sphereSDF);
  }

public:
  void AddObject(std::string modelName, ignition::math::Pose3d& objectPose)
  {
    sdf::SDF sphereSDF;
    std::string modelPathFull = gazeboModelsPath + "/" + modelName + "/model.sdf";
    std::ifstream file(modelPathFull);
    if(!file.is_open())
    {
      std::cout << "Model open error.\n";
      return;
    }
    std::string str((std::istreambuf_iterator<char>(file)),
                 std::istreambuf_iterator<char>());
    sphereSDF.SetFromString(str);
    GenerateSDF(sphereSDF, modelName + "1", objectPose);
    world->InsertModelSDF(sphereSDF);
  }

public:
  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) override
  {
    this->world = _parent;
    TestLoad(_parent);
    ignition::math::Pose3d p;
    p.Pos().Set(1.0, 4.0, 7.0);
    p.Rot().Set(0, 1.2, 0.6, 4.1);
    AddObject("ambulance", p);
    p.Pos().Set(7.0, 8.0, 6.0);
    p.Rot().Set(0, 0.2, 0.0, 0.1);
    AddObject("fire_truck", p);
    p.Pos().Set(5.0, 12.0, 4.0);
    p.Rot().Set(0, 0.0, 0.0, 0.0);
    AddObject("person_standing", p);
  }

public:
  double x_axis_freq = 1.0;

private:
  ignition::math::Pose3d pose3D;
  std::thread rosQueueThread;
  //gazebo_plugin_tutorial::Command cmd;
  std::string gazeboModelsPath = "/home/optio32/.gazebo/models";
  physics::WorldPtr world;
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
} // namespace gazebo

typedef struct _MODEL_POSE
{
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
  float pitch = 0.0;
  float roll = 0.0;
  float yaw = 0.0;

  inline std::string toString()
  {
    std::ostringstream out;
    out.precision(3);
    out << std::fixed << "\n<pose>" << x << " " << y << " " << z << " " << pitch << " " << roll << " " << yaw << "</pose>";
    return out.str();
    //return "<pose>"+std::to_string(x)+" "+std::to_string(y)+" "+std::to_string(z)+" "+
    // std::to_string(pitch)+" "+std::to_string(roll)+" "+std::to_string(yaw)+'\n';
  }

} MODEL_POSE, *PMODEL_POSE;

class SDFReader
{
  public:
    SDFReader(std::string sdfFilePath)
    {
      std::ifstream file(sdfFilePath);
      if(!file.is_open())
      {
        std::cout << "SDF file open error.\n";
        return;
      }
      std::stringstream buffer;
      buffer << file.rdbuf();
      sdfDataPtr.reset(new std::string(buffer.str()));      
    }

  public:
    ~SDFReader()
    {

    }

  public:
    void InsertModel(std::string modelName, const std::string& modelData, MODEL_POSE* modelPose)
    {
      auto pos = sdfDataPtr->rfind("</model>");
      pos += 9;
      sdfDataPtr->insert(pos, modelData);      
      if(modelPose != nullptr)
      {
        std::string visualTag = "<visual name=\"visual\">";
        std::cout << "VisualTAg = " << visualTag << '\n'; 
        auto pos = sdfDataPtr->rfind(visualTag);
        if(pos != std::string::npos)
        {
          std::cout << "POSe = " << pos << '\n';
          sdfDataPtr->insert(pos+visualTag.size(), modelPose->toString());
        }
      }
    }

  public:
    void Save(std::string outputName)
    {
      std::ofstream out(outputName);
      out << *sdfDataPtr;
      out.close();
    }

  private:
    std::unique_ptr<std::string> sdfDataPtr;
};

class ObjectSpawner
{
  public:
    ObjectSpawner(std::string _mainWorldFilePath)
    {
        this->mainWorldFilePath = _mainWorldFilePath;
        sdfReaderPtr.reset(new SDFReader(mainWorldFilePath));
    }

  public:
    ~ObjectSpawner()
    {

    }

  public:
    void AddObject(std::string model, MODEL_POSE* modelPosePtr = nullptr)
    {
      std::string modelPathFull = gazeboModelsPath + "/" + model + "/model.sdf";
      std::ifstream file(modelPathFull);
      if(!file.is_open())
      {
        std::cout << "Model open error.\n";
        return;
      }
      std::string modelData((std::istreambuf_iterator<char>(file)),
                 std::istreambuf_iterator<char>());
      modelData.erase(0, modelData.find("\n")+1);
      modelData.erase(0, modelData.find("\n")+1);
      auto pos = modelData.find("</sdf>");
      //std::cout << pos;
      modelData.erase(pos, modelData.size());
      //std::cout << modelData;
      sdfReaderPtr->InsertModel(model, modelData, modelPosePtr);
    }

  public:
    void Save(std::string outputFileName)
    {
      sdfReaderPtr->Save(outputFileName);     
    }
  

  private:
    std::string gazeboModelsPath = "/home/optio32/.gazebo/models";
    std::string mainWorldFilePath;
    sdf::SDFPtr sdfFile;
    std::unique_ptr<SDFReader> sdfReaderPtr;
};

int main()
{
  ObjectSpawner os("/home/optio32/test/generated_world.world");
  MODEL_POSE mp;
  mp.x = 1.0;
  mp.y = 12.0;
  mp.z = 4.0;
  os.AddObject("ambulance", &mp);
  mp.x = 4.0;
  mp.y = 3.0;
  mp.pitch = 1.0;
  os.AddObject("house_1", &mp);
  os.Save("test.world"); 
  
  /*std::string text = "slalsf df sdfds sd sds d";
  auto pos = text.find("df");
  pos += 2;
  auto edited = text.insert(pos, "ss");
  std::cout << edited;*/

  return 0;
}
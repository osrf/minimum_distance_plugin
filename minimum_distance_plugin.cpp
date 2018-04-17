/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <ccd/ccd.h>
#include <ccd/quat.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/contacts.pb.h>

#include "CCDWrapper.hpp"

using gazebo::physics::LinkPtr;
using gazebo::physics::ModelPtr;

namespace minimum_distance_plugin {

const std::string PluginName = "minimum_distance_plugin";
const std::string PairElementName = "pair";
const std::string FromElementName = "from";
const std::string ToElementName = "to";
const std::string RangeElementName = "range";
const std::string ModelElementName = "model";
const std::string LinkElementName = "link";
const std::string CollisionElementName = "collision";
const std::string GeometryElementName = "geometry";
const std::string PoseElementName = "pose";

const std::string EmptyElementName = "empty";
const std::string BoxElementName = "box";
const std::string CylinderElementName = "cylinder";
const std::string SphereElementName = "sphere";
const std::string CapsuleElementName = "capsule";

///////////////////////////////////////////////
struct CollisionObject
{
  std::unique_ptr<ccdw::Convex> convex;
  LinkPtr link;
  ignition::math::Pose3d offset;

  CollisionObject() = default;
  CollisionObject(CollisionObject&&) = default;

  mutable ccdw::TransformedConvex tf;

  void update_tf() const
  {
    const ignition::math::Pose3d T =
        (ignition::math::Matrix4d(link->GetWorldPose().Ign())
        * ignition::math::Matrix4d(offset)).Pose();

    const ignition::math::Quaterniond q = T.Rot();
    const ignition::math::Vector3d v = T.Pos();

    tf.xform = ccdw::Transform3(
          ccdw::quat(q.W(), q.X(), q.Y(), q.Z()),
          ccdw::vec3(v.X(), v.Y(), v.Z()));
  }
};

///////////////////////////////////////////////
gazebo::msgs::Vector3d convertToMsg(const ccdw::vec3& from)
{
  gazebo::msgs::Vector3d result;
  result.set_x(from.x());
  result.set_y(from.y());
  result.set_z(from.z());

  return result;
}

///////////////////////////////////////////////
struct CollisionTest
{
  std::string name;
  std::vector<CollisionObject> fromObjects;
  std::vector<CollisionObject> toObjects;
  double range;

  CollisionTest() = default;
  CollisionTest(CollisionTest&&) = default;

  mutable gazebo::transport::PublisherPtr publisher;

  void runTest(const ccdw::Checker& checker) const
  {
    if(firstRun)
    {
      firstRun = false;
      contact_cache.set_collision1(name + "/from");
      contact_cache.set_collision2(name + "/to");
      contact_cache.add_position();
      contact_cache.add_position();
      contact_cache.add_depth(std::numeric_limits<double>::infinity());
    }

    for(const CollisionObject& from : fromObjects)
      from.update_tf();

    for(const CollisionObject& to : toObjects)
      to.update_tf();

    ccdw::Report minReport;
    minReport.distance = std::numeric_limits<ccd_real_t>::infinity();

    for(const CollisionObject& from : fromObjects)
    {
      for(const CollisionObject& to : toObjects)
      {
        ccdw::Report report;
        checker.separate(report, &from.tf, &to.tf, range);

        if(report.distance < minReport.distance)
          minReport = report;
      }
    }

    *contact_cache.mutable_position(0) = convertToMsg(minReport.pos1);
    *contact_cache.mutable_position(1) = convertToMsg(minReport.pos2);
    *contact_cache.mutable_depth()->Mutable(0) = minReport.distance;

    publisher->Publish(contact_cache);
  }

private:
  mutable gazebo::msgs::Contact contact_cache;
  mutable bool firstRun = true;
};

///////////////////////////////////////////////
void checkDilation(
    std::unique_ptr<ccdw::Convex>& convex,
    const sdf::ElementPtr& element)
{
  if(sdf::ElementPtr dilationElem = element->GetElement("dilation"))
  {
    const double dilation = dilationElem->Get<double>();

    if(dilation < 0.0)
    {
      gzerr << "The element <dilation> of " << PluginName << " does not "
            << "support negative values (" << dilation << ")!\n";
      return;
    }

    convex = std::unique_ptr<ccdw::Convex>(
          ccdw::dilate(convex.get(), dilation));
  }
}

///////////////////////////////////////////////
std::unique_ptr<ccdw::Convex> generateBox(
    const sdf::ElementPtr& boxElem)
{
  ignition::math::Vector3d dims(1.0, 1.0, 1.0);

  if(sdf::ElementPtr sizeElem = boxElem->GetElement("size"))
  {
    dims = sizeElem->Get<ignition::math::Vector3d>();
  }
  else
  {
    gzerr << "Missing <size> element inside of <" << BoxElementName
          << "> element of " << PluginName << "\n";
  }

  std::unique_ptr<ccdw::Convex> box(
        new ccdw::Box(ccdw::vec3(dims[0], dims[1], dims[2])));

  checkDilation(box, boxElem);

  return box;
}

///////////////////////////////////////////////
std::unique_ptr<ccdw::Convex> generateCylinder(
    const sdf::ElementPtr& cylElem)
{
  double radius = 1.0;
  double length = 1.0;

  if(sdf::ElementPtr radiusElem = cylElem->GetElement("radius"))
  {
    radius = radiusElem->Get<double>();
  }
  else
  {
    gzerr << "Missing <radius> element inside of <" << CylinderElementName
          << "> element of " << PluginName << "\n";
  }

  if(sdf::ElementPtr lengthElem = cylElem->GetElement("length"))
  {
    length = lengthElem->Get<double>();
  }
  else
  {
    gzerr << "Missing <length> element inside of <" << CylinderElementName
          << "> element of " << PluginName << "\n";
  }

  std::unique_ptr<ccdw::Convex> cylinder(
        new ccdw::Cylinder(length, radius));

  checkDilation(cylinder, cylElem);

  return cylinder;
}

///////////////////////////////////////////////
std::unique_ptr<ccdw::Convex> generateSphere(
    const sdf::ElementPtr& sphereElem)
{
  double radius = 1.0;

  if(sdf::ElementPtr radiusElem = sphereElem->GetElement("radius"))
  {
    radius = radiusElem->Get<double>();
  }
  else
  {
    gzerr << "Missing <radius> element inside of <" << SphereElementName
          << "> element of " << PluginName << "\n";
  }

  return std::unique_ptr<ccdw::Convex>(ccdw::sphere(radius));
}

///////////////////////////////////////////////
std::unique_ptr<ccdw::Convex> generateCapsule(
    const sdf::ElementPtr& capElem)
{
  double radius = 1.0;
  double length = 1.0;

  if(sdf::ElementPtr radiusElem = capElem->GetElement("radius"))
  {
    radius = radiusElem->Get<double>();
  }
  else
  {
    gzerr << "Missing <radius> element inside of <" << CapsuleElementName
          << "> element of " << PluginName << "\n";
  }

  if(sdf::ElementPtr lengthElem = capElem->GetElement("length"))
  {
    length = lengthElem->Get<double>();
  }
  else
  {
    gzerr << "Missing <length> element inside of <" << CapsuleElementName
          << "> element of " << PluginName << "\n";
  }

  return std::unique_ptr<ccdw::Convex>(ccdw::capsule(length, radius));
}

///////////////////////////////////////////////
bool checkForInvalidShape(const sdf::ElementPtr& geomElem,
                          const std::string& shape)
{
  if(geomElem->GetElement(shape))
  {
    gzerr << "collision type <" << shape << "> is not supported by "
          << PluginName << "\n";

    return true;
  }

  return false;
}

///////////////////////////////////////////////
void appendCollisionObject(
    std::vector<CollisionObject>& objects,
    const LinkPtr& link,
    const sdf::ElementPtr& collisionElem)
{
  CollisionObject object;
  object.link = link;

  sdf::ElementPtr geomElem = collisionElem->GetElement(GeometryElementName);
  if(!geomElem)
  {
    gzerr << "Missing <" << GeometryElementName << "> element inside of <"
          << CollisionElementName << "> element of " << PluginName << "\n";
    return;
  }

  // Return if the user has specified the <empty> element
  if(geomElem->GetElement(EmptyElementName))
    return;

  if(sdf::ElementPtr boxElem = geomElem->GetElement(BoxElementName))
  {
    object.convex = generateBox(boxElem);
  }
  else if(sdf::ElementPtr cylElem = geomElem->GetElement(CylinderElementName))
  {
    object.convex = generateCylinder(cylElem);
  }
  else if(sdf::ElementPtr sphereElem = geomElem->GetElement(SphereElementName))
  {
    object.convex = generateSphere(sphereElem);
  }
  else if(sdf::ElementPtr capElem = geomElem->GetElement(CapsuleElementName))
  {
    object.convex = generateCapsule(capElem);
  }
  else
  {
    bool hasInvalidShape = false;
    for(const std::string& shape :
          { "heightmap", "image", "mesh", "plane", "polyline" })
      hasInvalidShape |= checkForInvalidShape(geomElem, shape);

    if(!hasInvalidShape)
    {
      gzwarn << "Could not find any shape type for <" << GeometryElementName
             << "> element of " << PluginName << ". Please specify a <box> or "
             << "<cylinder> element!\n";
    }

    return;
  }

  sdf::ElementPtr poseElem = collisionElem->GetElement(PoseElementName);
  if(poseElem)
  {
    object.offset = poseElem->Get<gazebo::math::Pose>().Ign();
  }

  objects.emplace_back(std::move(object));
}

///////////////////////////////////////////////
void traverseLink(
    std::vector<CollisionObject>& objects,
    const ModelPtr& model,
    const sdf::ElementPtr& linkElem)
{
  const std::string& linkName = linkElem->Get<std::string>("name");
  const LinkPtr& link = model->GetLink(linkName);
  if(!link)
  {
    gzerr << "Could not find a link named [" << linkName << "] in the model ["
          << model->GetName() << "], requested by " << PluginName << ", in the "
          << "loaded world!\n";
    return;
  }

  sdf::ElementPtr collisionElem = linkElem->GetFirstElement();
  while(collisionElem)
  {
    if(collisionElem->GetName() != CollisionElementName)
      continue;

    appendCollisionObject(objects, link, collisionElem);
  }
}

///////////////////////////////////////////////
void traverseModel(
    std::vector<CollisionObject>& objects,
    const gazebo::physics::WorldPtr& world,
    const sdf::ElementPtr& modelElem)
{
  const std::string& modelName = modelElem->Get<std::string>("name");
  const ModelPtr& model = world->GetModel(modelName);
  if(!model)
  {
    gzerr << "Could not find a model named [" << modelName << "], requested by "
          << PluginName << ", in the loaded world!\n";
    return;
  }

  sdf::ElementPtr linkElem = modelElem->GetFirstElement();
  while(linkElem)
  {
    if(linkElem->GetName() != LinkElementName)
      continue;

    traverseLink(objects, model, linkElem);
  }

  if(objects.empty())
  {
    gzwarn << "No collision objects found for [" << modelName << "] element\n";
  }
}

///////////////////////////////////////////////
std::vector<CollisionObject> generateCollisionObjects(
    const gazebo::physics::WorldPtr& world,
    const sdf::ElementPtr& pairElem,
    const std::string& fromOrTo)
{
  std::vector<CollisionObject> objects;

  sdf::ElementPtr fromOrToElem = pairElem->GetElement(fromOrTo);
  if(!fromOrToElem)
  {
    gzerr << "<" << PairElementName << "> element of " << PluginName
          << " is missing a <" << fromOrTo << "> element!\n";
    return {};
  }

  sdf::ElementPtr modelElem = fromOrToElem->GetFirstElement();
  while(modelElem)
  {
    if(modelElem->GetName() != ModelElementName)
      continue;

    traverseModel(objects, world, modelElem);
  }

  if(objects.empty())
  {
    gzwarn << "<" << fromOrTo << "> element of " << PluginName
           << " did not provide any collision geometries!\n";
  }

  return objects;
}

CollisionTest generateCollisionTest(
    std::vector<CollisionTest>& collisionTests,
    const gazebo::physics::WorldPtr& world,
    const sdf::ElementPtr& pairElem,
    const std::size_t count)
{
  CollisionTest test;
  test.fromObjects = generateCollisionObjects(world, pairElem, FromElementName);
  test.toObjects = generateCollisionObjects(world, pairElem, ToElementName);

  if(sdf::ParamPtr nameParam = pairElem->GetAttribute("name"))
  {
    nameParam->Get<std::string>(test.name);
  }
  else
  {
    test.name = "Test" + std::to_string(count);
  }

  test.range = 10.0;
  if(sdf::ElementPtr rangeElem = pairElem->GetElement(RangeElementName))
  {
    test.range = rangeElem->Get<double>();
  }

  collisionTests.emplace_back(std::move(test));
}


///////////////////////////////////////////////
class Plugin : public gazebo::WorldPlugin
{
public:

  Plugin()
    : computeResults(true)
  {
    // Use Load() and Init() to initialize everything else
  }

  void computeMinimumDistances()
  {
    if(!computeResults)
      return;

    for(const CollisionTest& test : collisionTests)
      test.runTest(checker);
  }

  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    const sdf::ElementPtr pairElem = _sdf->GetFirstElement();

    std::size_t count = 0;
    while(pairElem)
    {
      if (pairElem->GetName() != PairElementName)
      {
        gzwarn << "Unknown element type given to " << PluginName << ":"
               << pairElem->GetName() << "\n -- Expected a <pair> element\n";
        continue;
      }

      collisionTests.emplace_back(
            generateCollisionTest(collisionTests, _world, pairElem, count));

      pairElem->GetNextElement(PairElementName);
    }

    updateConnection = gazebo::event::Events::ConnectWorldUpdateEnd(
          [this](){ computeMinimumDistances(); });

    node = gazebo::transport::NodePtr(new gazebo::transport::Node);
  }

  void toggle(const boost::shared_ptr<const gazebo::msgs::Int>& request)
  {
    if(request->data() != 0)
      computeResults = true;
    else
      computeResults = false;
  }

  void Init() override
  {
    node->TryInit(gazebo::common::Time::Maximum());

    for(CollisionTest& test : collisionTests)
    {
      test.publisher =
          node->Advertise<gazebo::msgs::Contacts>(PluginName + "/" + test.name);
    }

    node->Subscribe<gazebo::msgs::Int, Plugin>(
          PluginName + "/toggle", &Plugin::toggle, this);
  }

private:

  std::vector<CollisionTest> collisionTests;

  ccdw::Checker checker;

  gazebo::transport::NodePtr node;

  std::atomic_bool computeResults;

  gazebo::event::ConnectionPtr updateConnection;
};

GZ_REGISTER_WORLD_PLUGIN(Plugin)

} // namespace minimum_distance_plugin

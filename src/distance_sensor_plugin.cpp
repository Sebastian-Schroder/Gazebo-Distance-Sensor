#include <distance_sensor_plugin.hpp>
#include <helper.hpp>

#include <gz/plugin/Register.hh>
#include <gz/sim/components.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Util.hh>



GZ_ADD_PLUGIN(
    monke_plugins::DistancePlugin,
    gz::sim::System,
    // monke_plugins::DistancePlugin::ISystemPreUpdate,
    monke_plugins::DistancePlugin::ISystemPostUpdate,
    monke_plugins::DistancePlugin::ISystemConfigure
)

using namespace monke_plugins;

DistancePlugin::DistancePlugin():
dataPtr(new DistancePluginPrivate){

}

DistancePlugin::~DistancePlugin(){

}

void DistancePlugin::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/){

        gzmsg << "loading distance sensor plugin\n";

        if(!(read_sdf<std::string>(_sdf,"parent"))){
            return;
        }
        this->dataPtr->connected_name= *read_sdf<std::string>(_sdf,"parent");


        if(!(read_sdf<std::string>(_sdf,"output_topic","/distance_sensor_plugin/sensor"))){
            return;
        }
        this->dataPtr->sensor_output_topic= *read_sdf<std::string>(_sdf,"output_topic","/distance_sensor_plugin/sensor");
        
        this->dataPtr->publisher = std::make_unique<gz::transport::Node::Publisher>(this->dataPtr->transport_node->Advertise<gz::msgs::Contacts>(this->dataPtr->sensor_output_topic));

        if(!(read_sdf<std::string>(_sdf,"enable_topic","/distance_sensor_plugin/enable"))){
            return;
        }
        this->dataPtr->enable_input_topic= *read_sdf<std::string>(_sdf,"enable_topic","/distance_sensor_plugin/enable");
        std::function<void(const gz::msgs::Boolean &)> callback = [this](gz::msgs::Boolean msg){this->dataPtr->PluginActive=msg.data();};
        this->dataPtr->transport_node->Subscribe<gz::msgs::Boolean>(this->dataPtr->enable_input_topic,callback);

        if(!(read_sdf<std::string>(_sdf,"entity"))){
            return;
        }
        if(read_sdf<std::string>(_sdf,"entity_link")){
            return;
        }
        this->dataPtr->distance.push_back(std::pair<std::string,std::string>(*read_sdf<std::string>(_sdf,"entity"),*read_sdf<std::string>(_sdf,"entity_link")));
        gzwarn << " distance size [" << this->dataPtr->distance.size() << "]\n\n";
        if(!(read_sdf<double>(_sdf,"min_dist"))){
            return;
        }
        this->dataPtr->min_dist = *read_sdf<double>(_sdf,"min_dist");

        

    }

void DistancePlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
const gz::sim::EntityComponentManager &_ecm){

    if(_info.paused){return;} //if sim is paused.
    if(!this->dataPtr->PluginActive){return;}

    this->dataPtr->connected_entity = *entitiesFromScopedName(this->dataPtr->connected_name,_ecm).begin();
    gz::sim::Model base_model(this->dataPtr->connected_entity);

    auto link = gz::sim::Link(base_model.LinkByName(_ecm,this->dataPtr->connected_link));
    auto pose = *link.WorldPose(_ecm);
    gzmsg << "number of elements found [" << this->dataPtr->distance.size() << "]\n";


    for(auto name: this->dataPtr->distance){
        auto object = gz::sim::Model(*entitiesFromScopedName(name.first,_ecm).begin());
        auto link2 = gz::sim::Link(object.LinkByName(_ecm,name.second));
        auto pose2 = *link.WorldPose(_ecm);

        double dist = sqrt((pose.X()-pose2.X())+(pose.Y()-pose2.Y())+(pose.Z()-pose2.Z()));

        gzmsg << "distance to entity[" << name.first <<"] : " << dist;

        if(dist <  this->dataPtr->min_dist){
            gz::msgs::Contacts msg;
            msg.add_contact();
            gz::msgs::Entity e1,e2;
            e1.set_id(base_model.Entity());
            e2.set_id(object.Entity());



            auto temp = msg.mutable_contact(0);
            temp->set_allocated_collision1(&e1);
            temp->set_allocated_collision2(&e2);


            
            if(msg.contact(0).collision1().id()!=e1.id()||msg.contact(0).collision2().id()!=e2.id()){
                gzerr << "contact message has inconsisted entity values " << 
                std::to_string(msg.contact(0).collision1().id()) << " vs " << std::to_string(e1.id()) << " | " <<
                std::to_string(msg.contact(0).collision2().id()) << " vs " << std::to_string(e2.id()); 
            }
            gzwarn << "entity within range, sending message";


            this->dataPtr->publisher->Publish(msg);

        }


    }
    // gzmsg << "JointCreator::PostUpdate - " << _info.simTime.count() << std::endl;
    
}
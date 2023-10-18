#include <gz/sim/System.hh>
#include <memory.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/contacts.pb.h>
#include <gz/transport/Node.hh>
#include <utility>
namespace monke_plugins
{ 
//https://github.com/osrf/mbzirc/blob/ec66f35c1fc925f18565c03251bd5f12f5b6b4e3/mbzirc_ign/src/SuctionGripper.cc#L278-L283
    class DistancePluginPrivate {

    public:
    DistancePluginPrivate():
    transport_node(std::make_unique<gz::transport::Node>())
    {

    }   
    std::vector<std::pair<std::string,std::string>> distance ; //name - linkname


        gz::sim::Entity connected_entity{gz::sim::kNullEntity};

        std::string connected_name;
        std::string connected_link;
        // std::vector<std::string> entity_names;


        bool PluginActive = true; //whether to turn joint creation on or off. turning off while joint is created causes joint to be destroyed

        
        double min_dist = 0;

        std::string sensor_output_topic;

        std::string enable_input_topic;


        std::unique_ptr<gz::transport::Node> transport_node;

        std::unique_ptr<gz::transport::Node::Publisher> publisher;




};


    class DistancePlugin: public gz::sim::System, 
        public gz::sim::ISystemPostUpdate,
        // public gz::sim::ISystemPreUpdate, 
        // public gz::sim::ISystemUpdate,
        public gz::sim::ISystemConfigure
    {
        public:
            DistancePlugin();
            ~DistancePlugin() override;
            
            void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) override;
            // void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;
            // void Update(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;
            
            
            void Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> & _sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &/*_eventMgr*/);
        


            
        private:
            std::unique_ptr<DistancePluginPrivate> dataPtr;

    };

}


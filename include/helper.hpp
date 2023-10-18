#include <gz/sim/System.hh>
#include <memory.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/contacts.pb.h>
#include <gz/transport/Node.hh>
#include <optional>

template<typename T>
std::optional<T> read_sdf(const std::shared_ptr<const sdf::Element> & _sdf, std::string Element, std::optional<T> default_value = std::nullopt){
    if(_sdf->HasElement(Element)){
        return(std::optional<T>(_sdf->Get<T>(Element)));
    }
    if(default_value.has_value()){
        return(default_value);
    }
    gzerr << "sdf element [" << Element << "] not found";
    return(std::nullopt);

}
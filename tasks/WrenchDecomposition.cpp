/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WrenchDecomposition.hpp"

using namespace ctrl_lib;

WrenchDecomposition::WrenchDecomposition(std::string const& name)
    : WrenchDecompositionBase(name){
}

WrenchDecomposition::WrenchDecomposition(std::string const& name, RTT::ExecutionEngine* engine)
    : WrenchDecompositionBase(name, engine){
}

WrenchDecomposition::~WrenchDecomposition(){
}

bool WrenchDecomposition::configureHook(){
    if (! WrenchDecompositionBase::configureHook())
        return false;

    std::vector<std::string> wrench_names = _wrench_names.get();
    
    // Create dynamic output ports
    for(auto w : wrench_names){
        if(wrench_interface_map.count(w) == 0) // Don't recreate interfaces
            wrench_interface_map[w] = std::make_shared<WrenchInterface>("wrench_" + w, this);
    }
    
    // Remove ports which are not required anymore
    for(auto it : wrench_interface_map){
        if(std::find(wrenches_names.begin(), wrenches_names.end(), it->first) == wrenches_names.end())
            wrench_interface_map.erase(it->first);

    return true;
}

bool WrenchDecomposition::startHook(){
    if (! WrenchDecompositionBase::startHook())
        return false;
    return true;
}

void WrenchDecomposition::updateHook(){
    WrenchDecompositionBase::updateHook();

    if(_wrenches.readNewest(wrenches_in) == RTT::NewData){
        for(auto n : wrenches_in.names){
            // Check correctness of wrench names
            if(wrench_interface_map.count(n) == 0)
                throw std::runtime_error("Name " + n + " has not been configured! Check your config of wrenches_names!");

            _wrench_interface_map[n]->writeSample(wrenches_in.getElementByName(n), wrenches_in.time);
        }        
    }
}

void WrenchDecomposition::errorHook(){
    WrenchDecompositionBase::errorHook();
}

void WrenchDecomposition::stopHook(){
    WrenchDecompositionBase::stopHook();
}

void WrenchDecomposition::cleanupHook(){
    WrenchDecompositionBase::cleanupHook();
}

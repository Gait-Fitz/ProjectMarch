#include <march_hardware/PDOmap.h>

namespace march4cpp
{

PDOmap::PDOmap(){
    this->initAllObjects();
}

void PDOmap::addObject(std::string objectname){
    if (this->allObjects.count(objectname) != 1) {
        ROS_WARN("IMC object %s does not exist (yet), or multiple exist", objectname.c_str());
    }
    else {
        this->PDOObjects[objectname] = this->allObjects[objectname];
    }
}

std::map<std::string, int> PDOmap::map(int slaveIndex, enum dataDirection direction){
    // Sort PDOObjects map
    this->sortPDOObjects();
    int reg;
    int SMAddress;
    if(direction == dataDirection::miso){
        reg = 0x1A00;
        SMAddress = 0x1C13;
    }
    else if(direction == dataDirection::mosi){
        reg = 0x1600;
        SMAddress = 0x1C12;
    }
    else{
        ROS_ERROR("Invalid dataDirection argument");
    }
    // Clear SyncManager Object
    // sdo_bit8(slaveIndex, SMAddress, 0, 0);
    ROS_INFO("sdo write: slaveindex %i, reg 0x%X, subindex 0, value 0x0", slaveIndex, SMAddress);
    int startReg = reg;
    int lastFilledReg = reg;
    int sizeleft = 64;
    int counter = 0;
    int byteOffset = 0;
    while(this->sortedPDOObjects.size() > 0){
        // Check if register is still empty
        if(sizeleft == 64){
            // sdo_write(slaveIndex, reg, 0, 0);
            ROS_INFO("sdo write: slaveIndex %i, reg 0x%X, subindex 0, value 0x0", slaveIndex, reg);
        }
        // Get next object (from end, because sorted from small to large)
        std::pair<std::string, IMCObject> nextObject = this->sortedPDOObjects.back();
        this->sortedPDOObjects.pop_back();
        // Add next object to map
        counter++;
        // sdo_write(slaveIndex, reg, counter,
        // this->combineAddressLength(instruction.second.address, instruction.second.length));
        ROS_INFO("sdo write: slaveIndex %i, reg 0x%X, subindex %i, value 0x%X",
                slaveIndex, reg, counter, this->combineAddressLength(nextObject.second.address, nextObject.second.length));
        this->byteOffsets[nextObject.first] = byteOffset;
        byteOffset += nextObject.second.length/8;
        sizeleft -= nextObject.second.length;
        // Check if this was the last object
        if (this->sortedPDOObjects.size() == 0){
            // sdo_write(slaveIndex, reg, 0, counter);
            ROS_INFO("sdo write: slaveIndex %i, reg 0x%X, subindex 0, value 0x%X", slaveIndex, reg, counter);
            lastFilledReg = reg;
            reg++;
        }
        // else, check if register is full
        else if (sizeleft <= 0){
            // sdo_write(slaveIndex, reg, 0, counter);
            ROS_INFO("sdo write: slaveIndex %i, reg 0x%X, subindex 0, value 0x%X", slaveIndex, reg, counter);
            reg++;
            counter = 0;
            sizeleft = 64;
        }
    }
    // For the unused registers, set count to zero
    for(int i = reg; i < startReg + this->nrofRegs; i++){
        // sdo_write(slaveIndex, i, 0, 0);
        ROS_INFO("sdo write: slaveIndex %i, reg 0x%X, subindex 0, value 0x0", slaveIndex, i);
    }
    // For all filled registers, set data to Sync Manager object
    int count = 0;
    for(int i = startReg; i <= lastFilledReg; i++){
        count++;
        // sdo_bit16(slaveIndex, SMAddress, count, 0x1600);
        ROS_INFO("sdo write: slaveindex %i, reg 0x%X, subindex %i, value 0x%X", slaveIndex, SMAddress, count, i);
    }
    // sdo_bit8(slaveIndex, SMAddress, 0, count);
    ROS_INFO("sdo write: slaveindex %i, reg 0x%X, subindex 0, value 0x%X", slaveIndex, SMAddress, count);
    return this->byteOffsets;
}

void PDOmap::sortPDOObjects(){
    // Sort from small to large
    int totalbits = 0;
    for(int i = 0; i < (sizeof(this->objectSizes)/sizeof(this->objectSizes[0])); i++){
        std::map<std::string, IMCObject>::iterator j;
        for (j = this->PDOObjects.begin(); j != this->PDOObjects.end(); j++){
            if (j->second.length == this->objectSizes[i]){
                std::pair<std::string, IMCObject> nextObject;
                nextObject.first = j->first;
                nextObject.second = j->second;
                this->sortedPDOObjects.push_back(nextObject);
                totalbits += this->objectSizes[i];
            }
        }
    }
    if(totalbits > this->nrofRegs*this->bitsPerReg){
        ROS_ERROR("Too many objects in PDO Map (total bits %d, only %d allowed)",
                totalbits, this->nrofRegs*this->bitsPerReg);
    }
}

uint32_t PDOmap::combineAddressLength(uint16_t address, uint16_t length){
    uint32_t MSword = ((address & 0xFFFF) << 16);
    uint32_t LSword = (length & 0xFFFF);
    return (MSword | LSword);
}

void PDOmap::initAllObjects(){
    // Object(address, length);
    this->allObjects["StatusWord"] = IMCObject(0x6041, 16);
    this->allObjects["ActualPosition"] = IMCObject(0x6064, 32);
    this->allObjects["MotionErrorRegister"] = IMCObject(0x2000, 16);
    this->allObjects["DetailedErrorRegister"] = IMCObject(0x2002, 16);
    this->allObjects["DCLinkVoltage"] = IMCObject(0x2055, 16);
    this->allObjects["DriveTemperature"] = IMCObject(0x2058, 16);
    this->allObjects["ActualTorque"] = IMCObject(0x6077, 16);
    this->allObjects["CurrentLimit"] = IMCObject(0x207F, 16);
    this->allObjects["MotorPosition"] = IMCObject(0x2088, 32);
    // etc...
}

}
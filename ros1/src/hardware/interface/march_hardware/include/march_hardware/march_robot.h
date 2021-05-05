/*
 * Copyright (C) 2021 Bas Volkers, Thijs Raymakers
 * Copyright (C) 2020 Joris Weeda, Olav de Haas, Roel Vos
 * Copyright (C) 2019 Isha Dijks, Martijn van der Marel, Tim Buckers
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * Version 3 as published by the Free Software Foundation WITH
 * additional terms published by Project MARCH per section 7 of
 * the GNU General Public License Version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License INCLUDING the additional terms for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * AND the additional terms along with this program. If not,
 * see <https://projectmarch.nl/s/LICENSE> and
 * <https://projectmarch.nl/s/LICENSE-ADDITIONAL-TERMS>.
 */

// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_MARCH_ROBOT_H
#define MARCH_HARDWARE_MARCH_ROBOT_H
#include "march_hardware/ethercat/ethercat_master.h"
#include "march_hardware/joint.h"
#include "march_hardware/power/power_distribution_board.h"
#include "march_hardware/pressure_sole/pressure_sole.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <urdf/model.h>

namespace march {
class MarchRobot {
private:
    ::std::vector<Joint> jointList;
    urdf::Model urdf_;
    EthercatMaster ethercatMaster;
    std::unique_ptr<PowerDistributionBoard> pdb_;
    std::vector<PressureSole> pressureSoles;

public:
    using iterator = std::vector<Joint>::iterator;

    MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf,
        ::std::string if_name, int ecatCycleTime, int ecatSlaveTimeout);

    MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf,
        std::unique_ptr<PowerDistributionBoard> powerDistributionBoard,
        ::std::string if_name, int ecatCycleTime, int ecatSlaveTimeout);

    MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf,
        std::unique_ptr<PowerDistributionBoard> powerDistributionBoard,
        std::vector<PressureSole> pressureSoles, ::std::string if_name,
        int ecatCycleTime, int ecatSlaveTimeout);

    ~MarchRobot();

    /* Delete copy constructor/assignment since the ethercat master can not be
     * copied */
    MarchRobot(const MarchRobot&) = delete;
    MarchRobot& operator=(const MarchRobot&) = delete;

    /* Delete move constructor/assignment since atomic bool cannot be moved */
    MarchRobot(MarchRobot&&) = delete;
    MarchRobot& operator=(MarchRobot&&) = delete;

    void resetMotorControllers();

    void startEtherCAT(bool reset_imc);

    void stopEtherCAT();

    int getMaxSlaveIndex();

    bool hasValidSlaves();

    bool isEthercatOperational();

    std::exception_ptr getLastEthercatException() const noexcept;

    void waitForPdo();

    int getEthercatCycleTime() const;

    Joint& getJoint(const ::std::string& jointName);

    Joint& getJoint(size_t index);

    size_t size() const;

    iterator begin();
    iterator end();

    bool hasPowerDistributionboard() const;
    PowerDistributionBoard* getPowerDistributionBoard() const;

    bool hasPressureSoles() const;
    std::vector<PressureSole> getPressureSoles() const;

    const urdf::Model& getUrdf() const;

    /** @brief Override comparison operator */
    friend bool operator==(const MarchRobot& lhs, const MarchRobot& rhs)
    {
        if (lhs.jointList.size() != rhs.jointList.size()) {
            return false;
        }
        for (unsigned int i = 0; i < lhs.jointList.size(); i++) {
            const march::Joint& lhsJoint = lhs.jointList.at(i);
            const march::Joint& rhsJoint = rhs.jointList.at(i);
            if (lhsJoint != rhsJoint) {
                return false;
            }
        }
        return true;
    }

    /** @brief Override stream operator for clean printing */
    friend ::std::ostream& operator<<(
        std::ostream& os, const MarchRobot& marchRobot)
    {
        for (unsigned int i = 0; i < marchRobot.jointList.size(); i++) {
            os << marchRobot.jointList.at(i) << "\n";
        }
        return os;
    }
};
} // namespace march
#endif // MARCH_HARDWARE_MARCH_ROBOT_H

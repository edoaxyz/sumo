/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2012-2026 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    MSCFModel_RailETCS.h
/// @author  Edoardo Grassi
/// @date    Thu, 29 Jan 2026
///
// <description missing>
/****************************************************************************/
#pragma once
#include <config.h>

#include <utils/common/LinearApproxHelpers.h>
#include "MSCFModel_Rail.h"

class MSCFModel_RailETCS : public MSCFModel
{

public:
    /** @brief Constructor
     *  @param[in] vtype the type for which this model is built and also the parameter object to configure this model
     */
    MSCFModel_RailETCS(const MSVehicleType *vtype);

    int getModelID() const override;

    MSCFModel *duplicate(const MSVehicleType *vtype) const override;

    MSCFModel::VehicleVariables *createVehicleVariables() const override;

    virtual ~MSCFModel_RailETCS();

    double maxNextSpeed(double speed, const MSVehicle *const veh) const override;

    double minNextSpeed(double speed, const MSVehicle *const veh) const override;

    double minNextSpeedEmergency(double speed, const MSVehicle *const veh) const override;

    double getSafeSpeed(const MSVehicle *const veh, double startingSpeed, double gap, double targetSpeed = 0, double maxSpeed = INVALID_DOUBLE) const;

    double freeSpeed(const MSVehicle *const veh, double speed, double seen, double maxSpeed, bool onInsertion, CalcReason usage) const override;

    double followSpeed(const MSVehicle *const veh, double speed, double gap2pred, double predSpeed, double predMaxDecel, const MSVehicle *const pred, const CalcReason usage) const override;

    double stopSpeed(const MSVehicle *const veh, const double speed, double gap, double decel, CalcReason usage) const override;

    bool startupDelayStopped() const override
    {
        // startup delay in trains is dominated by inertia + brake delay and thus applies to any kind of stopping
        return true;
    }

private:
    class ETCSVehicleVariables : public MSCFModel::VehicleVariables
    {
    public:
        ETCSVehicleVariables(const MSCFModel_RailETCS &model) : speedMultiplier(model.myTrainParams.speedMultiplier)
        {
            slopeEnergy.reserve(model.myTrainParams.numDistances);
        }

        double getSlopeEnergy(const MSVehicle *const veh, double gap);

    private:
        void updateSlopeEnergy(const MSVehicle *const veh);

        std::string routeID;
        double distanceMultiplier;
        double speedMultiplier;
        double lastOdometer;
        std::vector<float> slopeEnergy;
    };

    struct TrainParams
    {
        // TODO: handle mass factor
        // double mf;

        int numSpeeds;
        int numDistances;
        double maxSpeed;                                 // m/s
        double weight;                                   // kg
        double length;                                   // m
        LinearApproxHelpers::LinearApproxMap traction;   // m/s -> kN
        LinearApproxHelpers::LinearApproxMap resistance; // m/s -> kN
        LinearApproxHelpers::LinearApproxMap braking;    // m/s -> kN
        LinearApproxHelpers::LinearApproxMap emgBraking; // m/s -> kN

        std::vector<float> safeSpeedDistanceMap;
        std::vector<unsigned short> safeDistanceSpeedMap;
        double distanceMultiplier;
        double speedMultiplier;

        double getResistance(double speed) const;
        double getTraction(double speed) const;
        double getBraking(double speed) const;
        double getEmgBraking(double speed) const;
    };

    TrainParams myTrainParams;

    void prepareSafeSpeedMap(int numSpeeds, int numDistances);
    double getSafeSpeedFast(const MSVehicle *const veh, double startingSpeed, double gap, double targetSpeed = 0, double maxSpeed = INVALID_DOUBLE) const;
    double getSafeSpeedAccurate(const MSVehicle *const veh, double startingSpeed, double gap, double targetSpeed = 0, double maxSpeed = INVALID_DOUBLE) const;
    double minNextSpeed(double speed, const MSVehicle *const veh, double slope) const;
};

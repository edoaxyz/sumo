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

    /** @brief Returns the model's name
     * @return The model's name
     * @see MSCFModel::getModelName
     */
    int getModelID() const override;

    /** @brief Duplicates the car-following model
     * @param[in] vtype The vehicle type this model belongs to (1:1)
     * @return A duplicate of this car-following model
     */
    MSCFModel *duplicate(const MSVehicleType *vtype) const override;

    /** @brief Creates vehicle-specific variables for this model used for fast speed calculations
     * @return A new instance of VehicleVariables
     */
    MSCFModel::VehicleVariables *createVehicleVariables() const override;

    virtual ~MSCFModel_RailETCS();

    double maxNextSpeed(double speed, const MSVehicle *const veh) const override;

    double minNextSpeed(double speed, const MSVehicle *const veh) const override;

    double minNextSpeedEmergency(double speed, const MSVehicle *const veh) const override;

    /**
     * @brief Returns the minimum deceleration to stop safely given the current speed and the train's characteristics
     * @param[in] veh The vehicle itself
     * @param[in] startingSpeed The vehicle's current speed
     * @return The minimum deceleration to stop safely
     */
    double getSafeDecel(const MSVehicle *const veh, double startingSpeed) const;

    double freeSpeed(const MSVehicle *const veh, double speed, double seen, double maxSpeed, bool onInsertion, CalcReason usage) const override;

    double followSpeed(const MSVehicle *const veh, double speed, double gap2pred, double predSpeed, double predMaxDecel, const MSVehicle *const pred, const CalcReason usage) const override;

    double stopSpeed(const MSVehicle *const veh, const double speed, double gap, double decel, CalcReason usage) const override;

    bool startupDelayStopped() const override
    {
        // startup delay in trains is dominated by inertia + brake delay and thus applies to any kind of stopping
        return true;
    }

private:
    struct TrainParams
    {
        // TODO: handle mass factor
        // double mf;

        int numSpeeds;                                   // number of speed entries in the maps
        int numDistances;                                // number of distance entries in the safe speed map
        double maxSpeed;                                 // m/s
        double weight;                                   // kg
        double length;                                   // m
        LinearApproxHelpers::LinearApproxMap traction;   // m/s -> kN
        LinearApproxHelpers::LinearApproxMap resistance; // m/s -> kN
        LinearApproxHelpers::LinearApproxMap braking;    // m/s -> kN
        LinearApproxHelpers::LinearApproxMap emgBraking; // m/s -> kN

        double speedMultiplier;                           // to convert from m/s to safeSpeedDistanceMap indices

        // @brief Gets the resistance force at a given speed
        double getResistance(double speed) const;
        // @brief Gets the traction force at a given speed
        double getTraction(double speed) const;
        // @brief Gets the service braking force at a given speed
        double getBraking(double speed) const;
        // @brief Gets the emergency braking force at a given speed
        double getEmgBraking(double speed) const;
    };

    /** @class ETCSVehicleVariables
     * @brief Container that holds the variables about the current energy
     * profile due to slope changes in the current route
     */
    class ETCSVehicleVariables : public MSCFModel::VehicleVariables
    {
    public:
        /* @brief Constructor
         * @param[in] model The car-following model this variable belongs to
         */
        ETCSVehicleVariables(const MSCFModel_RailETCS &model) : trainParams(model.myTrainParams) {}

        /*
         * @brief Gets the minimum gradient in the upcoming route section relevant for braking
         * The minimum gradient is computed as the minimum slope of all lanes in the upcoming
         * route section that can be reached within the given gap.
         * @param[in] veh The vehicle itself, used to get the current route and position
         * @param[in] gap The distance ahead to consider
         * @return The minimum gradient in degree
         */
        float getMinGradient(const MSVehicle *const veh, double gap);

        /*
         * @brief Gets the minimum deceleration in the upcoming route section relevant for braking.
         * The minimum deceleration is computed as the minimum deceleration of all lanes in the upcoming
         * route section that can be reached within the given gap, considering only service braking.
         *
         * @param[in] veh The vehicle itself, used to get the current route and position
         * @param[in] gap The distance ahead to consider
         * @return The minimum deceleration in m/s^2
         */
        float getMinDecl(const MSVehicle *const veh, double gap);

    private:
        /*
         * @brief Updates the minimum gradient and maximum speed limits by checking the upcoming lanes in the route and their slopes
         * @param[in] veh The vehicle itself, used to get the current route and position
         */
        void update(const MSVehicle *const veh);

        const TrainParams &trainParams;
        std::string routeID;       // route identifier of the last computed energy values
        double distanceMultiplier; // to convert from meters to minGradient indices
        double lastOdometer;       // last odometer value when the slope energy was computed
        std::unordered_map<unsigned short, std::unordered_map<unsigned short,
                                                              std::pair<signed char, float>>>
            map; // map of min gradients and min decelerations for distance start and end indices
    };

    TrainParams myTrainParams; // train dynamics parameters

    /** @brief Prepares the safe speed maps for fast lookup
     * @param[in] numSpeeds Number of speed entries
     * @param[in] numDistances Number of distance entries
     */
    void prepareSafeSpeedMap(int numSpeeds, int numDistances);
};

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
/// @file    MSCFModel_RailETCS.cpp
/// @author  Edoardo Grassi
/// @date    Thu, 29 Jan 2026
///
// <description missing>
/****************************************************************************/
#include <config.h>

#include <utils/common/MsgHandler.h>
#include <utils/common/StringUtils.h>
#include <utils/common/StringTokenizer.h>
#include <utils/geom/GeomHelper.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSLane.h>
#include "MSCFModel_RailETCS.h"

// ===========================================================================
// TrainParams method definitions
// ===========================================================================

double
MSCFModel_RailETCS::TrainParams::getResistance(double speed) const
{
    if (resistance.empty())
    {
        return 0;
    }
    return LinearApproxHelpers::getInterpolatedValue(resistance, speed) * 1000; // N
}

double
MSCFModel_RailETCS::TrainParams::getTraction(double speed) const
{
    if (traction.empty())
    {
        return 0;
    }
    return LinearApproxHelpers::getInterpolatedValue(traction, speed) * 1000; // N
}

double
MSCFModel_RailETCS::TrainParams::getBraking(double speed) const
{
    if (braking.empty())
    {
        return 0;
    }
    return LinearApproxHelpers::getInterpolatedValue(braking, speed) * 1000; // N
}

double
MSCFModel_RailETCS::TrainParams::getEmgBraking(double speed) const
{
    if (emgBraking.empty())
    {
        return 0;
    }
    return LinearApproxHelpers::getInterpolatedValue(emgBraking, speed) * 1000; // N
}

// ===========================================================================
// method definitions
// ===========================================================================

MSCFModel_RailETCS::MSCFModel_RailETCS(const MSVehicleType *vtype) : MSCFModel(vtype)
{
    if (vtype->wasSet(VTYPEPARS_LENGTH_SET))
    {
        myTrainParams.length = vtype->getLength();
    }
    if (vtype->wasSet(VTYPEPARS_MASS_SET))
    {
        myTrainParams.weight = vtype->getMass();
    }

    // init tabular curves
    myTrainParams.traction = vtype->getParameter().getCFProfile(SUMO_ATTR_TRACTION_TABLE, myTrainParams.traction);
    myTrainParams.resistance = vtype->getParameter().getCFProfile(SUMO_ATTR_RESISTANCE_TABLE, myTrainParams.resistance);
    myTrainParams.braking = vtype->getParameter().getCFProfile(SUMO_ATTR_BRAKING_TABLE, myTrainParams.braking);
    myTrainParams.emgBraking = vtype->getParameter().getCFProfile(SUMO_ATTR_EMG_BRAKING_TABLE, myTrainParams.emgBraking);
    auto speedTable = vtype->getParameter().getCFValueTable(SUMO_ATTR_SPEED_TABLE);
    myTrainParams.maxSpeed = *std::max_element(speedTable.begin(), speedTable.end());
    int distanceSamples = vtype->getParameter().getCFParam(SUMO_ATTR_NUM_SAMPLES, -1);

    if (myTrainParams.traction.empty())
    {
        throw ProcessError(TLF("tractionTable must be defined for vType '%'.", vtype->getID()));
    }
    if (myTrainParams.resistance.empty())
    {
        throw ProcessError(TLF("resistanceTable must be defined for vType '%'.", vtype->getID()));
    }
    if (myTrainParams.braking.empty())
    {
        throw ProcessError(TLF("brakingTable must be defined for vType '%'.", vtype->getID()));
    }
    if (myTrainParams.emgBraking.empty())
    {
        throw ProcessError(TLF("emgBrakingTable must be defined for vType '%'.", vtype->getID()));
    }

    if (distanceSamples != -1)
        prepareSafeSpeedMap(speedTable.size(), distanceSamples);
}

void MSCFModel_RailETCS::prepareSafeSpeedMap(int numSpeeds, int numDistances)
{
    myTrainParams.numSpeeds = numSpeeds;
    myTrainParams.numDistances = numDistances;
    double currentDistance = 0;
    myTrainParams.speedMultiplier = myTrainParams.maxSpeed / numSpeeds;
    myTrainParams.safeSpeedDistanceMap.reserve(numSpeeds);
    myTrainParams.safeDistanceSpeedMap.reserve(numDistances);
    double currentBraking = INT_MAX;

    for (int i = 0; i < numSpeeds; i++)
    {
        double currentSpeed = myTrainParams.speedMultiplier * i;
        double nextSpeed = myTrainParams.speedMultiplier * (i + 1);
        currentBraking = MIN2(myTrainParams.getResistance(nextSpeed) + myTrainParams.getBraking(nextSpeed), currentBraking);
        double partialDistance = (pow(nextSpeed, 2) - pow(currentSpeed, 2)) / (2. * currentBraking / myTrainParams.weight);
        currentDistance += partialDistance;
        myTrainParams.safeSpeedDistanceMap.push_back(currentDistance);
    }
    myTrainParams.distanceMultiplier = currentDistance / numDistances;

    int speedIndex = 0;
    for (int i = 0; i < numDistances; i++)
    {
        while (i * myTrainParams.distanceMultiplier > myTrainParams.safeSpeedDistanceMap[speedIndex] && speedIndex < myTrainParams.safeSpeedDistanceMap.size())
        {
            speedIndex++;
        }
        myTrainParams.safeDistanceSpeedMap.push_back(speedIndex);
    }
}

MSCFModel_RailETCS::~MSCFModel_RailETCS() {}

int MSCFModel_RailETCS::getModelID() const
{
    return SUMO_TAG_CF_RAIL_ETCS;
}

MSCFModel::VehicleVariables *MSCFModel_RailETCS::createVehicleVariables() const
{
    VehicleVariables *ret = new ETCSVehicleVariables(*this);
    return ret;
}

MSCFModel *
MSCFModel_RailETCS::duplicate(const MSVehicleType *vtype) const
{
    return new MSCFModel_RailETCS(vtype);
}

double MSCFModel_RailETCS::maxNextSpeed(double speed, const MSVehicle *const veh) const
{
    double resistance = myTrainParams.getResistance(speed);                          // N
    double gravity = myTrainParams.weight * GRAVITY * sin(DEG2RAD(veh->getSlope())); // N
    double traction = myTrainParams.getTraction(speed);                              // N

    double a = (traction - resistance - gravity) / myTrainParams.weight;

    double maxNextSpeed = speed + ACCEL2SPEED(a);
    return maxNextSpeed;
}

double MSCFModel_RailETCS::minNextSpeed(double speed, const MSVehicle *const veh) const
{
    return minNextSpeed(speed, veh, veh->getSlope());
}

double MSCFModel_RailETCS::minNextSpeed(double speed, const MSVehicle *const veh, double slope) const
{
    double resistance = myTrainParams.getResistance(speed);                // N
    double gravity = myTrainParams.weight * GRAVITY * sin(DEG2RAD(slope)); // N
    double braking = myTrainParams.getBraking(speed);                      // N

    const double a = (-braking - resistance - gravity) / myTrainParams.weight;
    const double minNextSpeed = speed + ACCEL2SPEED(a);
    if (MSGlobals::gSemiImplicitEulerUpdate)
    {
        return MAX2(minNextSpeed, 0.);
    }
    else
    {
        // NOTE: ballistic update allows for negative speeds to indicate a stop within the next timestep
        return minNextSpeed;
    }
}

double MSCFModel_RailETCS::minNextSpeedEmergency(double speed, const MSVehicle *const veh) const
{
    double resistance = myTrainParams.getResistance(speed);                          // N
    double gravity = myTrainParams.weight * GRAVITY * sin(DEG2RAD(veh->getSlope())); // N
    double emgBraking = myTrainParams.getEmgBraking(speed);

    const double a = (-emgBraking - resistance - gravity) / myTrainParams.weight;
    const double minNextSpeed = speed + ACCEL2SPEED(a);
    if (MSGlobals::gSemiImplicitEulerUpdate)
    {
        return MAX2(minNextSpeed, 0.);
    }
    else
    {
        // NOTE: ballistic update allows for negative speeds to indicate a stop within the next timestep
        return minNextSpeed;
    }
}

double MSCFModel_RailETCS::getSafeSpeed(const MSVehicle *const veh, double startingSpeed, double gap, double targetSpeed, double maxSpeed) const
{
    // return getSafeSpeedAccurate(veh, startingSpeed, gap, targetSpeed, maxSpeed);
    if (myTrainParams.safeDistanceSpeedMap.empty() || myTrainParams.safeSpeedDistanceMap.empty())
    {
        return getSafeSpeedAccurate(veh, startingSpeed, gap, targetSpeed, maxSpeed);
    }
    else
    {
        return getSafeSpeedFast(veh, startingSpeed, gap, targetSpeed, maxSpeed);
    }
}

double MSCFModel_RailETCS::getSafeSpeedFast(const MSVehicle *const veh, double startingSpeed, double gap, double targetSpeed, double maxSpeed) const
{
    // WIP
    ETCSVehicleVariables *vehVar = (ETCSVehicleVariables *)veh->getCarFollowVariables();
    double slopeSpeedStart = vehVar->getSlopeSpeed(veh, 0);
    int startSpeedIndex = ceil(MAX2(startingSpeed, 0.) / myTrainParams.speedMultiplier);
    int endSpeedIndex = floor(MAX2(startingSpeed + slopeSpeedStart, 0.) / myTrainParams.speedMultiplier);
    double additionalGap = (myTrainParams.safeSpeedDistanceMap[endSpeedIndex] - myTrainParams.safeSpeedDistanceMap[startSpeedIndex]);

    double speedGap = vehVar->getSlopeSpeed(veh, gap);

    int speedIndex = floor(MAX2(targetSpeed - vehVar->getSlopeSpeed(veh, gap), 0.) / myTrainParams.speedMultiplier);
    double targetDistance = myTrainParams.safeSpeedDistanceMap[speedIndex] + gap + additionalGap;
    int targetDistanceIndex = floor(targetDistance / myTrainParams.distanceMultiplier);
    double safeSpeed = myTrainParams.safeDistanceSpeedMap[targetDistanceIndex] * myTrainParams.speedMultiplier;
    // if (currentGravity > 0) safeSpeed = MAX2(safeSpeed-gravitySpeed, minNextSpeed(startingSpeed, veh));
    //  get the offset from minextspeed with gradient and without (safespeed is without gradient)

    if (maxSpeed == INVALID_DOUBLE)
    {
        maxSpeed = maxNextSpeed(startingSpeed, veh);
    }
    return MAX2(MIN2(maxSpeed, safeSpeed), minNextSpeedEmergency(startingSpeed, veh));
}

double MSCFModel_RailETCS::getSafeSpeedAccurate(const MSVehicle *const veh, double startingSpeed, double gap, double targetSpeed, double maxSpeed) const
{
    if (maxSpeed == INVALID_DOUBLE)
    {
        maxSpeed = maxNextSpeed(startingSpeed, veh);
    }
    std::vector<const MSLane *> lanes = veh->getUpcomingLanesUntil(gap);
    if (lanes.size() == 0)
    {
        lanes.push_back(veh->getLane());
    }
    int laneIndex = lanes.size() - 1;
    double posInLane = veh->getLanePosAfterDist(gap).second;
    if (posInLane == -1)
        return targetSpeed;
    double distance = 0.;
    double currentSpeed = targetSpeed;
    double currentGravity = myTrainParams.weight * GRAVITY * sin(DEG2RAD(lanes[laneIndex]->getShape().slopeDegreeAtOffset(posInLane)));
    while (distance < gap && laneIndex >= 0 && maxSpeed > currentSpeed)
    {
        double nextSpeed = currentSpeed + 1. / 3.6;
        double currentBraking = myTrainParams.getResistance(nextSpeed) + myTrainParams.getBraking(nextSpeed);
        double partialDistance = (pow(nextSpeed, 2) - pow(currentSpeed, 2)) / (2. * (currentBraking + currentGravity) / myTrainParams.weight);
        if (posInLane - partialDistance < 0)
        {
            currentSpeed += sqrt(2. * posInLane * (currentBraking + currentGravity) / myTrainParams.weight);
            if (laneIndex > 0)
            {
                currentGravity = myTrainParams.weight * GRAVITY * MIN2(sin(DEG2RAD(lanes[laneIndex]->getShape().slopeDegreeAtOffset(0))), sin(DEG2RAD(lanes[laneIndex - 1]->getShape().slopeDegreeAtOffset(0))));
                posInLane = lanes[laneIndex - 1]->getLength();
            }
            laneIndex--;
            distance += posInLane;
        }
        else
        {
            distance += partialDistance;
            if (distance < gap)
                currentSpeed += nextSpeed - currentSpeed;
            posInLane -= partialDistance;
        }
    }
    return MAX2(MIN2(maxSpeed, currentSpeed), minNextSpeedEmergency(startingSpeed, veh));
}

double MSCFModel_RailETCS::freeSpeed(const MSVehicle *const veh, double speed, double dist, double targetSpeed,
                                     const bool onInsertion, const CalcReason usage) const
{
    if (MSGlobals::gSemiImplicitEulerUpdate)
    {
        return getSafeSpeed(veh, speed, dist, targetSpeed);
    }
    WRITE_ERROR(TL("Anything else than semi implicit euler update is not yet implemented. Exiting!"));
    throw ProcessError();
}

double MSCFModel_RailETCS::followSpeed(const MSVehicle *const veh, double speed, double gap2pred, double predSpeed, double /*predMaxDecel*/, const MSVehicle *const /*pred = 0*/, const CalcReason /*usage = CalcReason::CURRENT*/) const
{
    if (MSGlobals::gSemiImplicitEulerUpdate)
    {
        return getSafeSpeed(veh, speed, gap2pred, predSpeed);
    }
    WRITE_ERROR(TL("Anything else than semi implicit euler update is not yet implemented. Exiting!"));
    throw ProcessError();
}

double MSCFModel_RailETCS::stopSpeed(const MSVehicle *const veh, const double speed, double gap, double /*decel*/, const CalcReason usage) const
{
    if (MSGlobals::gSemiImplicitEulerUpdate)
    {
        return getSafeSpeed(veh, speed, gap);
    }
    WRITE_ERROR(TL("Anything else than semi implicit euler update is not yet implemented. Exiting!"));
    throw ProcessError();
}

double MSCFModel_RailETCS::ETCSVehicleVariables::getSlopeSpeed(const MSVehicle *const veh, double gap)
{
    if (veh->getRoute().getID() != routeID)
        updateSlopeSpeed(veh);
    double currentPosition = veh->getOdometer() - lastOdometer + veh->getLength();
    double currentUntil = currentPosition + gap;
    int startPos = floor(currentPosition / distanceMultiplier);
    int endPos = ceil(currentUntil / distanceMultiplier);
    return slopeSpeeds[endPos].second;
}

void MSCFModel_RailETCS::ETCSVehicleVariables::updateSlopeSpeed(const MSVehicle *const veh)
{
    routeID = veh->getRoute().getID();
    auto lanes = veh->getUpcomingLanesUntil(__DBL_MAX__);
    int laneIndex = 0;
    double lanePos = veh->getPositionOnLane();
    double nextLanesLength = -lanePos;
    double speed = 100;
    for (auto lane : lanes)
        nextLanesLength += lane->getLength();
    distanceMultiplier = nextLanesLength / slopeSpeeds.capacity();
    for (int i = 0; i < slopeSpeeds.capacity(); i++)
    {
        double nextPos = lanePos + distanceMultiplier;
        double nextIndex = laneIndex;
        double prevSpeed = speed;
        double minDegree = lanes[nextIndex]->getShape().slopeDegreeAtOffset(0);
        speed = sqrt(MAX2(2. * GRAVITY * sin(DEG2RAD(lanes[nextIndex]->getShape().slopeDegreeAtOffset(0))) * (MIN2(lanes[nextIndex]->getLength(), nextPos) - lanePos) + pow(speed, 2), 0.));
        while (nextPos > lanes[nextIndex]->getLength())
        {
            nextPos -= lanes[nextIndex]->getLength();
            nextIndex++;
            speed = sqrt(MAX2(2. * GRAVITY * sin(DEG2RAD(lanes[nextIndex]->getShape().slopeDegreeAtOffset(0))) * MIN2(lanes[nextIndex]->getLength(), nextPos) + pow(speed, 2), 0.));
            minDegree = MIN2(lanes[nextIndex]->getShape().slopeDegreeAtOffset(0), minDegree);
        }
        lanePos = nextPos;
        laneIndex = nextIndex;
        slopeSpeeds.push_back(std::make_pair(prevSpeed-100, sqrt(2. * GRAVITY * sin(DEG2RAD(minDegree)) * distanceMultiplier + pow(prevSpeed, 2))-100));
    }
    lastOdometer = veh->getOdometer();
}
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

    myTrainParams.numSpeeds = speedTable.size();
    myTrainParams.numDistances = distanceSamples;
    myTrainParams.speedMultiplier = myTrainParams.numSpeeds / myTrainParams.maxSpeed;
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
    double resistance = myTrainParams.getResistance(speed);                          // N
    double gravity = myTrainParams.weight * GRAVITY * sin(DEG2RAD(veh->getSlope())); // N
    double braking = myTrainParams.getBraking(speed);                                // N

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

double MSCFModel_RailETCS::getSafeDecel(const MSVehicle *const veh, double gap) const
{
    ETCSVehicleVariables *vehVars = dynamic_cast<ETCSVehicleVariables *>(veh->getCarFollowVariables());
    return vehVars->getMinDecl(veh, gap) + GRAVITY * sin(DEG2RAD(vehVars->getMinGradient(veh, gap)));
}

double MSCFModel_RailETCS::freeSpeed(const MSVehicle *const veh, double speed, double dist, double targetSpeed,
                                     const bool onInsertion, const CalcReason usage) const
{
    if (MSGlobals::gSemiImplicitEulerUpdate)
    {
        const double safeDecel = getSafeDecel(veh, dist);

        const double v = SPEED2DIST(targetSpeed);
        if (dist < v)
        {
            return targetSpeed;
        }
        const double b = ACCEL2DIST(safeDecel);
        const double y = MAX2(0.0, ((sqrt((b + 2.0 * v) * (b + 2.0 * v) + 8.0 * b * dist) - b) * 0.5 - v) / b);
        const double yFull = floor(y);
        const double exactGap = (yFull * yFull + yFull) * 0.5 * b + yFull * v + (y > yFull ? v : 0.0);
        const double fullSpeedGain = (yFull + (onInsertion ? 1. : 0.)) * ACCEL2SPEED(safeDecel);
        return DIST2SPEED(MAX2(0.0, dist - exactGap) / (yFull + 1)) + fullSpeedGain + targetSpeed;
    }
    else
    {
        WRITE_ERROR(TL("Anything else than semi implicit euler update is not yet implemented. Exiting!"));
        throw ProcessError();
    }
}

double MSCFModel_RailETCS::followSpeed(const MSVehicle *const veh, double speed, double gap2pred, double predSpeed, double /*predMaxDecel*/, const MSVehicle *const /*pred = 0*/, const CalcReason /*usage = CalcReason::CURRENT*/) const
{
    const double safeDecel = getSafeDecel(veh, gap2pred);

    const double vsafe = maximumSafeStopSpeed(gap2pred, safeDecel, speed, false, TS, false); // absolute breaking distance
    const double vmin = minNextSpeed(speed, veh);
    const double vmax = maxNextSpeed(speed, veh);

    if (MSGlobals::gSemiImplicitEulerUpdate)
    {
        return MIN2(vsafe, vmax);
    }
    else
    {
        // ballistic
        // XXX: the euler variant can break as strong as it wishes immediately! The ballistic cannot, refs. #2575.
        return MAX2(MIN2(vsafe, vmax), vmin);
    }
}

double MSCFModel_RailETCS::stopSpeed(const MSVehicle *const veh, const double speed, double gap, double /*decel*/, const CalcReason /*usage*/) const
{
    const double safeDecel = getSafeDecel(veh, gap);

    return MIN2(maximumSafeStopSpeed(gap, safeDecel, speed, false, TS, false), maxNextSpeed(speed, veh));
}

float MSCFModel_RailETCS::ETCSVehicleVariables::getMinGradient(const MSVehicle *const veh, double gap)
{
    if (veh->getRoute().getID() != routeID)
        update(veh);
    double currentPosition = veh->getOdometer() - lastOdometer; // position over precomputed route
    int index = MIN2((int)floor(currentPosition / distanceMultiplier), trainParams.numDistances - 2);
    int endIndex = MIN2(MAX2((int)ceil((currentPosition + gap) / distanceMultiplier), index + 1), trainParams.numDistances - 1);
    return map[index][endIndex].first / 10.;
}

float MSCFModel_RailETCS::ETCSVehicleVariables::getMinDecl(const MSVehicle *const veh, double gap)
{
    if (veh->getRoute().getID() != routeID)
        update(veh);
    double currentPosition = MAX2(veh->getOdometer() - lastOdometer, 0.); // position over precomputed route
    int index = MIN2((int)floor(currentPosition / distanceMultiplier), trainParams.numDistances - 2);
    int endIndex = MIN2(MAX2((int)ceil((currentPosition + gap) / distanceMultiplier), index + 1), trainParams.numDistances - 1);
    return map[index][endIndex].second / trainParams.weight;
}

void MSCFModel_RailETCS::ETCSVehicleVariables::update(const MSVehicle *const veh)
{
    routeID = veh->getRoute().getID();
    auto lanes = veh->getUpcomingLanesUntil(__DBL_MAX__);
    int laneIndex = 0;
    double lanePos = veh->getPositionOnLane();
    double nextLanesLength = -lanePos; // length from current position
    for (auto lane : lanes)
        nextLanesLength += lane->getLength();
    distanceMultiplier = nextLanesLength / trainParams.numDistances;
    for (int i = 0; i < trainParams.numDistances - 1; i++)
    {
        for (int j = i + 1; j < trainParams.numDistances; j++)
        {
            double myLaneIndex = laneIndex;
            double maxBrakingDistance = (j - i) * distanceMultiplier + lanePos - lanes[myLaneIndex]->getLength();
            double minDec = trainParams.getBraking(lanes[myLaneIndex]->getSpeedLimit());
            double minGradient = lanes[myLaneIndex]->getShape().slopeDegreeAtOffset(0);
            while (maxBrakingDistance > 0 && myLaneIndex < lanes.size() - 1)
            {
                myLaneIndex++;
                minDec = MIN2(minDec, trainParams.getBraking(lanes[myLaneIndex]->getSpeedLimit()));
                minGradient = MIN2(minGradient, lanes[myLaneIndex]->getShape().slopeDegreeAtOffset(0));
                maxBrakingDistance -= lanes[myLaneIndex]->getLength();
            }
            assert(floor(minDec) > 0);
            map[i][j] = std::make_pair<signed char, float>(floor(minGradient * 10.), floor(minDec));
        }

        double toAdd = distanceMultiplier;
        while (lanes[laneIndex]->getLength() - toAdd <= 0 && laneIndex < lanes.size())
        {
            toAdd -= lanes[laneIndex]->getLength();
            laneIndex++;
            lanePos = 0;
        }
        lanePos += toAdd;
    }
    lastOdometer = veh->getOdometer();
}
/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file navigator_mission.cpp
 *
 * Helper class to access missions
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author Simon Wilks <simon@uaventure.com>
 * @author Andreas Antener <andreas@uaventure.com>
 * @author Sander Smeets <sander@droneslab.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "mission.h"
#include "navigator.h"

#include <string.h>
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <lib/geo/geo.h>
#include <navigator/navigation.h>
#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/events.h>

using namespace time_literals;

Mission::Mission(Navigator *navigator) :
	MissionBase(navigator, 10)
{
}

void
Mission::on_inactive()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();

	if (_need_mission_save && _vehicle_status_sub.get().arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		save_mission_state();
	}

	MissionBase::on_inactive();
}

bool
Mission::isLanding()
{
	// vehicle is currently landing if
	//  mission valid, still flying, and in the landing portion of mission (past land start marker)
	bool on_landing_stage = get_land_start_available() && _mission.current_seq > get_land_start_index();

	// special case: if the land start index is at a LOITER_TO_ALT WP, then we're in the landing sequence already when the
	// distance to the WP is below the loiter radius + acceptance.
	if (_mission.current_seq == get_land_start_index() && _mission_item.nav_cmd == NAV_CMD_LOITER_TO_ALT) {
		const float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
					_navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

		// consider mission_item.loiter_radius invalid if NAN or 0, use default value in this case.
		const float mission_item_loiter_radius_abs = (PX4_ISFINITE(_mission_item.loiter_radius)
				&& fabsf(_mission_item.loiter_radius) > FLT_EPSILON) ? fabsf(_mission_item.loiter_radius) :
				_navigator->get_loiter_radius();

		on_landing_stage = d_current <= (_navigator->get_acceptance_radius() + mission_item_loiter_radius_abs);
	}

	return _navigator->get_mission_result()->valid && on_landing_stage;
}

bool
Mission::set_current_mission_index(uint16_t index)
{
	if (index == _mission.current_seq) {
		return true;
	}
	if (_navigator->get_mission_result()->valid && (index < _mission.count)) {
		if (goToItem(index, true) != PX4_OK) {
			// Keep the old mission index (it was not updated by the interface) and report back.
			return false;
		}

		_is_current_planned_mission_item_valid = true;
		// we start from the first item so can reset the cache
		if (_mission.current_seq == 0) {
			resetItemCache();
		}

		// update mission items if already in active mission
		if (isActive()) {
			// prevent following "previous - current" line
			_navigator->reset_triplets();
			update_mission();
			set_mission_items();
		}

		return true;
	}

	return false;
}

bool Mission::setNextMissionItem()
{
	return (goToNextItem(true) == PX4_OK);
}

bool
Mission::do_need_vertical_takeoff()
{
	if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

		float takeoff_alt = calculate_takeoff_altitude(&_mission_item);

		if (_land_detected_sub.get().landed) {
			/* force takeoff if landed (additional protection) */
			_need_takeoff = true;

		} else if (_global_pos_sub.get().alt > takeoff_alt - _navigator->get_altitude_acceptance_radius()) {
			/* if in-air and already above takeoff height, don't do takeoff */
			_need_takeoff = false;

		} else if (_global_pos_sub.get().alt <= takeoff_alt - _navigator->get_altitude_acceptance_radius()
			   && (_mission_item.nav_cmd == NAV_CMD_TAKEOFF
			       || _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF)) {
			/* if in-air but below takeoff height and we have a takeoff item */
			_need_takeoff = true;
		}

		/* check if current mission item is one that requires takeoff before */
		if (_need_takeoff && (
			    _mission_item.nav_cmd == NAV_CMD_TAKEOFF ||
			    _mission_item.nav_cmd == NAV_CMD_WAYPOINT ||
			    _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF ||
			    _mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
			    _mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED)) {

			_need_takeoff = false;
			return true;
		}
	}

	return false;
}

bool
Mission::do_need_move_to_land()
{
	if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && (_mission_item.nav_cmd == NAV_CMD_LAND || _mission_item.nav_cmd == NAV_CMD_VTOL_LAND)) {

		float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
				  _global_pos_sub.get().lat, _global_pos_sub.get().lon);

		return d_current > _navigator->get_acceptance_radius();
	}

	return false;
}

bool
Mission::do_need_move_to_takeoff()
{
	if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF) {

		float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
				  _global_pos_sub.get().lat, _global_pos_sub.get().lon);

		return d_current > _navigator->get_acceptance_radius();
	}

	return false;
}

float
Mission::calculate_takeoff_altitude(struct mission_item_s *mission_item)
{
	/* calculate takeoff altitude */
	float takeoff_alt = get_absolute_altitude_for_item(*mission_item);

	/* takeoff to at least MIS_TAKEOFF_ALT above home/ground, even if first waypoint is lower */
	if (_land_detected_sub.get().landed) {
		takeoff_alt = fmaxf(takeoff_alt, _global_pos_sub.get().alt + _navigator->get_takeoff_min_alt());

	} else {
		takeoff_alt = fmaxf(takeoff_alt, _navigator->get_home_position()->alt + _navigator->get_takeoff_min_alt());
	}

	return takeoff_alt;
}

void Mission::setActiveMissionItems()
{
	/* Get mission item that comes after current if available */
	constexpr static size_t max_num_next_items{2u};
	int32_t next_mission_items_index[max_num_next_items];
	size_t num_found_items;

	getNextPositionItems(_mission.current_seq, next_mission_items_index, num_found_items, max_num_next_items);

	mission_item_s next_mission_items[max_num_next_items];
	const dm_item_t dataman_id = static_cast<dm_item_t>(_mission.dataman_id);
	for (size_t i=0U; i < num_found_items; i++) {
		mission_item_s next_mission_item;
		bool success = _dataman_cache.loadWait(dataman_id, next_mission_items_index[i], reinterpret_cast<uint8_t *>(&next_mission_item), sizeof(next_mission_item));
		if (success) {
			next_mission_items[i] = next_mission_item;
		} else {
			num_found_items = i;
			break;
		}
	}

	/*********************************** handle mission item *********************************************/
	WorkItemType new_work_item_type = WorkItemType::WORK_ITEM_TYPE_DEFAULT;

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	const position_setpoint_s current_setpoint_copy = pos_sp_triplet->current;

	if (item_contains_position(_mission_item)) {
		/* force vtol land */
		if (_navigator->force_vtol() && _mission_item.nav_cmd == NAV_CMD_LAND) {
			_mission_item.nav_cmd = NAV_CMD_VTOL_LAND;
		}

		handleTakeoff(new_work_item_type, next_mission_items, num_found_items);

		handleLanding(new_work_item_type, next_mission_items, num_found_items);

		// TODO Precision land needs to be refactored: https://github.com/PX4/Firmware/issues/14320
		if (new_work_item_type != WorkItemType::WORK_ITEM_TYPE_PRECISION_LAND) {
			mission_apply_limitation(_mission_item);
			mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
		}

		// Only set the previous position item if the current one really changed
		if ((_work_item_type != WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND) &&
		    !position_setpoint_equal(&pos_sp_triplet->current, &current_setpoint_copy)) {
			pos_sp_triplet->previous = current_setpoint_copy;
		}

		// Allow a rotary wing vehicle to decelerate before reaching a wp with a hold time or a timeout
		// This is done by setting the position triplet's next position's valid flag to false,
		// which makes the FlightTask disregard the next position
		// TODO: Setting the next waypoint's validity flag to handle braking / correct waypoint behavior
		// seems hacky, handle this more properly.
		const bool brake_for_hold = _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					    && (get_time_inside(_mission_item) > FLT_EPSILON || item_has_timeout(_mission_item));

		if (_mission_item.autocontinue && !brake_for_hold) {
			/* try to process next mission item */
			if (num_found_items >= 1u) {
				/* got next mission item, update setpoint triplet */
				mission_item_to_position_setpoint(next_mission_items[0u], &pos_sp_triplet->next);

			} else {
				/* next mission item is not available */
				pos_sp_triplet->next.valid = false;
			}

		} else {
			/* vehicle will be paused on current waypoint, don't set next item */
			pos_sp_triplet->next.valid = false;
		}

	} else if (item_contains_gate(_mission_item)) {
		// The mission item is a gate, let's check if the next item in the list provides
		// a position to go towards.

		if (num_found_items > 0u) {
			// We have a position, convert it to the setpoint and update setpoint triplet
			mission_apply_limitation(next_mission_items[0u]);
			mission_item_to_position_setpoint(next_mission_items[0u], &pos_sp_triplet->current);
		}

		if (num_found_items >= 2u) {
			/* got next mission item, update setpoint triplet */
			mission_apply_limitation(next_mission_items[1u]);
			mission_item_to_position_setpoint(next_mission_items[1u], &pos_sp_triplet->next);

		} else {
			pos_sp_triplet->next.valid = false;
		}

	} else {
		handleVtolTransition(new_work_item_type, next_mission_items, num_found_items);
	}

	issue_command(_mission_item);

	/* set current work item type */
	_work_item_type = new_work_item_type;

	/* require takeoff after landing or idle */
	if (pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LAND
	    || pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {

		_need_takeoff = true;
	}

	_navigator->set_can_loiter_at_sp(false);
	reset_mission_item_reached();

	if (_mission_type == MissionType::MISSION_TYPE_MISSION) {
		set_mission_result();
	}

	publish_navigator_mission_item(); // for logging
	_navigator->set_position_setpoint_triplet_updated();
}

void Mission::handleTakeoff(WorkItemType &new_work_item_type, mission_item_s next_mission_items[],
			    size_t &num_found_items)
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* do takeoff before going to setpoint if needed and not already in takeoff */
	/* in fixed-wing this whole block will be ignored and a takeoff item is always propagated */
	if (do_need_vertical_takeoff() &&
	    _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) {

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_TAKEOFF;

		/* use current mission item as next position item */
		num_found_items = 1u;
		next_mission_items[0u] = _mission_item;
		next_mission_items[0u].nav_cmd = NAV_CMD_WAYPOINT;

		/* Update current mission item*/
		float takeoff_alt = calculate_takeoff_altitude(&_mission_item);

		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Takeoff to %.1f meters above home\t",
				 (double)(takeoff_alt - _navigator->get_home_position()->alt));
		events::send<float>(events::ID("mission_takeoff_to"), events::Log::Info,
				    "Takeoff to {1:.1m_v} above home", takeoff_alt - _navigator->get_home_position()->alt);

		_mission_item.nav_cmd = NAV_CMD_TAKEOFF;
		_mission_item.lat = _global_pos_sub.get().lat;
		_mission_item.lon = _global_pos_sub.get().lon;
		/* hold heading for takeoff items */
		_mission_item.yaw = _navigator->get_local_position()->heading;
		_mission_item.altitude = takeoff_alt;
		_mission_item.altitude_is_relative = false;
		_mission_item.autocontinue = true;
		_mission_item.time_inside = 0.0f;

	} else if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF
		   && _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT
		   && _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

		/* if there is no need to do a takeoff but we have a takeoff item, treat is as waypoint */
		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
		_mission_item.yaw = NAN;

	} else if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
		   && _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) {
		// if the vehicle is already in fixed wing mode then the current mission item
		// will be accepted immediately and the work items will be skipped
		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_TAKEOFF;


		/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
		_mission_item.yaw = NAN;
	}

	/* if we just did a normal takeoff navigate to the actual waypoint now */
	if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF &&
	    _work_item_type == WorkItemType::WORK_ITEM_TYPE_TAKEOFF) {

		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
		_mission_item.yaw = NAN;
	}

	/* if we just did a VTOL takeoff, prepare transition */
	if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF &&
	    _work_item_type == WorkItemType::WORK_ITEM_TYPE_TAKEOFF &&
	    _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
	    !_land_detected_sub.get().landed) {

		/* disable weathervane before front transition for allowing yaw to align */
		pos_sp_triplet->current.disable_weather_vane = true;

		/* set yaw setpoint to heading of VTOL_TAKEOFF wp against current position */
		_mission_item.yaw = get_bearing_to_next_waypoint(
					    _global_pos_sub.get().lat, _global_pos_sub.get().lon,
					    _mission_item.lat, _mission_item.lon);

		_mission_item.force_heading = true;

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_ALIGN;

		/* set position setpoint to current while aligning */
		_mission_item.lat = _global_pos_sub.get().lat;
		_mission_item.lon = _global_pos_sub.get().lon;
	}

	/* heading is aligned now, prepare transition */
	if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF &&
	    _work_item_type == WorkItemType::WORK_ITEM_TYPE_ALIGN &&
	    _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
	    !_land_detected_sub.get().landed) {

		/* re-enable weather vane again after alignment */
		pos_sp_triplet->current.disable_weather_vane = false;

		/* check if the vtol_takeoff waypoint is on top of us */
		if (do_need_move_to_takeoff()) {
			new_work_item_type = WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF;
		}

		set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
		_mission_item.yaw = _navigator->get_local_position()->heading;

		// keep current setpoints (FW position controller generates wp to track during transition)
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	}

	/* takeoff completed and transitioned, move to takeoff wp as fixed wing */
	if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
	    && _work_item_type == WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF) {

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_DEFAULT;
		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		_mission_item.autocontinue = true;
		_mission_item.time_inside = 0.0f;
	}
}

void Mission::handleLanding(WorkItemType &new_work_item_type, mission_item_s next_mission_items[],
			    size_t &num_found_items)
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* move to land wp as fixed wing */
	if (_mission_item.nav_cmd == NAV_CMD_VTOL_LAND
	    && (_work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT
		|| _work_item_type == WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF)
	    && !_land_detected_sub.get().landed) {

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND;

		/* use current mission item as next position item */
		num_found_items = 1u;
		next_mission_items[0u] = _mission_item;

		float altitude = _global_pos_sub.get().alt;

		if (pos_sp_triplet->current.valid && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
			altitude = pos_sp_triplet->current.alt;
		}

		_mission_item.altitude = altitude;
		_mission_item.altitude_is_relative = false;
		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		_mission_item.autocontinue = true;
		_mission_item.time_inside = 0.0f;
		_mission_item.vtol_back_transition = true;
	}

	/* transition to MC */
	if (_mission_item.nav_cmd == NAV_CMD_VTOL_LAND
	    && _work_item_type == WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND
	    && _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
	    && !_land_detected_sub.get().landed) {

		set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
		_mission_item.altitude = _global_pos_sub.get().alt;
		_mission_item.altitude_is_relative = false;

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION;

		// make previous setpoint invalid, such that there will be no prev-current line following
		// if the vehicle drifted off the path during back-transition it should just go straight to the landing point
		pos_sp_triplet->previous.valid = false;
	}

	/* move to landing waypoint before descent if necessary */
	if (do_need_move_to_land() &&
	    (_work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT ||
	     _work_item_type == WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION)) {

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND;

		/* use current mission item as next position item */
		num_found_items = 1u;
		next_mission_items[0u] = _mission_item;

		/*
			* Ignoring waypoint altitude:
			* Set altitude to the same as we have now to prevent descending too fast into
			* the ground. Actual landing will descend anyway until it touches down.
			* XXX: We might want to change that at some point if it is clear to the user
			* what the altitude means on this waypoint type.
			*/
		float altitude = _global_pos_sub.get().alt;

		if (pos_sp_triplet->current.valid
		    && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
			altitude = pos_sp_triplet->current.alt;
		}

		_mission_item.altitude = altitude;
		_mission_item.altitude_is_relative = false;
		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		_mission_item.autocontinue = true;

		// have to reset here because these field were used in set_vtol_transition_item
		_mission_item.time_inside = 0.f;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();

		// make previous setpoint invalid, such that there will be no prev-current line following.
		// if the vehicle drifted off the path during back-transition it should just go straight to the landing point
		pos_sp_triplet->previous.valid = false;

	} else if (_mission_item.nav_cmd == NAV_CMD_LAND && _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) {
		if (_mission_item.land_precision > 0 && _mission_item.land_precision < 3) {
			new_work_item_type = WorkItemType::WORK_ITEM_TYPE_PRECISION_LAND;

			if (_mission_item.land_precision == 1) {
				_navigator->get_precland()->set_mode(PrecLandMode::Opportunistic);

			} else { //_mission_item.land_precision == 2
				_navigator->get_precland()->set_mode(PrecLandMode::Required);
			}

			_navigator->get_precland()->on_activation();
		}
	}

	/* we just moved to the landing waypoint, now descend */
	if (_work_item_type == WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND) {

		if (_mission_item.land_precision > 0 && _mission_item.land_precision < 3) {
			new_work_item_type = WorkItemType::WORK_ITEM_TYPE_PRECISION_LAND;

			if (_mission_item.land_precision == 1) {
				_navigator->get_precland()->set_mode(PrecLandMode::Opportunistic);

			} else { //_mission_item.land_precision == 2
				_navigator->get_precland()->set_mode(PrecLandMode::Required);
			}

			_navigator->get_precland()->on_activation();
		}
	}

	/* ignore yaw for landing items */
	/* XXX: if specified heading for landing is desired we could add another step before the descent
		* that aligns the vehicle first */
	if (_mission_item.nav_cmd == NAV_CMD_LAND || _mission_item.nav_cmd == NAV_CMD_VTOL_LAND) {
		_mission_item.yaw = NAN;
	}
}

void Mission::handleVtolTransition(WorkItemType &new_work_item_type, mission_item_s next_mission_items[],
				   size_t &num_found_items)
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* turn towards next waypoint before MC to FW transition */
	if (_mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
	    && _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT
	    && new_work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT
	    && _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && !_land_detected_sub.get().landed
	    && (num_found_items > 0u)) {

		/* disable weathervane before front transition for allowing yaw to align */
		pos_sp_triplet->current.disable_weather_vane = true;

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_ALIGN;

		set_align_mission_item(&_mission_item, &next_mission_items[0u]);

		/* set position setpoint to target during the transition */
		mission_apply_limitation(_mission_item);
		mission_item_to_position_setpoint(next_mission_items[0u], &pos_sp_triplet->current);
	}

	/* yaw is aligned now */
	if (_work_item_type == WorkItemType::WORK_ITEM_TYPE_ALIGN &&
	    new_work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) {

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_DEFAULT;

		/* re-enable weather vane again after alignment */
		pos_sp_triplet->current.disable_weather_vane = false;

		pos_sp_triplet->previous = pos_sp_triplet->current;
		// keep current setpoints (FW position controller generates wp to track during transition)
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	}
}

void
Mission::save_mission_state()
{
	if (_vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		// Save only while disarmed, as this is a blocking operation
		_need_mission_save = true;
		return;
	}

	_need_mission_save = false;
	mission_s mission_state = {};

	/* read current state */
	bool success = _dataman_client.readSync(DM_KEY_MISSION_STATE, 0, reinterpret_cast<uint8_t *>(&mission_state),
						sizeof(mission_s));

	if (success) {
		/* data read successfully, check dataman ID and items count */
		if (mission_state.dataman_id == _mission.dataman_id && mission_state.count == _mission.count && mission_state.mission_update_counter && _mission.mission_update_counter) {
			/* navigator may modify only sequence, write modified state only if it changed */
			if (mission_state.current_seq != _mission.current_seq) {
				mission_state = _mission;

				success = _dataman_client.writeSync(DM_KEY_MISSION_STATE, 0, reinterpret_cast<uint8_t *>(&mission_state),
								    sizeof(mission_s));

				if (!success) {

					PX4_ERR("Can't save mission state");
				}
			}
		}

	} else {
		/* invalid data, this must not happen and indicates error in mission publisher */
		mission_state = _mission;

		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Invalid mission state.\t");
		/* EVENT
		 * @description No mission or storage failure
		 */
		events::send(events::ID("mission_invalid_mission_state"), events::Log::Error, "Invalid mission state");

		/* write modified state only if changed */
		success = _dataman_client.writeSync(DM_KEY_MISSION_STATE, 0, reinterpret_cast<uint8_t *>(&mission_state),
						    sizeof(mission_s));

		if (!success) {

			PX4_ERR("Can't save mission state");
		}
	}
}

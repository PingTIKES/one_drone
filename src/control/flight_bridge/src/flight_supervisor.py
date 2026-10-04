"""Pure flight-state gates; PX4 publishing stays in Px4Adapter.

Keeping the decisions pure makes fault and restart paths testable without
creating a second Offboard publisher or a cross-process control dependency.
"""


class FlightSupervisor:
    @staticmethod
    def takeoff_blockers(state, px4_position_valid, vio_stable, map_ready):
        missing = []
        if state != 'IDLE':
            missing.append(f'flight_state={state} (need IDLE)')
        if not px4_position_valid:
            missing.append('fresh PX4 local position')
        if not vio_stable:
            missing.append('stable VALID OpenVINS')
        if not map_ready:
            missing.append('rebuilt map after VIO reset')
        return missing

    @staticmethod
    def safety_hold(state, px4_position_valid, vio_ready, map_ready):
        if state in ('TAKEOFF', 'CRUISE') and not (px4_position_valid and vio_ready):
            return 'Position or OpenVINS invalid; waiting for stable automatic recovery'
        if state == 'CRUISE' and not map_ready:
            return 'Planner heartbeat or fused map stale; waiting for recovery'
        return None

    @staticmethod
    def may_auto_recover(state, enabled, map_ready, depth_valid,
                         resume_state, px4_position_valid, vio_stable,
                         status_fresh, armed, offboard, failsafe):
        return (state == 'HOLD' and enabled and map_ready and depth_valid and
                resume_state in ('TAKEOFF', 'CRUISE') and
                px4_position_valid and vio_stable and status_fresh and
                armed and offboard and not failsafe)

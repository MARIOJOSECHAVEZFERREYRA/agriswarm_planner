class DroneEnergyModel:
    def __init__(self, drone, liquid_density_kg_l=1.0):
        self.drone = drone
        self.liquid_density = liquid_density_kg_l
    def instant_mass(self, reagent_remaining_l):
        reagent_mass = reagent_remaining_l * self.liquid_density
        return self.drone.mass_empty_kg + self.drone.mass_battery_kg + reagent_mass
    def hover_power(self, reagent_remaining_l):
        m_empty_total = self.drone.mass_empty_kg + self.drone.mass_battery_kg
        m_current = self.instant_mass(reagent_remaining_l)
        mass_ratio = m_current / m_empty_total
        return self.drone.power_hover_empty_w * (mass_ratio ** 1.5)
    def cruise_power(self, reagent_remaining_l):
        return self.hover_power(reagent_remaining_l)
    def spray_power(self, reagent_remaining_l):
        return self.cruise_power(reagent_remaining_l) + self.drone.spray_pump_power_w
    def _straight_profile(self, distance_m, v_target_ms, a_acc_ms2, a_dec_ms2):
        d_acc_full = v_target_ms ** 2 / (2.0 * a_acc_ms2)
        d_dec_full = v_target_ms ** 2 / (2.0 * a_dec_ms2)
        if distance_m >= d_acc_full + d_dec_full:
            v_peak = v_target_ms
            t_acc = v_peak / a_acc_ms2
            t_dec = v_peak / a_dec_ms2
            d_cruise = distance_m - d_acc_full - d_dec_full
            t_cruise = d_cruise / v_peak
        else:
            v_peak = (2.0 * distance_m / (1.0 / a_acc_ms2 + 1.0 / a_dec_ms2)) ** 0.5
            t_acc = v_peak / a_acc_ms2
            t_dec = v_peak / a_dec_ms2
            t_cruise = 0.0
        return t_acc, t_cruise, t_dec, v_peak
    def energy_straight(self, distance_m, reagent_remaining_l):
        t_acc, t_cruise, t_dec, _ = self._straight_profile(
            distance_m,
            self.drone.speed_cruise_ms,
            self.drone.accel_horizontal_ms2,
            self.drone.decel_horizontal_ms2,
        )
        p_base = self.spray_power(reagent_remaining_l)
        energy_ws = (
            self.drone.power_accel_factor * p_base * t_acc
            + p_base * t_cruise
            + self.drone.power_decel_factor * p_base * t_dec
        )
        return energy_ws / 3600.0
    def energy_turn(self, angle_deg, reagent_remaining_l):
        effective_duration = self.drone.turn_duration_s * (angle_deg / 180.0)
        power_during_turn = self.hover_power(reagent_remaining_l) * self.drone.turn_power_factor
        energy_ws = power_during_turn * effective_duration
        return energy_ws / 3600.0
    def energy_transit(self, distance_m, reagent_remaining_l):
        t_acc, t_cruise, t_dec, _ = self._straight_profile(
            distance_m,
            self.drone.speed_max_ms,
            self.drone.accel_horizontal_ms2,
            self.drone.decel_horizontal_ms2,
        )
        p_base = self.cruise_power(reagent_remaining_l)
        energy_ws = (
            self.drone.power_accel_factor * p_base * t_acc
            + p_base * t_cruise
            + self.drone.power_decel_factor * p_base * t_dec
        )
        return energy_ws / 3600.0
    def reagent_consumed(self, distance_m):
        return self.drone.spray_flow_rate_lpm * self.time_straight(distance_m) / 60.0
    def time_straight(self, distance_m):
        t_acc, t_cruise, t_dec, _ = self._straight_profile(
            distance_m,
            self.drone.speed_cruise_ms,
            self.drone.accel_horizontal_ms2,
            self.drone.decel_horizontal_ms2,
        )
        return t_acc + t_cruise + t_dec
    def time_turn(self, angle_deg):
        return self.drone.turn_duration_s * (angle_deg / 180.0)
    def time_transit(self, distance_m):
        t_acc, t_cruise, t_dec, _ = self._straight_profile(
            distance_m,
            self.drone.speed_max_ms,
            self.drone.accel_horizontal_ms2,
            self.drone.decel_horizontal_ms2,
        )
        return t_acc + t_cruise + t_dec
    def usable_energy_wh(self):
        return self.drone.battery_capacity_wh * (1.0 - self.drone.battery_reserve_pct / 100.0)
    def reserve_wh_static(self):
        return self.usable_energy_wh() * 0.20
    def reserve_wh_mobile(self):
        return self.usable_energy_wh() * 0.20
    def energy_landing_takeoff(self, reagent_remaining_l):
        height = self.drone.spray_height_m
        vertical_speed = self.drone.speed_vertical_ms
        descent_time = height / vertical_speed
        ascent_time = height / vertical_speed
        power = self.hover_power(reagent_remaining_l)
        return power * (descent_time + ascent_time) / 3600.0
    def energy_to_service_static(self, dist_m, liquid_rem):
        return self.energy_transit(dist_m, liquid_rem)
    def energy_to_service_dynamic(self, dist_m, liquid_rem):
        return self.energy_transit(dist_m, liquid_rem) + self.energy_landing_takeoff(liquid_rem)
    def feasible_after_segment_static(self, energy_rem, liquid_rem, energy_step, liq_step, dist_to_base):
        liquid_after = liquid_rem - liq_step
        e_service = self.energy_to_service_static(dist_to_base, max(liquid_after, 0.0))
        return (
            energy_rem - energy_step - e_service >= self.reserve_wh_static()
            and liquid_after >= 0.0
        )
    def feasible_after_segment_dynamic(self, energy_rem, liquid_rem, energy_step, liq_step, dist_to_rv):
        liquid_after = liquid_rem - liq_step
        e_service = self.energy_to_service_dynamic(dist_to_rv, max(liquid_after, 0.0))
        return (
            energy_rem - energy_step - e_service >= self.reserve_wh_mobile()
            and liquid_after >= 0.0
        )
    def can_continue(self, energy_remaining_wh, reagent_remaining_l, distance_to_rendezvous_m):
        transit_energy = self.energy_transit(distance_to_rendezvous_m, reagent_remaining_l)
        landing_takeoff_energy = self.energy_landing_takeoff(reagent_remaining_l)
        reserve_energy = self.reserve_wh_mobile()
        required_energy = transit_energy + landing_takeoff_energy + reserve_energy
        return energy_remaining_wh >= required_energy and reagent_remaining_l > 0

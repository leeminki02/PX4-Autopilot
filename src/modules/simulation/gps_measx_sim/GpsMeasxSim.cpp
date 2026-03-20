/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "GpsMeasxSim.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/geo/geo.h>
#include <cmath>

using namespace matrix;

// WGS84 Constants
static constexpr double CONSTANTS_R_OF_EARTH = 6378137.0;          // Semi-major axis (a)
static constexpr double CONSTANTS_EARTH_E2 = 6.69437999014e-3;          // Eccentricity squared (e^2)

static const double ms_to_meters = 299792458.0 * 0.001; // speed of light in m/s * 1 ms

/**
 * Convert Geodetic coordinates (Lat, Lon, Alt) to ECEF coordinates (X, Y, Z)
 * * @param lat: Latitude in degrees
 * @param lon: Longitude in degrees
 * @param alt: Altitude in meters (AMSL or Ellipsoid height)
 * @param x, y, z: Output ECEF coordinates in meters
 */
static void map_projection_global_get_ecef(double lat, double lon, double alt, double *x, double *y, double *z)
{
    double lat_rad = math::radians(lat);
    double lon_rad = math::radians(lon);

    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_lon = sin(lon_rad);
    double cos_lon = cos(lon_rad);

    // Prime Vertical Radius of Curvature (N)
    // N = a / sqrt(1 - e^2 * sin^2(lat))
    double N = CONSTANTS_R_OF_EARTH / sqrt(1.0 - CONSTANTS_EARTH_E2 * sin_lat * sin_lat);

    // ECEF Conversion Formula
    *x = (N + alt) * cos_lat * cos_lon;
    *y = (N + alt) * cos_lat * sin_lon;
    *z = (N * (1.0 - CONSTANTS_EARTH_E2) + alt) * sin_lat;
}

GpsMeasxSim::GpsMeasxSim() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

GpsMeasxSim::~GpsMeasxSim()
{
	perf_free(_loop_perf);
}

bool GpsMeasxSim::init()
{
	ScheduleOnInterval(125_ms); // 8 Hz
	return true;
}

float GpsMeasxSim::generate_wgn()
{
	// generate white Gaussian noise sample with std=1

	// algorithm 1:
	// float temp=((float)(rand()+1))/(((float)RAND_MAX+1.0f));
	// return sqrtf(-2.0f*logf(temp))*cosf(2.0f*M_PI_F*rand()/RAND_MAX);
	// algorithm 2: from BlockRandGauss.hpp
	static float V1, V2, S;
	static bool phase = true;
	float X;

	if (phase) {
		do {
			float U1 = (float)rand() / (float)RAND_MAX;
			float U2 = (float)rand() / (float)RAND_MAX;
			V1 = 2.0f * U1 - 1.0f;
			V2 = 2.0f * U2 - 1.0f;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1.0f || fabsf(S) < 1e-8f);

		X = V1 * float(sqrtf(-2.0f * float(logf(S)) / S));

	} else {
		X = V2 * float(sqrtf(-2.0f * float(logf(S)) / S));
	}

	phase = !phase;
	return X;
}

void GpsMeasxSim::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}

	gnss_ephemeris_s eph;
	if (_gnss_ephemeris_in_sub.update(&eph)) {
		_gnss_ephemeris_out_pub.publish(eph);
	}

	bool position_updated = false;
	if (_sim_en_spoof.get() == 0) {
		if (_vehicle_global_position_sub.updated() && _vehicle_local_position_sub.updated()) {
			position_updated = true;
		}
	} else {
		if (_spoofer_global_position_sub.updated() && _spoofer_local_position_sub.updated()) {
			position_updated = true;
		}
	}

	if (position_updated) {
		vehicle_local_position_s lpos{};
		vehicle_global_position_s gpos{};

		if (_sim_en_spoof.get() != 1) {
			// if 0 (benign) or 2 (mixed), use true position for sensor_gps.
			// explanation: for mixed mode, for more dynamic behavior testings, we should make the simulation of valid positioning (but the gnss_raw_measx measurements will be distorted)
			// GPS spoofer simulation disabled: benign behavior
			_vehicle_local_position_sub.copy(&lpos);
			_vehicle_global_position_sub.copy(&gpos);
		} else {
			// GPS spoofer simulation enabled: use spoofer position
			// warn once "spoofer enabled"
			if (!_has_warned_spoofer) {
				PX4_WARN("GPS spoofer simulation enabled");
				_has_warned_spoofer = true;
			}
			_spoofer_local_position_sub.copy(&lpos);
			_spoofer_global_position_sub.copy(&gpos);
		}

		double latitude = gpos.lat + math::degrees((double)generate_wgn() * 0.2 / CONSTANTS_RADIUS_OF_EARTH);
		double longitude = gpos.lon + math::degrees((double)generate_wgn() * 0.2 / CONSTANTS_RADIUS_OF_EARTH);
		double altitude = (double)(gpos.alt + (generate_wgn() * 0.5f));

		Vector3f gps_vel = Vector3f{lpos.vx, lpos.vy, lpos.vz} + noiseGauss3f(0.06f, 0.077f, 0.158f);

		// device id
		device::Device::DeviceId device_id;
		device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
		device_id.devid_s.bus = 0;
		device_id.devid_s.address = 0;
		device_id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;

		sensor_gps_s sensor_gps{};

		if (_sim_gps_used.get() >= 4) {
			// fix
			sensor_gps.fix_type = 3; // 3D fix
			sensor_gps.s_variance_m_s = 0.4f;
			sensor_gps.c_variance_rad = 0.1f;
			sensor_gps.eph = 0.9f;
			sensor_gps.epv = 1.78f;
			sensor_gps.hdop = 0.7f;
			sensor_gps.vdop = 1.1f;

		} else {
			// no fix
			sensor_gps.fix_type = 0; // No fix
			sensor_gps.s_variance_m_s = 100.f;
			sensor_gps.c_variance_rad = 100.f;
			sensor_gps.eph = 100.f;
			sensor_gps.epv = 100.f;
			sensor_gps.hdop = 100.f;
			sensor_gps.vdop = 100.f;
		}

		sensor_gps.timestamp_sample = gpos.timestamp_sample;
		sensor_gps.time_utc_usec = 0;
		sensor_gps.device_id = device_id.devid;
		sensor_gps.latitude_deg = latitude; // Latitude in degrees
		sensor_gps.longitude_deg = longitude; // Longitude in degrees
		sensor_gps.altitude_msl_m = altitude; // Altitude in meters above MSL
		sensor_gps.altitude_ellipsoid_m = altitude;
		sensor_gps.noise_per_ms = 0;
		sensor_gps.jamming_indicator = 0;
		sensor_gps.vel_m_s = sqrtf(gps_vel(0) * gps_vel(0) + gps_vel(1) * gps_vel(1)); // GPS ground speed, (metres/sec)
		sensor_gps.vel_n_m_s = gps_vel(0);
		sensor_gps.vel_e_m_s = gps_vel(1);
		sensor_gps.vel_d_m_s = gps_vel(2);
		sensor_gps.cog_rad = atan2(gps_vel(1),
						gps_vel(0)); // Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
		sensor_gps.timestamp_time_relative = 0;
		sensor_gps.heading = NAN;
		sensor_gps.heading_offset = NAN;
		sensor_gps.heading_accuracy = 0;
		sensor_gps.automatic_gain_control = 0;
		sensor_gps.jamming_state = 0;
		sensor_gps.spoofing_state = 0;
		sensor_gps.vel_ned_valid = true;
		sensor_gps.satellites_used = _sim_gps_used.get();

		sensor_gps.timestamp = hrt_absolute_time();
		_sensor_gps_pub.publish(sensor_gps);

	}
	
	if (_satellite_ecef_sub.updated()) { // I hope this is always true when we reach here
		/* signal property simulator */

		/* get satellite ecef positions from _satellite_ecef_sub */
		satellite_ecef_s sat_ecef{};
		_satellite_ecef_sub.copy(&sat_ecef);
		// TODO: calculate doppler shifts with noise using groundtruth positions
		// PX4_INFO("Received %d satellites from satellite_ecef_groundtruth", sat_ecef.count);

		/* get groundtruth positions */
		vehicle_global_position_s gpos_truth{};
		vehicle_local_position_s lpos_truth{};
		_vehicle_global_position_sub.copy(&gpos_truth);
		_vehicle_local_position_sub.copy(&lpos_truth);

		double drone_x, drone_y, drone_z;
		map_projection_global_get_ecef(gpos_truth.lat, gpos_truth.lon, gpos_truth.alt,
					      		&drone_x, &drone_y, &drone_z);
		Vector3d p_drone(drone_x, drone_y, drone_z);

		// transform velocity to ecef
		double lat_rad = math::radians(gpos_truth.lat);
		double lon_rad = math::radians(gpos_truth.lon);
		float sin_lat = sin(lat_rad);
		float cos_lat = cos(lat_rad);
		float sin_lon = sin(lon_rad);
		float cos_lon = cos(lon_rad);

		Matrix3f R_ned_to_ecef;
		R_ned_to_ecef(0, 0) = -sin_lat * cos_lon;
		R_ned_to_ecef(0, 1) = -sin_lon;
		R_ned_to_ecef(0, 2) = -cos_lat * cos_lon;
		R_ned_to_ecef(1, 0) = -sin_lat * sin_lon;
		R_ned_to_ecef(1, 1) =  cos_lon;
		R_ned_to_ecef(1, 2) = -cos_lat * sin_lon;
		R_ned_to_ecef(2, 0) =  cos_lat;
		R_ned_to_ecef(2, 1) =  0.0f;
		R_ned_to_ecef(2, 2) = -sin_lat;

		Vector3f v_ned(lpos_truth.vx, lpos_truth.vy, lpos_truth.vz);
		Vector3f v_drone_f = R_ned_to_ecef * v_ned;
		Vector3d v_drone(v_drone_f(0), v_drone_f(1), v_drone_f(2));

		/* load spoofer position as ecef */
		int8_t sim_en_spoof = _sim_en_spoof.get();
		bool is_benign = (_sim_en_spoof.get() == 0);
		Vector3d p_emitter(0.0, 0.0, 0.0);
		Vector3d v_emitter(0.0, 0.0, 0.0);

		if (!is_benign) {
			vehicle_global_position_s gpos_spoofer{};
			vehicle_local_position_s lpos_spoofer{};
			_spoofer_global_position_sub.copy(&gpos_spoofer);
			_spoofer_local_position_sub.copy(&lpos_spoofer);

			double spoofer_x, spoofer_y, spoofer_z;
			map_projection_global_get_ecef(gpos_spoofer.lat, gpos_spoofer.lon, gpos_spoofer.alt,
						       &spoofer_x, &spoofer_y, &spoofer_z);
			p_emitter = Vector3d(spoofer_x, spoofer_y, spoofer_z);

			// transform velocity from NED to ecef
			Vector3f v_ned_spoofer(lpos_spoofer.vx, lpos_spoofer.vy, lpos_spoofer.vz);
			Vector3f v_spoofer_f = R_ned_to_ecef * v_ned_spoofer;
			v_emitter = Vector3d(v_spoofer_f(0), v_spoofer_f(1), v_spoofer_f(2));
		}

		/* calculate doppler */
		gnss_raw_measx_s gnss_raw_measx{};
		gnss_raw_measx.timestamp = hrt_absolute_time();

		int count = 0;
		// if sim_en_spoof is 2 (mixed), create a spoofing effect on half of the satellites and benign on the other half
		// the selection of which satellites are spoofed: even svid-ed satellites are simulated as spoofed.
		
		for (int i = 0; i < sat_ecef.count; i++) {
			Vector3d p_sat(sat_ecef.p_x[i], sat_ecef.p_y[i], sat_ecef.p_z[i]);
			Vector3d v_sat(sat_ecef.v_x[i], sat_ecef.v_y[i], sat_ecef.v_z[i]);

			double doppler_total = 0.0;
			const double lambda = 0.19029367279836487; // TODO: check L1 frequency wavelength
			
			if (is_benign || (sim_en_spoof == 2 && (sat_ecef.svid[i] % 2 != 0))) { 
				// benign observation
				// benign doppler: doppler_total = doppler_sat_drone
				Vector3d rel_vel = v_sat - v_drone;
				Vector3d u_los = (p_sat - p_drone).normalized(); // line-of-sight unit vector
				doppler_total = - (rel_vel.dot(u_los)) / lambda;
			} else { 
				// spoofed observation
				// spoofed doppler: doppler_total = doppler_sat_emitter + doppler_emitter_drone
				// 1. doppler_sim = doppler_sat_emitter
				Vector3d rel_vel = v_sat - v_emitter;
				Vector3d u_los = (p_sat - p_emitter).normalized(); // line-of-sight unit vector
				double doppler_sim = - (rel_vel.dot(u_los)) / lambda;

				// 2. doppler_phy = doppler_emitter_drone (the doppler shift caused by spoofer and drone's physical motion)
				Vector3d rel_vel_phy = v_emitter - v_drone;
				Vector3d u_los_phy = (p_emitter - p_drone).normalized(); // line-of-sight unit vector
				double doppler_phy = - (rel_vel_phy.dot(u_los_phy)) / lambda;

				doppler_total = doppler_sim + doppler_phy;
			}
			
			// add noise
			doppler_total += (double)generate_wgn() * 0.5; // 0.5 Hz std dev
			// int32[24] dopplerms		# Doppler Measurement (m/s) [*0.04 m/s]
			int32_t doppler_ms = (int32_t)round(doppler_total / 0.04 * lambda);
			// int32[24] dopplerhz		# Doppler Measurement [*0.2  Hz]
			int32_t doppler_hz = (int32_t)round(doppler_total / 0.2);

			double true_range = 0.0;

			if (is_benign || (sim_en_spoof == 2 && (sat_ecef.svid[i] % 2 != 0))) {
				true_range = (p_sat - p_drone).norm();
			} else {
				double range_sim = (p_sat - p_emitter).norm(); 
				double range_phy = (p_emitter - p_drone).norm();
				true_range = range_sim + range_phy;
			}

			true_range += (double)generate_wgn() * 2.0;

			double remainder_dist = fmod(true_range, ms_to_meters);
			
			double total_chips = (remainder_dist / ms_to_meters) * 1023.0;

			uint16_t whole_chips = (uint16_t)floor(total_chips);
			uint16_t frac_chips = (uint16_t)round((total_chips - whole_chips) * 1024.0);

			if (whole_chips >= 1023) { whole_chips = 1022; }
			if (frac_chips >= 1024) { frac_chips = 1023; }

			// populate
			gnss_raw_measx.cno[i] = 45; // dummy value for now.
			gnss_raw_measx.gnssid[i] = 0; // they're all GPS satellites.
			gnss_raw_measx.svid[i] = sat_ecef.svid[i];
			gnss_raw_measx.dopplerms[i] = doppler_ms;
			gnss_raw_measx.dopplerhz[i] = doppler_hz;
			gnss_raw_measx.wholechips[i] = whole_chips;
			gnss_raw_measx.fracchips[i] = frac_chips;

			count++;
			if (count >= gnss_raw_measx.GNSS_MEASX_MAX_SATELLITES) {
				break;
			}
		}
		gnss_raw_measx.count = count;

		_gnss_raw_measx_pub.publish(gnss_raw_measx);
	}

	perf_end(_loop_perf);
}

int GpsMeasxSim::task_spawn(int argc, char *argv[])
{
	GpsMeasxSim *instance = new GpsMeasxSim();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int GpsMeasxSim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GpsMeasxSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gps_measx_sim", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gps_measx_sim_main(int argc, char *argv[])
{
	return GpsMeasxSim::main(argc, argv);
}

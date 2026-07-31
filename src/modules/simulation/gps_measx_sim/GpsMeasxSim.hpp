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

#pragma once

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/gnss_ephemeris.h>
#include <uORB/topics/gnss_raw_measx.h>
#include <uORB/topics/satellite_ecef.h>


using namespace time_literals;

class GpsMeasxSim : public ModuleBase<GpsMeasxSim>, public ModuleParams, public px4::WorkItem
{
public:
	GpsMeasxSim();
	~GpsMeasxSim() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();
	bool _has_warned_spoofer = false;

private:
	void Run() override;

	bool log_received = false;
	int prev_sim_en_spoof = -1; //< represents unset


	// generate white Gaussian noise sample with std=1
	static float generate_wgn();

	// generate white Gaussian noise sample as a 3D vector with specified std
	matrix::Vector3f noiseGauss3f(float stdx, float stdy, float stdz) { return matrix::Vector3f(generate_wgn() * stdx, generate_wgn() * stdy, generate_wgn() * stdz); }

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position_groundtruth)};
	uORB::PublicationMulti<sensor_gps_s> _sensor_gps_pub{ORB_ID(sensor_gps)};

	uORB::Subscription _spoofer_global_position_sub{ORB_ID(spoofer_global_position)};
	uORB::Subscription _spoofer_local_position_sub{ORB_ID(spoofer_local_position)};
	// Run() is triggered directly by this arriving, mirroring how a real receiver latches all
	// channels off one common epoch instead of being polled by an independent timer.
	uORB::SubscriptionCallbackWorkItem _satellite_ecef_sub{this, ORB_ID(satellite_ecef)};
	uORB::Subscription _gnss_ephemeris_in_sub{ORB_ID(gnss_ephemeris_in)};
	uORB::PublicationMulti<gnss_ephemeris_s> _gnss_ephemeris_out_pub{ORB_ID(gnss_ephemeris_out)};
	uORB::PublicationMulti<gnss_raw_measx_s> _gnss_raw_measx_pub{ORB_ID(gnss_raw_measx)};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	// Diagnostics only: how tightly satellite_ecef arrivals actually drive Run(), and how stale the
	// sample is by the time it's processed here. See sync_diagnostics section in Run().
	perf_counter_t _sat_ecef_trigger_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": sat_ecef trigger interval")};
	hrt_abstime _hrt_to_realtime_offset_us{0}; //< one-shot calibration; see init()
	uint32_t _sync_dbg_count{0};
	int64_t _sync_dbg_latency_sum_us{0};
	int64_t _sync_dbg_latency_max_us{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SIM_GPS_USED>) _sim_gps_used,
		(ParamInt<px4::params::SIM_EN_SPOOF>) _sim_en_spoof
	)
};

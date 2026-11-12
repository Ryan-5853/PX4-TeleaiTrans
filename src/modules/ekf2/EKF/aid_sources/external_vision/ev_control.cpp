/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file ev_control.cpp
 * Control functions for ekf external vision control
 */

#include "ekf.h"
#include "aid_sources/external_vision/ev_vel.h"

void Ekf::controlExternalVisionFusion(const imuSample &imu_sample)
{
	_ev_pos_b_est.predict(_dt_ekf_avg);
	_ev_hgt_b_est.predict(_dt_ekf_avg);

	// Check for new external vision data
	extVisionSample ev_sample;

	if (_ext_vision_buffer && _ext_vision_buffer->pop_first_older_than(imu_sample.time_us, &ev_sample)) {

		bool ev_reset = (ev_sample.reset_counter != _ev_sample_prev.reset_counter);

		// determine if we should use the horizontal position observations
		bool quality_sufficient = (_params.ev_quality_minimum <= 0) || (ev_sample.quality >= _params.ev_quality_minimum);

		const bool starting_conditions_passing = quality_sufficient
				&& ((ev_sample.time_us - _ev_sample_prev.time_us) < EV_MAX_INTERVAL)
				&& ((_params.ev_quality_minimum <= 0)
				    || (_ev_sample_prev.quality >= _params.ev_quality_minimum)) // previous quality sufficient
				&& ((_params.ev_quality_minimum <= 0)
				    || (_ext_vision_buffer->get_newest().quality >= _params.ev_quality_minimum)) // newest quality sufficient
				&& isNewestSampleRecent(_time_last_ext_vision_buffer_push, EV_MAX_INTERVAL);

		updateEvAttitudeErrorFilter(ev_sample, ev_reset);

		if (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::ATTITUDE)) {
			controlEvAttitudeFusion(ev_sample, starting_conditions_passing, ev_reset, quality_sufficient);
		} else {
			if (_ev_att_active) { stopEvYawFusion(); _ev_att_active = false; }
			controlEvYawFusion(imu_sample, ev_sample, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_yaw);
		}

		switch (ev_sample.vel_frame) {
		case VelocityFrame::BODY_FRAME_FRD: {
				EvVelBodyFrameFrd ev_vel_body(*this, ev_sample, _params.ev_vel_noise, imu_sample);
				controlEvVelFusion(ev_vel_body, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_vel);
				break;
			}

		case VelocityFrame::LOCAL_FRAME_NED: {
				EvVelLocalFrameNed ev_vel_ned(*this, ev_sample, _params.ev_vel_noise, imu_sample);
				controlEvVelFusion(ev_vel_ned, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_vel);
				break;
			}

		case VelocityFrame::LOCAL_FRAME_FRD: {
				EvVelLocalFrameFrd ev_vel_frd(*this, ev_sample, _params.ev_vel_noise, imu_sample);
				controlEvVelFusion(ev_vel_frd, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_vel);
				break;
			}

		default:
			break;
		}

		controlEvPosFusion(imu_sample, ev_sample, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_pos);
		controlEvHeightFusion(imu_sample, ev_sample, starting_conditions_passing, ev_reset, quality_sufficient,
				      _aid_src_ev_hgt);

		if (quality_sufficient) {
			_ev_sample_prev = ev_sample;
		}

	} else if ((_control_status.flags.ev_pos || _control_status.flags.ev_vel || _control_status.flags.ev_yaw
		    || _control_status.flags.ev_hgt)
		   && isTimedOut(_ev_sample_prev.time_us, 2 * EV_MAX_INTERVAL)) {

		// Turn off EV fusion mode if no data has been received
		stopEvPosFusion();
		stopEvVelFusion();
		stopEvYawFusion();
		_ev_att_active = false;
		_aid_src_ev_att.fused = false;
		stopEvHgtFusion();

		_ev_q_error_initialized = false;

		ECL_WARN("vision data stopped");
	}
}

void Ekf::updateEvAttitudeErrorFilter(extVisionSample &ev_sample, bool ev_reset)
{
	const Quatf q_error((_state.quat_nominal * ev_sample.quat.inversed()).normalized());

	if (!q_error.isAllFinite()) {
		return;
	}

	if (!_ev_q_error_initialized || ev_reset) {
		_ev_q_error_filt.reset(q_error);
		_ev_q_error_initialized = true;

	} else {
		_ev_q_error_filt.update(q_error);
	}
}

namespace
{
// PX4 attitude errors are left-multiplicative, expressed in the local frame.
Vector3f evRotationResidual(const Quatf &predicted, const Quatf &observed)
{
	Quatf error = (predicted * observed.inversed()).normalized();

	if (error(0) < 0.f) { error = -error; }

	return matrix::AxisAnglef(error);
}

matrix::SquareMatrix3f evRotationJacobian(const Quatf &predicted, const Quatf &observed)
{
	// Central differences of the SO(3) log avoid Euler singularities. Only the
	// three attitude-error states are nonzero; use the same left perturbation as fuse().
	matrix::SquareMatrix3f H;
	constexpr float epsilon = 1e-3f;

	for (int axis = 0; axis < 3; ++axis) {
		Vector3f delta{};
		delta(axis) = epsilon;
		const Vector3f plus = evRotationResidual(Quatf(matrix::AxisAnglef(delta)) * predicted, observed);
		const Vector3f minus = evRotationResidual(Quatf(matrix::AxisAnglef(-delta)) * predicted, observed);
		H.col(axis) = (plus - minus) / (2.f * epsilon);
	}

	return H;
}
}

void Ekf::controlEvAttitudeFusion(const extVisionSample &sample, bool starting, bool reset, bool quality)
{
	auto &aid = _aid_src_ev_att;
	aid.fused = false;
	const bool frame_valid = sample.pos_frame == PositionFrame::LOCAL_FRAME_NED
				 || sample.pos_frame == PositionFrame::LOCAL_FRAME_FRD;
	const bool frame_conflict = sample.pos_frame != PositionFrame::LOCAL_FRAME_NED
				    && (_control_status.flags.gnss_pos || _control_status.flags.gnss_vel);

	if (!quality || !frame_valid || frame_conflict || !_control_status.flags.tilt_align
	    || !sample.quat.isAllFinite() || !sample.orientation_var.isAllFinite() || !starting
	    || (reset && _control_status.flags.in_air)) {
		if (_ev_att_active) { stopEvYawFusion(); }
		_ev_att_active = false;
		aid.innovation_rejected = true;
		return;
	}

	const float variance = math::max(sample.orientation_var.max(), sq(_params.ev_att_noise), sq(0.01f));

	if (!_ev_att_active) {
		// Switch off yaw-only fusion before taking ownership of its heading-aiding flag.
		stopEvYawFusion();

		if (!_control_status.flags.in_air && sample.pos_frame == PositionFrame::LOCAL_FRAME_FRD) {
			resetQuatStateYaw(getEulerYaw(sample.quat), variance);
			_control_status.flags.yaw_align = false; // arbitrary heading, not geographic north
		}
	}

	const Vector3f innovation = evRotationResidual(_state.quat_nominal, sample.quat);
	const auto H_att = evRotationJacobian(_state.quat_nominal, sample.quat);
	Vector3f innovation_variance;

	for (int axis = 0; axis < 3; ++axis) {
		VectorState H{};
		for (int j = 0; j < 3; ++j) { H(State::quat_nominal.idx + j) = H_att(axis, j); }
		innovation_variance(axis) = H.dot(P * H) + variance;
	}

	updateAidSourceStatus(aid, sample.time_us, Vector3f(matrix::AxisAnglef(sample.quat)),
			      Vector3f(variance, variance, variance), innovation, innovation_variance,
			      math::max(_params.heading_innov_gate, 1.f));

	// Never follow a rigid-body flip/rebinding or a large in-flight attitude jump.
	if (innovation.norm() > math::radians(30.f) || aid.innovation_rejected) {
		aid.innovation_rejected = true;

		if (_ev_att_active && isTimedOut(aid.time_last_fuse, EV_MAX_INTERVAL)) {
			stopEvYawFusion();
			_ev_att_active = false;
		}

		return;
	}

	for (int axis = 0; axis < 3; ++axis) {
		// Relinearize at the state corrected by preceding scalar updates.
		const auto H_current = evRotationJacobian(_state.quat_nominal, sample.quat);
		const Vector3f residual = evRotationResidual(_state.quat_nominal, sample.quat);
		VectorState H{};
		for (int j = 0; j < 3; ++j) { H(State::quat_nominal.idx + j) = H_current(axis, j); }
		const VectorState PH = P * H;
		const float S = H.dot(PH) + variance;
		VectorState K = PH / S;
		measurementUpdate(K, H, variance, residual(axis));
	}

	aid.fused = true;
	aid.time_last_fuse = _time_delayed_us;
	_ev_att_active = true;
	// Reuse the existing external-heading flag to inhibit competing magnetic yaw fusion.
	_control_status.flags.ev_yaw = true;

	if (sample.pos_frame == PositionFrame::LOCAL_FRAME_NED) { _control_status.flags.yaw_align = true; }
}

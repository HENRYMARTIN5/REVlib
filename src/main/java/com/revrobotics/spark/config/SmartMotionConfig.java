/*
 * Copyright (c) 2024 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package com.revrobotics.spark.config;

import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.ClosedLoopSlot;

public class SmartMotionConfig extends BaseConfig {
  /** Create a new object to configure SmartMotion. */
  public SmartMotionConfig() {
    super(CANType.kSpark);
  }

  /**
   * Applies settings from another {@link SmartMotionConfig} to this one.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link SmartMotionConfig} to copy settings from
   * @return The updated {@link SmartMotionConfig} for method chaining
   */
  public SmartMotionConfig apply(SmartMotionConfig config) {
    super.apply(config);
    return this;
  }

  /**
   * Set the maximum velocity for the Smart Motion mode of the controller. This is the cruising
   * velocity of the profile. Natively, the units are in RPM but will be affected by the velocity
   * conversion factor.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link SmartMotionConfig#maxVelocity(double, ClosedLoopSlot)}.
   *
   * @param maxVelocity The maximum velocity with the velocity conversion factor applied
   * @return The modified {@link SmartMotionConfig} object for method chaining
   * @deprecated It is recommended to migrate to MAXMotion instead.
   */
  @Deprecated(forRemoval = true)
  public SmartMotionConfig maxVelocity(double maxVelocity) {
    return maxVelocity(maxVelocity, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the maximum velocity for the Smart Motion mode of the controller for a specific closed loop
   * slot. This is the cruising velocity of the profile. Natively, the units are in RPM but will be
   * affected by the velocity conversion factor.
   *
   * @param maxVelocity The maximum velocity with the velocity conversion factor applied
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link SmartMotionConfig} object for method chaining
   * @deprecated It is recommended to migrate to MAXMotion instead.
   */
  @Deprecated(forRemoval = true)
  public SmartMotionConfig maxVelocity(double maxVelocity, ClosedLoopSlot slot) {
    putParameter(
        SparkParameter.kSmartMotionMaxVelocity_0.value + slot.value * 4, (float) maxVelocity);
    return this;
  }

  /**
   * Set the maximum acceleration for the Smart Motion mode of the controller. This is the rate at
   * which the velocity will increase until the max velocity is reached. Natively, the units are in
   * RPM per second but will be affected by the velocity conversion factor.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link SmartMotionConfig#maxAcceleration(double, ClosedLoopSlot)}.
   *
   * @param maxAcceleration The maximum acceleration with the velocity conversion factor applied
   * @return The modified {@link SmartMotionConfig} object for method chaining
   * @deprecated It is recommended to migrate to MAXMotion instead.
   */
  @Deprecated(forRemoval = true)
  public SmartMotionConfig maxAcceleration(double maxAcceleration) {
    return maxAcceleration(maxAcceleration, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the maximum acceleration for the Smart Motion mode of the controller for a specific PID
   * slot. This is the rate at which the velocity will increase until the max velocity is reached.
   * Natively, the units are in RPM per second but will be affected by the velocity conversion
   * factor.
   *
   * @param maxAcceleration The maximum acceleration with the velocity conversion factor applied
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link SmartMotionConfig} object for method chaining
   * @deprecated It is recommended to migrate to MAXMotion instead.
   */
  @Deprecated(forRemoval = true)
  public SmartMotionConfig maxAcceleration(double maxAcceleration, ClosedLoopSlot slot) {
    putParameter(
        SparkParameter.kSmartMotionMaxAccel_0.value + slot.value * 4, (float) maxAcceleration);
    return this;
  }

  /**
   * Set the minimum velocity for the Smart Motion mode of the controller. Any requested velocities
   * below this value will be set to 0. Natively, the units are in RPM but will be affected by the
   * velocity conversion factor.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link SmartMotionConfig#minOutputVelocity(double, ClosedLoopSlot)}.
   *
   * @param minVelocity The minimum velocity with the velocity conversion factor applied
   * @return The modified {@link SmartMotionConfig} object for method chaining
   * @deprecated It is recommended to migrate to MAXMotion instead.
   */
  @Deprecated(forRemoval = true)
  public SmartMotionConfig minOutputVelocity(double minVelocity) {
    return minOutputVelocity(minVelocity, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the minimum velocity for the Smart Motion mode of the controller for a specific closed loop
   * slot. Any requested velocities below this value will be set to 0. Natively, the units are in
   * RPM but will be affected by the velocity conversion factor.
   *
   * @param minVelocity The minimum velocity with the velocity conversion factor applied
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link SmartMotionConfig} object for method chaining
   * @deprecated It is recommended to migrate to MAXMotion instead.
   */
  @Deprecated(forRemoval = true)
  public SmartMotionConfig minOutputVelocity(double minVelocity, ClosedLoopSlot slot) {
    putParameter(
        SparkParameter.kSmartMotionMinVelOutput_0.value + slot.value * 4, (float) minVelocity);
    return this;
  }

  /**
   * Set the allowed closed loop error for the Smart Motion mode of the controller. This value is
   * how much deviation from the setpoint is tolerated and is useful in preventing oscillation
   * around the setpoint. Natively, the units are in rotations but will be affected by the position
   * conversion factor.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link SmartMotionConfig#allowedClosedLoopError(double, ClosedLoopSlot)}.
   *
   * @param allowedError The allowed error with the position conversion factor applied
   * @return The modified {@link SmartMotionConfig} object for method chaining
   * @deprecated It is recommended to migrate to MAXMotion instead.
   */
  @Deprecated(forRemoval = true)
  public SmartMotionConfig allowedClosedLoopError(double allowedError) {
    return allowedClosedLoopError(allowedError, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the allowed closed loop error for the Smart Motion mode of the controller for a specific
   * closed loop slot. This value is how much deviation from the setpoint is tolerated and is useful
   * in preventing oscillation around the setpoint. Natively, the units are in rotations but will be
   * affected by the position conversion factor.
   *
   * @param allowedError The allowed error with the position conversion factor applied
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link SmartMotionConfig} object for method chaining
   * @deprecated It is recommended to migrate to MAXMotion instead.
   */
  @Deprecated(forRemoval = true)
  public SmartMotionConfig allowedClosedLoopError(double allowedError, ClosedLoopSlot slot) {
    putParameter(
        SparkParameter.kSmartMotionAllowedClosedLoopError_0.value + slot.value * 4,
        (float) allowedError);
    return this;
  }
}

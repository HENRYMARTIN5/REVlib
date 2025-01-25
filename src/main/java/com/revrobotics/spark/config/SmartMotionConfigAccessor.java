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

import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.ClosedLoopSlot;

public class SmartMotionConfigAccessor {
  private final long sparkHandle;

  protected SmartMotionConfigAccessor(long sparkHandle) {
    this.sparkHandle = sparkHandle;
  }

  public double getMaxVelocity() {
    return getMaxVelocity(ClosedLoopSlot.kSlot0);
  }

  public double getMaxAcceleration() {
    return getMaxAcceleration(ClosedLoopSlot.kSlot0);
  }

  public double getMinOutputVelocity() {
    return getMinOutputVelocity(ClosedLoopSlot.kSlot0);
  }

  public double getAllowedClosedLoopError() {
    return getAllowedClosedLoopError(ClosedLoopSlot.kSlot0);
  }

  public double getMaxVelocity(ClosedLoopSlot slot) {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameter.kSmartMotionMaxVelocity_0.value + slot.value * 4);
  }

  public double getMaxAcceleration(ClosedLoopSlot slot) {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameter.kSmartMotionMaxAccel_0.value + slot.value * 4);
  }

  public double getMinOutputVelocity(ClosedLoopSlot slot) {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameter.kSmartMotionMinVelOutput_0.value + slot.value * 4);
  }

  public double getAllowedClosedLoopError(ClosedLoopSlot slot) {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameter.kSmartMotionAllowedClosedLoopError_0.value + slot.value * 4);
  }
}

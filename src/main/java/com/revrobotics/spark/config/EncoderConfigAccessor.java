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

public class EncoderConfigAccessor {
  private final long sparkHandle;

  protected EncoderConfigAccessor(long sparkHandle) {
    this.sparkHandle = sparkHandle;
  }

  public int getCountsPerRevolution() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
        sparkHandle, SparkParameter.kEncoderCountsPerRev.value);
  }

  public boolean getInverted() {
    return CANSparkJNI.c_Spark_GetParameterBool(sparkHandle, SparkParameter.kEncoderInverted.value);
  }

  public double getPositionConversionFactor() {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameter.kPositionConversionFactor.value);
  }

  public double getVelocityConversionFactor() {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameter.kVelocityConversionFactor.value);
  }

  public int getQuadratureAverageDepth() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
        sparkHandle, SparkParameter.kEncoderAverageDepth.value);
  }

  public int getQuadratureMeasurementPeriod() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kEncoderSampleDelta.value)
        >> 1;
  }

  public int getUvwAverageDepth() {
    int value =
        CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kHallSensorAverageDepth.value);

    return 1 << value;
  }

  public int getUvwMeasurementPeriod() {
    float value =
        CANSparkJNI.c_Spark_GetParameterFloat32(
            sparkHandle, SparkParameter.kHallSensorSampleRate.value);
    return (int) (value * 1000);
  }
}
